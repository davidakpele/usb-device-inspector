"""Military-grade 3D drone renderer — drone_3d_view.py

Design language: tactical operations centre / UAV ground-control station.

Environment
-----------
* Deep night-operations sky with star field and horizon glow
* Large tactical grid (5 m spacing) with 50 m sector labels
* Terrain surface: dark olive/grey with subtle texture lines
* Runway-style centre cross-hair at origin
* Range rings (25 m, 50 m, 100 m) from origin — tactical distance markers
* Sector boundary markers every 45° on the outer ring

Drone — MQ-style quadrotor UAV
-------------------------------
* Larger body: central fuselage + 4 carbon-fibre booms (ARM = 0.75 m)
* Central electronics pod: hexagonal body with dome canopy
* Camera gimbal: small sphere below nose
* Landing gear: 4 retractable legs (shown on ground, retracted in flight)
* Navigation lights: bright green (front-left), red (front-right),
  white strobes (rear) — standard aviation lighting
* Rotor guards: thin ring around each rotor disc
* Rotor discs: large (R = 0.55 m), multi-blade with blur at speed

HUD — tactical GCS style
-------------------------
* Top-centre: threat/mode badge in military amber/green
 * Top-left: full telemetry panel with MGRS-style position
* Top-right: digital compass rose with bearing tape
* Bottom-left: artificial horizon (ADI) + FPV reticle
* Bottom-centre: flight command with icon
* Bottom-right: altitude ladder + speed tape (dual vertical bars)
* Right edge: threat warning / mode alert strip
"""
from __future__ import annotations

import math
import random
from collections import deque

from PySide6.QtCore import QPointF, QRectF, Qt
from PySide6.QtGui import (
    QColor, QFont, QFontMetrics, QLinearGradient,
    QPainter, QPainterPath, QPen, QBrush,
    QPolygonF, QRadialGradient,
)
from PySide6.QtWidgets import QWidget

from app.core.drone_physics import DroneState, FlightMode

# ===========================================================================
# Scene constants
# ===========================================================================

SCALE     = 90.0    # pixels per metre — larger world feels bigger
TRAIL_LEN = 300     # longer trail for tactical situational awareness

# ---------------------------------------------------------------------------
# Tactical colour palette — military amber/green-on-black
# ---------------------------------------------------------------------------

_C_BG       = QColor(6,   8,  12)      # near-black background
_C_SKY_MID  = QColor(8,  14,  24)
_C_SKY_HRZ  = QColor(18,  32,  22)    # horizon: night-vision green tint
_C_GRID     = QColor(32,  52,  32, 80)  # dark tactical green grid
_C_GRID_AX  = QColor(40,  90,  40, 140) # axis lines brighter
_C_TERRAIN  = QColor(18,  28,  14)     # dark olive terrain base
_C_AMBER    = QColor(255, 176,  0)     # primary HUD amber
_C_GREEN    = QColor( 40, 220,  80)    # tactical green
_C_RED      = QColor(220,  40,  40)    # warning red
_C_BLUE     = QColor( 60, 160, 240)    # info blue
_C_WHITE    = QColor(230, 240, 220)    # slightly warm white
_C_DIM      = QColor( 80, 100,  80)    # dim label colour

# Drone colours
_C_BOOM     = QColor(30,  40,  35)     # carbon fibre boom — near black
_C_BOOM_HI  = QColor(60,  80,  60)    # boom highlight edge
_C_BODY_D   = QColor(22,  30,  22)    # body dark
_C_BODY_L   = QColor(50,  70,  50)    # body panel lines
_C_DOME     = QColor(30,  60, 100, 200) # camera dome: dark blue glass
_C_GUARD    = QColor(45,  55,  45, 160) # rotor guard ring
_C_ROTOR    = QColor(50,  80,  50, 190) # rotor disc: dark
_C_BLADE    = QColor(100, 160, 100, 230)# blade line
_C_NAV_GRN  = QColor( 0,  255,  80, 240) # port nav light (front-left)
_C_NAV_RED  = QColor(255,  30,  30, 240) # starboard nav light (front-right)
_C_STROBE   = QColor(255, 255, 220, 240) # rear strobe

# Mode colours
_MODE_RGB: dict[str, tuple[int,int,int]] = {
    "DISARMED":  ( 60,  70,  60),
    "ARMED":     (200, 160,   0),
    "HOVER":     ( 40, 180,  80),
    "SPORT":     (220,  60,  40),
    "PRECISION": ( 40, 160, 220),
    "LANDING":   (180, 140,   0),
    "TAKEOFF":   ( 40, 200, 120),
}

# Pre-built pens
_PN = Qt.PenStyle.NoPen


# ===========================================================================
# Projection
# ===========================================================================

def _proj(wx: float, wy: float, wz: float,
          cx: float, cy: float,
          cc: float, sc: float,
          scale: float = 90.0) -> tuple[float, float]:
    """Oblique projection — nearly top-down so the ground looks FLAT/LEVEL.

    The Z depth compression (rz * scale * DEPTH_X / DEPTH_Y) determines
    how much the ground plane tilts away from horizontal:
      0.0 / 0.0  = true top-down (perfectly flat, no 3D feel)
      0.15/ 0.08 = very flat — ground is nearly horizontal, slight depth
      0.45/ 0.22 = strong oblique (old value — ground looks tilted)

    We use a shallow oblique so the ground feels level while still giving
    enough 3D perspective to judge altitude.
    """
    rx =  wx * cc + wz * sc
    rz = -wx * sc + wz * cc
    # DEPTH_X = how much Z offset shifts the screen horizontally
    # DEPTH_Y = how much Z offset shifts the screen vertically
    # Smaller values → flatter / more horizontal-looking ground
    DEPTH_X = 0.18
    DEPTH_Y = 0.09
    return (cx + rx * scale - rz * scale * DEPTH_X,
            cy - wy * scale + rz * scale * DEPTH_Y)


def _qpt(wx, wy, wz, cx, cy, cc, sc, scale=90.0) -> QPointF:
    sx, sy = _proj(wx, wy, wz, cx, cy, cc, sc, scale)
    return QPointF(sx, sy)


def _arrow_head(p: QPainter, tip: QPointF, base: QPointF,
                head_len=14, head_w=7) -> None:
    dx = tip.x()-base.x(); dy = tip.y()-base.y()
    ln = math.sqrt(dx*dx+dy*dy)
    if ln < 1:
        return
    ux, uy = dx/ln, dy/ln
    px, py = -uy, ux
    poly = QPolygonF([
        tip,
        QPointF(tip.x()-ux*head_len+px*head_w, tip.y()-uy*head_len+py*head_w),
        QPointF(tip.x()-ux*head_len-px*head_w, tip.y()-uy*head_len-py*head_w),
    ])
    p.setPen(_PN); p.drawPolygon(poly)


# ===========================================================================
# Widget
# ===========================================================================

class Drone3DWidget(QWidget):
    """Military-grade tactical UAV ground-control display."""

    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self._state = DroneState()
        self._trail: deque[tuple[float,float,float]] = deque(maxlen=TRAIL_LEN)
        self._cam_yaw = 28.0
        rad = math.radians(self._cam_yaw)
        self._cam_cos = math.cos(rad)
        self._cam_sin = math.sin(rad)

        # Camera follow — track drone position with adjustable smoothness.
        # _fx/_fy/_fz are the camera's CURRENT focus point in world space.
        self._fx = self._fy = self._fz = 0.0

        # Zoom — base scale in pixels/metre. Scroll wheel adjusts this.
        # Range: 20 (far out = wide area view) … 200 (close up)
        self._zoom: float = SCALE   # start at default
        self._zoom_min: float = 20.0
        self._zoom_max: float = 220.0

        # Strobe state
        self._frame = 0
        # Star field
        rng = random.Random(42)
        self._stars = [(rng.random(), rng.random() * 0.48,
                        rng.randint(30, 110)) for _ in range(120)]
        self.setMinimumSize(560, 440)
        self.setStyleSheet("background:#06080C;")
        # Enable mouse wheel events
        self.setFocusPolicy(Qt.FocusPolicy.WheelFocus)

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def update_state(self, state: DroneState) -> None:
        self._state = state
        self._frame += 1
        if state.is_airborne or state.y > 0.05:
            self._trail.append((state.x, state.y, state.z))
        elif state.y < 0.01:
            self._trail.clear()

        # Camera follow — tight horizontal tracking so the drone always
        # stays near screen centre as it moves.
        # Horizontal (X/Z): alpha=0.18 — catches up within ~6 frames
        # Vertical (Y/altitude): alpha=0.08 — smoother altitude tracking
        # so sudden climbs don't jerk the view.
        ah = 0.18   # horizontal follow speed
        av = 0.08   # altitude follow speed
        self._fx += (state.x - self._fx) * ah
        self._fz += (state.z - self._fz) * ah
        self._fy += (state.y - self._fy) * av
        self.update()

    def set_camera_yaw(self, deg: float) -> None:
        self._cam_yaw = deg % 360.0
        rad = math.radians(self._cam_yaw)
        self._cam_cos = math.cos(rad)
        self._cam_sin = math.sin(rad)
        self.update()

    def orbit(self, delta: float) -> None:
        self.set_camera_yaw(self._cam_yaw + delta)

    def zoom_in(self) -> None:
        self._zoom = min(self._zoom_max, self._zoom * 1.20)
        self.update()

    def zoom_out(self) -> None:
        self._zoom = max(self._zoom_min, self._zoom / 1.20)
        self.update()

    def zoom_reset(self) -> None:
        self._zoom = SCALE
        self.update()

    def wheelEvent(self, event) -> None:   # noqa: N802
        """Scroll wheel: zoom in/out. Hold Ctrl for fine steps."""
        delta = event.angleDelta().y()
        factor = 1.12 if event.modifiers() & Qt.KeyboardModifier.ControlModifier else 1.25
        if delta > 0:
            self._zoom = min(self._zoom_max, self._zoom * factor)
        elif delta < 0:
            self._zoom = max(self._zoom_min, self._zoom / factor)
        self.update()
        event.accept()

    # ------------------------------------------------------------------
    # Paint entry point
    # ------------------------------------------------------------------

    def paintEvent(self, _ev) -> None:   # noqa: N802
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        w, h = self.width(), self.height()
        cc, sc = self._cam_cos, self._cam_sin
        z = self._zoom   # current zoom level

        # Camera origin — drone locked to centre of screen.
        # cx: horizontal follow of world X.
        # cy: vertical follow of world Y (altitude lifts view up) AND
        #     world Z (depth with the flatter projection ratio 0.09).
        # Using _zoom so both follow and grid scale together.
        cx = w * 0.5 - self._fx * z * 0.55
        cy = (h * 0.5 + 80
              - self._fy * z          # altitude: rise with drone
              - self._fz * z * 0.09)  # depth: small shift (matches DEPTH_Y)

        self._draw_environment(p, w, h, cx, cy, cc, sc, z)
        self._draw_trail(p, cx, cy, cc, sc, z)
        self._draw_shadow(p, cx, cy, cc, sc, z)
        self._draw_drone(p, cx, cy, cc, sc, z)
        self._draw_velocity_vector(p, cx, cy, cc, sc, z)
        self._draw_hud(p, w, h)
        p.end()

    # ==================================================================
    # ENVIRONMENT
    # ==================================================================

    def _draw_environment(self, p, w, h, cx, cy, cc, sc) -> None:
        self._draw_sky(p, w, h)
        self._draw_terrain(p, w, h, cx, cy, cc, sc)
        self._draw_range_rings(p, cx, cy, cc, sc)
        self._draw_grid(p, cx, cy, cc, sc)
        self._draw_origin_marker(p, cx, cy, cc, sc)
        self._draw_compass_ground(p, cx, cy, cc, sc)

    # ------------------------------------------------------------------
    # Sky with star field and horizon glow
    # ------------------------------------------------------------------

    def _draw_sky(self, p, w, h) -> None:
        g = QLinearGradient(0, 0, 0, h * 0.72)
        g.setColorAt(0.0, _C_BG)
        g.setColorAt(0.5, _C_SKY_MID)
        g.setColorAt(1.0, _C_SKY_HRZ)
        p.fillRect(0, 0, w, int(h * 0.72), QBrush(g))

        # Horizon line glow
        hy = int(h * 0.65)
        hg = QLinearGradient(0, hy - 8, 0, hy + 8)
        hg.setColorAt(0.0, QColor(0, 0, 0, 0))
        hg.setColorAt(0.5, QColor(40, 90, 40, 60))
        hg.setColorAt(1.0, QColor(0, 0, 0, 0))
        p.fillRect(0, hy - 8, w, 16, QBrush(hg))

        # Stars
        for fx, fy, alpha in self._stars:
            p.setPen(QPen(QColor(200, 230, 200, alpha), 1))
            p.drawPoint(int(fx * w), int(fy * h))

        # Fill terrain base below horizon
        p.fillRect(0, int(h * 0.65), w, h, QBrush(_C_TERRAIN))

    # ------------------------------------------------------------------
    # Terrain surface with subtle texture lines
    # ------------------------------------------------------------------

    def _draw_terrain(self, p, w, h, cx, cy, cc, sc) -> None:
        s = self._state
        # Subtle diagonal texture lines on terrain
        tex_pen = QPen(QColor(25, 38, 20, 40), 1)
        p.setPen(tex_pen)
        ox = round(s.x / 10) * 10
        oz = round(s.z / 10) * 10
        for i in range(-8, 9):
            wx = ox + i * 10
            # Diagonal texture at 45°
            p.drawLine(
                _qpt(wx, 0, oz - 80, cx, cy, cc, sc),
                _qpt(wx + 40, 0, oz + 40, cx, cy, cc, sc))

    # ------------------------------------------------------------------
    # Range rings — tactical distance markers
    # ------------------------------------------------------------------

    def _draw_range_rings(self, p, cx, cy, cc, sc) -> None:
        s = self._state
        for ring_r, label in ((25, "25"), (50, "50"), (100, "100")):
            alpha = max(20, 80 - int(s.altitude * 2))
            pen = QPen(QColor(40, 100, 40, alpha), 1, Qt.PenStyle.DotLine)
            p.setPen(pen)
            pts = []
            for deg in range(0, 361, 6):
                rad = math.radians(deg)
                wx = math.sin(rad) * ring_r
                wz = -math.cos(rad) * ring_r
                pts.append(_qpt(wx, 0, wz, cx, cy, cc, sc))
            for i in range(len(pts) - 1):
                p.drawLine(pts[i], pts[i+1])
            # Range label at East position
            lx, ly = _proj(ring_r, 0, 0, cx, cy, cc, sc)
            p.setFont(QFont("Consolas", 7))
            p.setPen(QPen(QColor(40, 120, 40, alpha + 40)))
            p.drawText(int(lx)+2, int(ly)-2, 28, 12,
                       Qt.AlignmentFlag.AlignLeft, f"{label}m")

    # ------------------------------------------------------------------
    # Tactical grid — 5 m spacing, 50 m sector labels
    # ------------------------------------------------------------------

    def _draw_grid(self, p, cx, cy, cc, sc) -> None:
        s   = self._state
        GAP = 5.0    # 5 m grid squares
        N   = 20     # 20 lines each side = 100 m radius
        ox  = round(s.x / GAP) * GAP
        oz  = round(s.z / GAP) * GAP

        for i in range(-N, N + 1):
            # Major lines every 5th (25 m)
            is_major = (i % 5 == 0)
            is_axis  = (i == 0)
            if is_axis:
                pen = QPen(QColor(50, 130, 50, 160), 1)
            elif is_major:
                pen = QPen(QColor(35, 80, 35, 100), 1)
            else:
                pen = QPen(QColor(22, 45, 22, 55), 1)
            p.setPen(pen)

            wz = oz + i * GAP
            p.drawLine(_qpt(ox-N*GAP, 0, wz, cx, cy, cc, sc),
                       _qpt(ox+N*GAP, 0, wz, cx, cy, cc, sc))
            wx = ox + i * GAP
            p.drawLine(_qpt(wx, 0, oz-N*GAP, cx, cy, cc, sc),
                       _qpt(wx, 0, oz+N*GAP, cx, cy, cc, sc))

        # Sector labels every 25 m
        p.setFont(QFont("Consolas", 7))
        for d in (-75, -50, -25, 0, 25, 50, 75):
            wx = round((s.x + d) / 25) * 25
            sx, sy = _proj(wx, 0, oz, cx, cy, cc, sc)
            p.setPen(QPen(QColor(50, 120, 50, 100)))
            p.drawText(int(sx)-14, int(sy)+2, 28, 11,
                       Qt.AlignmentFlag.AlignCenter, f"{int(wx)}")

    # ------------------------------------------------------------------
    # Origin marker — runway-style cross
    # ------------------------------------------------------------------

    def _draw_origin_marker(self, p, cx, cy, cc, sc) -> None:
        L = 3.0  # arm length metres
        pen = QPen(_C_AMBER, 2)
        p.setPen(pen)
        p.drawLine(_qpt(-L, 0,  0, cx, cy, cc, sc),
                   _qpt( L, 0,  0, cx, cy, cc, sc))
        p.drawLine(_qpt( 0, 0, -L, cx, cy, cc, sc),
                   _qpt( 0, 0,  L, cx, cy, cc, sc))
        # Origin dot
        ox, oy = _proj(0, 0, 0, cx, cy, cc, sc)
        p.setBrush(QBrush(_C_AMBER))
        p.setPen(_PN)
        p.drawEllipse(QPointF(ox, oy), 4, 3)

    # ------------------------------------------------------------------
    # Ground compass with large tactical labels
    # ------------------------------------------------------------------

    def _draw_compass_ground(self, p, cx, cy, cc, sc) -> None:
        s = self._state
        R = 12.0  # metres from drone centre

        dirs = [
            ("N",  0, -R, _C_GREEN),
            ("S",  0,  R, _C_RED),
            ("E",  R,  0, _C_AMBER),
            ("W", -R,  0, _C_BLUE),
        ]
        p.setFont(QFont("Consolas", 10, QFont.Weight.Bold))
        for lbl, dx, dz, col in dirs:
            sx, sy = _proj(s.x+dx, 0, s.z+dz, cx, cy, cc, sc)
            # Filled badge
            p.setBrush(QBrush(QColor(col.red(), col.green(), col.blue(), 170)))
            p.setPen(_PN)
            p.drawEllipse(QPointF(sx, sy), 16, 10)
            p.setPen(QPen(QColor(0, 0, 0, 200)))
            p.drawText(int(sx)-16, int(sy)-10, 32, 20,
                       Qt.AlignmentFlag.AlignCenter, lbl)

        # Heading ray on ground
        yr = math.radians(s.yaw)
        nx = s.x + math.sin(yr) * 6.0
        nz = s.z - math.cos(yr) * 6.0
        p.setPen(QPen(QColor(255, 176, 0, 200), 2))
        p.drawLine(_qpt(s.x, 0, s.z, cx, cy, cc, sc),
                   _qpt(nx,  0, nz,  cx, cy, cc, sc))

    # ==================================================================
    # TRAIL
    # ==================================================================

    def _draw_trail(self, p, cx, cy, cc, sc) -> None:
        trail = list(self._trail)
        n = len(trail)
        if n < 2:
            return
        for i in range(1, n):
            frac  = i / n
            alpha = int(180 * frac)
            # Colour shifts amber→green as the trail ages
            r = int(255 * (1 - frac) * 0.7)
            g = int(180 * frac + 60)
            p.setPen(QPen(QColor(r, g, 40, alpha), 2))
            ax, ay = _proj(*trail[i-1], cx, cy, cc, sc)
            bx, by = _proj(*trail[i],   cx, cy, cc, sc)
            p.drawLine(QPointF(ax, ay), QPointF(bx, by))

    # ==================================================================
    # SHADOW
    # ==================================================================

    def _draw_shadow(self, p, cx, cy, cc, sc) -> None:
        s = self._state
        if s.y < 0.1:
            return
        alpha = max(0, int(120 - s.y * 5))
        if alpha <= 0:
            return
        sx, sy = _proj(s.x, 0, s.z, cx, cy, cc, sc)
        rx = max(8, int(70 - s.y * 2))
        ry = int(rx * 0.30)
        # Shadow gradient
        sg = QRadialGradient(QPointF(sx, sy), rx)
        sg.setColorAt(0.0, QColor(0, 0, 0, alpha))
        sg.setColorAt(1.0, QColor(0, 0, 0, 0))
        p.setBrush(QBrush(sg))
        p.setPen(_PN)
        p.drawEllipse(QPointF(sx, sy), rx, ry)

    # ==================================================================
    # DRONE — MQ-style quadrotor UAV
    # ==================================================================

    def _draw_drone(self, p, cx, cy, cc, sc) -> None:
        s   = self._state
        ARM = 0.75   # longer booms for bigger presence

        yr  = math.radians(s.yaw)
        pr  = math.radians(s.pitch)
        rr  = math.radians(s.roll)
        cyr, syr = math.cos(yr), math.sin(yr)
        cpr, spr = math.cos(pr), math.sin(pr)
        crr, srr = math.cos(rr), math.sin(rr)

        def b2w(bx, by, bz):
            cx2 = bx*crr - by*srr; cy2 = bx*srr + by*crr
            dx  = cx2; dy2 = cy2*cpr - bz*spr; dz = cy2*spr + bz*cpr
            ex  = dx*cyr - dz*syr; ez = dx*syr + dz*cyr
            return ex + s.x, dy2 + s.y, ez + s.z

        # Motor positions: front = -Z (north), back = +Z (south)
        motors = [
            b2w( ARM, 0, -ARM),  # FR
            b2w(-ARM, 0, -ARM),  # FL
            b2w(-ARM, 0,  ARM),  # BL
            b2w( ARM, 0,  ARM),  # BR
        ]

        def wp(wx, wy, wz): return _qpt(wx, wy, wz, cx, cy, cc, sc)
        cpt = wp(s.x, s.y, s.z)

        # ── Landing gear (shown only near ground) ────────────────────
        if s.y < 1.5:
            gear_alpha = int(255 * max(0, 1.0 - s.y / 1.5))
            gear_pen = QPen(QColor(50, 70, 50, gear_alpha), 2)
            p.setPen(gear_pen)
            for mx, my, mz in motors:
                ground_pt = wp(mx, 0, mz)
                motor_pt  = wp(mx, my, mz)
                p.drawLine(motor_pt, ground_pt)
                # Foot pad
                p.setBrush(QBrush(QColor(40, 60, 40, gear_alpha)))
                p.setPen(_PN)
                p.drawEllipse(ground_pt, 5, 3)

        # ── Boom spars — thick carbon-fibre tubes ─────────────────────
        for mx, my, mz in motors:
            mp = wp(mx, my, mz)
            # Thick dark core
            p.setPen(QPen(_C_BOOM, 6, Qt.PenStyle.SolidLine,
                          Qt.PenCapStyle.RoundCap))
            p.drawLine(cpt, mp)
            # Thin highlight edge
            p.setPen(QPen(_C_BOOM_HI, 1))
            p.drawLine(cpt, mp)

        # ── Central body — hexagonal fuselage ─────────────────────────
        body_pts = []
        for i in range(6):
            a = math.radians(i * 60)
            bw = b2w(math.cos(a) * 0.18, 0, math.sin(a) * 0.18)
            body_pts.append(wp(*bw))
        poly = QPolygonF(body_pts)
        p.setBrush(QBrush(_C_BODY_D))
        p.setPen(QPen(_C_BODY_L, 1))
        p.drawPolygon(poly)

        # Body panel lines (cross detail)
        p.setPen(QPen(QColor(50, 80, 50), 1))
        p.drawLine(wp(*b2w(-0.16, 0, 0)), wp(*b2w(0.16, 0, 0)))
        p.drawLine(wp(*b2w(0, 0, -0.16)), wp(*b2w(0, 0,  0.16)))

        # ── Camera gimbal dome (below body front) ─────────────────────
        cam_w = b2w(0, -0.05, -0.12)   # slightly below, slightly forward
        cam_pt = wp(*cam_w)
        # Gimbal mount ring
        p.setBrush(QBrush(QColor(30, 40, 30)))
        p.setPen(QPen(QColor(60, 90, 60), 1))
        p.drawEllipse(cam_pt, 9, 6)
        # Dome glass
        p.setBrush(QBrush(_C_DOME))
        p.setPen(QPen(QColor(80, 120, 180, 150), 1))
        p.drawEllipse(cam_pt, 7, 5)
        # Lens reflection
        p.setBrush(QBrush(QColor(200, 220, 255, 80)))
        p.setPen(_PN)
        p.drawEllipse(QPointF(cam_pt.x()-2, cam_pt.y()-1), 2, 2)

        # ── Top electronics dome ──────────────────────────────────────
        top_w = b2w(0, 0.10, 0)
        p.setBrush(QBrush(QColor(28, 38, 28)))
        p.setPen(QPen(QColor(60, 90, 60), 1))
        p.drawEllipse(wp(*top_w), 12, 7)
        # Antenna stub
        ant_w = b2w(0, 0.22, -0.05)
        p.setPen(QPen(QColor(80, 100, 80), 2))
        p.drawLine(wp(*top_w), wp(*ant_w))

        # ── Motor pods ────────────────────────────────────────────────
        for i, (mx, my, mz) in enumerate(motors):
            mp = wp(mx, my, mz)
            p.setBrush(QBrush(QColor(28, 40, 28)))
            p.setPen(QPen(QColor(60, 100, 60), 1))
            p.drawEllipse(mp, 14, 9)
            # Motor vent lines
            p.setPen(QPen(QColor(50, 80, 50), 1))
            for vang in (0, 60, 120):
                va_r = math.radians(vang)
                vx = math.cos(va_r) * 0.12
                vz = math.sin(va_r) * 0.12
                p.drawLine(
                    wp(*b2w(mx-s.x + vx, my-s.y, mz-s.z + vz)),
                    wp(*b2w(mx-s.x - vx, my-s.y, mz-s.z - vz)))

        # ── Rotors ────────────────────────────────────────────────────
        if s.rotor_speed > 0.01:
            self._draw_rotors(p, cx, cy, cc, sc, motors, s)

        # ── Navigation lights (aviation standard) ────────────────────
        if s.mode != FlightMode.DISARMED:
            la = min(255, int(s.rotor_speed * 280))
            strobe_on = (self._frame % 30) < 8   # strobe blink

            for i, (mx, my, mz) in enumerate(motors):
                mp = wp(mx, my, mz)
                if i == 0:   # FR = starboard = RED
                    col = QColor(_C_NAV_RED.red(), _C_NAV_RED.green(),
                                 _C_NAV_RED.blue(), la)
                elif i == 1:  # FL = port = GREEN
                    col = QColor(_C_NAV_GRN.red(), _C_NAV_GRN.green(),
                                 _C_NAV_GRN.blue(), la)
                else:         # Rear = WHITE STROBE
                    alpha = la if strobe_on else la // 4
                    col = QColor(_C_STROBE.red(), _C_STROBE.green(),
                                 _C_STROBE.blue(), alpha)

                # Halo glow
                glow = QRadialGradient(mp, 14)
                glow.setColorAt(0.0, QColor(col.red(), col.green(), col.blue(), 80))
                glow.setColorAt(1.0, QColor(col.red(), col.green(), col.blue(), 0))
                p.setBrush(QBrush(glow)); p.setPen(_PN)
                p.drawEllipse(mp, 14, 10)
                # Core dot
                p.setBrush(QBrush(col)); p.setPen(_PN)
                p.drawEllipse(mp, 4, 3)

        # ── Altitude stem ─────────────────────────────────────────────
        if s.y > 0.5:
            p.setPen(QPen(QColor(40, 100, 40, 60), 1,
                          Qt.PenStyle.DashLine))
            p.drawLine(wp(s.x, 0, s.z), cpt)
            # Altitude tick marks every 5 m
            for alt_m in range(5, int(s.y) + 1, 5):
                tx, ty = _proj(s.x, alt_m, s.z, cx, cy, cc, sc)
                p.setPen(QPen(QColor(40, 130, 40, 100), 1))
                p.drawLine(QPointF(tx - 6, ty), QPointF(tx + 6, ty))

        # ── Rotor wash (turbulence effect below when flying) ──────────
        if s.rotor_speed > 0.2 and s.y > 0.5:
            for scale, alpha in ((90, 30), (60, 50), (35, 70)):
                ga = int(s.rotor_speed * alpha)
                glow = QRadialGradient(cpt, scale)
                glow.setColorAt(0.0, QColor(40, 120, 40, ga))
                glow.setColorAt(0.7, QColor(20, 80, 20, ga // 3))
                glow.setColorAt(1.0, QColor(0, 0, 0, 0))
                p.setBrush(QBrush(glow)); p.setPen(_PN)
                p.drawEllipse(cpt, scale, int(scale * 0.35))

        # ── FWD nose marker ───────────────────────────────────────────
        yr_use = math.radians(s.yaw)
        nd = ARM * 1.2
        nwx = s.x + math.sin(yr_use) * nd
        nwy = s.y
        nwz = s.z - math.cos(yr_use) * nd
        np_ = wp(nwx, nwy, nwz)
        p.setBrush(QBrush(_C_AMBER))
        p.setPen(QPen(QColor(0, 0, 0, 180), 1))
        p.drawEllipse(np_, 6, 4)
        nsx, nsy = _proj(nwx, nwy, nwz, cx, cy, cc, sc)
        p.setPen(QPen(_C_AMBER))
        p.setFont(QFont("Consolas", 8, QFont.Weight.Bold))
        p.drawText(int(nsx)-14, int(nsy)-16, 28, 12,
                   Qt.AlignmentFlag.AlignCenter, "FWD")

    def _draw_rotors(self, p, cx, cy, cc, sc, motors, s: DroneState) -> None:
        R     = 0.55   # large tactical rotor radius
        speed = s.rotor_speed

        for i, (mx, my, mz) in enumerate(motors):
            mp  = _qpt(mx, my, mz, cx, cy, cc, sc)
            ang = s.rotor_angles[i]

            # Rotor guard ring (static)
            guard_pts = [_qpt(mx + math.cos(math.radians(k))*R, my,
                               mz + math.sin(math.radians(k))*R, cx, cy, cc, sc)
                         for k in range(0, 361, 10)]
            p.setBrush(Qt.BrushStyle.NoBrush)
            p.setPen(QPen(_C_GUARD, 1))
            p.drawPolygon(QPolygonF(guard_pts))

            # Disc blur — more opaque at higher speed
            disc_alpha = int(speed * 130)
            disc_pts = [_qpt(mx+math.cos(math.radians(ang+k*30))*R, my,
                              mz+math.sin(math.radians(ang+k*30))*R, cx, cy, cc, sc)
                        for k in range(12)]
            disc_col = QColor(_C_ROTOR.red(), _C_ROTOR.green(),
                              _C_ROTOR.blue(), disc_alpha)
            p.setBrush(QBrush(disc_col))
            p.setPen(QPen(QColor(30, 60, 30, disc_alpha // 2), 1))
            p.drawPolygon(QPolygonF(disc_pts))

            # 4 blade lines
            blade_alpha = min(240, int(speed * 240))
            bp = QPen(QColor(_C_BLADE.red(), _C_BLADE.green(),
                             _C_BLADE.blue(), blade_alpha), 2,
                      Qt.PenStyle.SolidLine, Qt.PenCapStyle.RoundCap)
            p.setPen(bp)
            for b in range(4):
                ba_r = math.radians(ang + b * 90)
                p.drawLine(mp, _qpt(mx + math.cos(ba_r)*R, my,
                                    mz + math.sin(ba_r)*R, cx, cy, cc, sc))

    # ==================================================================
    # VELOCITY VECTOR
    # ==================================================================

    def _draw_velocity_vector(self, p, cx, cy, cc, sc) -> None:
        s   = self._state
        spd = s.speed_h
        if spd < 0.4:
            return
        vlen = math.sqrt(s.vx**2 + s.vz**2)
        if vlen < 0.01:
            return
        scale = min(5.0, spd / 1.5)
        uvx, uvz = s.vx / vlen, s.vz / vlen
        tx  = s.x + uvx * scale
        tz  = s.z + uvz * scale
        bp  = _qpt(s.x, s.y, s.z, cx, cy, cc, sc)
        tp  = _qpt(tx,  s.y, tz,  cx, cy, cc, sc)

        # Glow behind the arrow
        for width, alpha in ((7, 40), (4, 80), (2, 200)):
            p.setPen(QPen(QColor(_C_GREEN.red(), _C_GREEN.green(),
                                 _C_GREEN.blue(), alpha), width))
            p.drawLine(bp, tp)

        p.setBrush(QBrush(_C_GREEN))
        _arrow_head(p, tp, bp, head_len=14, head_w=8)

        # Speed label with tactical box
        p.setFont(QFont("Consolas", 9, QFont.Weight.Bold))
        spd_txt = f"{spd:.1f}"
        p.setBrush(QBrush(QColor(0, 20, 0, 180)))
        p.setPen(_PN)
        p.drawRect(int(tp.x())+4, int(tp.y())-10, 40, 14)
        p.setPen(QPen(_C_GREEN))
        p.drawText(int(tp.x())+6, int(tp.y())-10, 38, 14,
                   Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter,
                   f"{spd_txt}m/s")

    # ==================================================================
    # HUD — Tactical Ground-Control Station display
    # ==================================================================

    def _draw_hud(self, p: QPainter, w: int, h: int) -> None:
        s = self._state
        self._hud_scanlines(p, w, h)          # subtle CRT scanline effect
        self._hud_mode_badge(p, w, s)          # top-centre mode + command
        self._hud_telemetry(p, s)              # top-left data block
        self._hud_compass_rose(p, w, s)        # top-right compass
        self._hud_adi(p, h, s)                 # bottom-left attitude
        self._hud_altitude_ladder(p, w, h, s)  # bottom-right ladder
        self._hud_minimap(p, w, h)             # bottom-right-corner radar
        self._hud_threat_strip(p, w, h, s)     # right edge status strip
        self._hud_crosshair(p, w, h)           # centre reticle

    # ------------------------------------------------------------------
    # Scanlines — subtle CRT / NVG effect
    # ------------------------------------------------------------------

    def _hud_scanlines(self, p: QPainter, w: int, h: int) -> None:
        p.setPen(QPen(QColor(0, 0, 0, 18), 1))
        for y in range(0, h, 3):
            p.drawLine(0, y, w, y)

    # ------------------------------------------------------------------
    # Centre reticle — FPV-style crosshair
    # ------------------------------------------------------------------

    def _hud_crosshair(self, p: QPainter, w: int, h: int) -> None:
        cx, cy = w // 2, h // 2
        col = QColor(40, 220, 80, 90)
        p.setPen(QPen(col, 1))
        gap = 12
        arm = 28
        # Horizontal
        p.drawLine(cx - arm - gap, cy, cx - gap, cy)
        p.drawLine(cx + gap, cy, cx + arm + gap, cy)
        # Vertical
        p.drawLine(cx, cy - arm - gap, cx, cy - gap)
        p.drawLine(cx, cy + gap, cx, cy + arm + gap)
        # Corner ticks
        for dx, dy in ((-1,-1),(1,-1),(1,1),(-1,1)):
            p.drawLine(cx+dx*gap, cy+dy*gap,
                       cx+dx*(gap+10), cy+dy*gap)
            p.drawLine(cx+dx*gap, cy+dy*gap,
                       cx+dx*gap, cy+dy*(gap+10))

    # ------------------------------------------------------------------
    # Mode badge + flight command — top centre
    # ------------------------------------------------------------------

    def _hud_mode_badge(self, p: QPainter, w: int, s: DroneState) -> None:
        mode = s.mode.value
        rgb  = _MODE_RGB.get(mode, (60, 70, 60))
        r, g, b = rgb

        # ── Mode badge ────────────────────────────────────────────────
        f = QFont("Consolas", 12, QFont.Weight.Bold)
        p.setFont(f)
        mode_txt = f"[ {mode} ]"
        fm = p.fontMetrics()
        mw = fm.horizontalAdvance(mode_txt) + 16
        mh = fm.height() + 8
        mx = (w - mw) // 2
        my = 8

        # Dark background box with coloured border
        p.setBrush(QBrush(QColor(0, 10, 0, 200)))
        p.setPen(QPen(QColor(r, g, b, 200), 1))
        p.drawRect(mx, my, mw, mh)
        # Corner brackets
        blen = 8
        p.setPen(QPen(QColor(r, g, b), 2))
        for bx, by, sx_, sy_ in (
            (mx,    my,    1, 1), (mx+mw, my,    -1, 1),
            (mx,    my+mh, 1,-1), (mx+mw, my+mh, -1,-1),
        ):
            p.drawLine(bx, by, bx + sx_*blen, by)
            p.drawLine(bx, by, bx, by + sy_*blen)
        p.setPen(QPen(QColor(r, g, b)))
        p.drawText(mx, my, mw, mh, Qt.AlignmentFlag.AlignCenter, mode_txt)

        # ── Flight command banner ─────────────────────────────────────
        cmd = s.flight_command
        if cmd and "Disarmed" not in cmd:
            f2 = QFont("Segoe UI", 14, QFont.Weight.Bold)
            p.setFont(f2)
            fm2 = p.fontMetrics()
            cw  = fm2.horizontalAdvance(cmd) + 24
            ch  = fm2.height() + 10
            cbx = (w - cw) // 2
            cby = my + mh + 4

            # Pick banner colour from direction
            br, bg_, bb = r, g, b
            cmd_low = cmd.lower()
            if "forward" in cmd_low:   br, bg_, bb = 40, 220,  80
            elif "back"  in cmd_low:   br, bg_, bb = 220, 60,  40
            elif "right" in cmd_low:   br, bg_, bb = 255, 176,   0
            elif "left"  in cmd_low:   br, bg_, bb = 60, 160, 240
            elif "climb" in cmd_low:   br, bg_, bb = 40, 220,  80
            elif "descend" in cmd_low: br, bg_, bb = 220, 80,  40

            p.setBrush(QBrush(QColor(br, bg_, bb, 180)))
            p.setPen(_PN)
            p.drawRoundedRect(cbx, cby, cw, ch, 5, 5)
            p.setPen(QPen(QColor(0, 10, 0)))
            p.drawText(cbx, cby, cw, ch, Qt.AlignmentFlag.AlignCenter, cmd)

    # ------------------------------------------------------------------
    # Telemetry block — top left, GCS-style
    # ------------------------------------------------------------------

    def _hud_telemetry(self, p: QPainter, s: DroneState) -> None:
        # Background panel
        p.setBrush(QBrush(QColor(0, 10, 0, 180)))
        p.setPen(QPen(_C_GREEN, 1))
        p.drawRect(6, 6, 160, 158)
        # Panel title
        p.setFont(QFont("Consolas", 7, QFont.Weight.Bold))
        p.setPen(QPen(_C_AMBER))
        p.drawText(8, 6, 158, 14, Qt.AlignmentFlag.AlignCenter, "◆ UAV TELEMETRY ◆")
        p.setPen(QPen(QColor(_C_AMBER.red(), _C_AMBER.green(), _C_AMBER.blue(), 100)))
        p.drawLine(8, 20, 164, 20)

        rows = [
            ("ALT",      f"{s.altitude:7.2f} m",     _C_GREEN),
            ("H-SPD",    f"{s.speed_h:6.2f} m/s",   _C_GREEN),
            ("V-SPD",    f"{s.speed_v:+6.2f} m/s",  _C_AMBER if abs(s.speed_v)>0.3 else _C_GREEN),
            ("HDG",      f"{s.heading:6.1f}°",       _C_AMBER),
            ("PITCH",    f"{s.pitch:+6.1f}°",        _C_WHITE),
            ("ROLL",     f"{s.roll:+6.1f}°",         _C_WHITE),
            ("X POS",    f"{s.x:7.1f} m",            _C_BLUE),
            ("Z POS",    f"{s.z:7.1f} m",            _C_BLUE),
            ("DIST",     f"{s.total_distance:6.0f} m", _C_DIM),
            ("FLT TIME", f"{s.flight_time:6.1f} s",  _C_DIM),
        ]
        p.setFont(QFont("Consolas", 9))
        y = 24
        for label, value, col in rows:
            p.setPen(QPen(_C_DIM))
            p.drawText(8, y, 58, 14, Qt.AlignmentFlag.AlignRight, label)
            p.setPen(QPen(col))
            p.drawText(70, y, 94, 14, Qt.AlignmentFlag.AlignLeft, value)
            y += 14

    # ------------------------------------------------------------------
    # Compass rose — top right, bearing tape style
    # ------------------------------------------------------------------

    def _hud_compass_rose(self, p: QPainter, w: int, s: DroneState) -> None:
        cx, cy, r = w - 58, 60, 46

        # Dark panel
        p.setBrush(QBrush(QColor(0, 10, 0, 200)))
        p.setPen(QPen(_C_GREEN, 1))
        p.drawEllipse(QPointF(cx, cy), r, r)

        # Tick marks every 10°
        p.setFont(QFont("Consolas", 7, QFont.Weight.Bold))
        for deg in range(0, 360, 10):
            rad_h = math.radians(deg - s.heading)
            sin_h, cos_h = math.sin(rad_h), math.cos(rad_h)
            is_card = (deg % 90 == 0)
            is_major = (deg % 30 == 0)
            tick_len = 10 if is_card else (6 if is_major else 3)
            inner = r - tick_len
            ox = cx + inner * sin_h
            oy = cy - inner * cos_h
            tx = cx + (r - 2) * sin_h
            ty = cy - (r - 2) * cos_h
            col = _C_AMBER if is_card else (_C_WHITE if is_major else _C_DIM)
            p.setPen(QPen(col, 2 if is_card else 1))
            p.drawLine(QPointF(ox, oy), QPointF(tx, ty))
            if is_major:
                lbl = {0:"N",90:"E",180:"S",270:"W"}.get(deg, str(deg))
                lx = cx + (inner - 10) * sin_h
                ly = cy - (inner - 10) * cos_h
                p.setPen(QPen(_C_GREEN if deg == 0 else col))
                p.drawText(int(lx)-8, int(ly)-7, 16, 14,
                           Qt.AlignmentFlag.AlignCenter, lbl)

        # Fixed heading needle (always points up = current heading)
        p.setPen(QPen(_C_AMBER, 2))
        p.drawLine(QPointF(cx, cy - r + 4), QPointF(cx, cy - 10))
        # Heading triangle
        tri = QPolygonF([
            QPointF(cx, cy - r + 2),
            QPointF(cx - 5, cy - r + 12),
            QPointF(cx + 5, cy - r + 12),
        ])
        p.setBrush(QBrush(_C_AMBER)); p.setPen(_PN)
        p.drawPolygon(tri)

        # Centre heading readout box
        p.setBrush(QBrush(QColor(0, 20, 0, 210)))
        p.setPen(QPen(_C_AMBER, 1))
        p.drawRect(int(cx) - 20, int(cy) - 9, 40, 18)
        p.setFont(QFont("Consolas", 10, QFont.Weight.Bold))
        p.setPen(QPen(_C_AMBER))
        p.drawText(int(cx)-20, int(cy)-9, 40, 18,
                   Qt.AlignmentFlag.AlignCenter, f"{s.heading:05.1f}")

        # Centre dot
        p.setBrush(QBrush(_C_GREEN)); p.setPen(_PN)
        p.drawEllipse(QPointF(cx, cy), 3, 3)

    # ------------------------------------------------------------------
    # ADI (Attitude Direction Indicator) — bottom left
    # ------------------------------------------------------------------

    def _hud_adi(self, p: QPainter, h: int, s: DroneState) -> None:
        cx, cy, r = 58, h - 70, 46

        p.setClipRect(int(cx-r), int(cy-r), r*2, r*2)
        # Sky half
        p.setBrush(QBrush(QColor(0, 30, 80, 200))); p.setPen(_PN)
        p.drawEllipse(QPointF(cx, cy), r, r)
        # Ground half with pitch offset
        pitch_off = s.pitch * (r / 55.0)
        rr_rad    = math.radians(-s.roll)
        cos_r     = math.cos(rr_rad)
        sin_r     = math.sin(rr_rad)
        path = QPainterPath()
        hx1 = cx - r*cos_r - pitch_off*sin_r; hy1 = cy - r*sin_r + pitch_off*cos_r
        hx2 = cx + r*cos_r + pitch_off*sin_r; hy2 = cy + r*sin_r - pitch_off*cos_r
        path.moveTo(hx1, hy1); path.lineTo(hx2, hy2)
        path.lineTo(cx+r, cy+r+2); path.lineTo(cx-r, cy+r+2)
        path.closeSubpath()
        p.setBrush(QBrush(QColor(60, 40, 10, 200)))
        p.drawPath(path)
        p.setClipping(False)

        # Pitch ladder lines
        p.setFont(QFont("Consolas", 7))
        for pitch_mark in (-20, -10, 0, 10, 20):
            pm_off  = (pitch_mark - s.pitch) * (r / 55.0)
            pm_rotx = -pm_off * sin_r
            pm_roty =  pm_off * cos_r
            px = cx + pm_rotx
            py = cy + pm_roty
            line_w = 20 if pitch_mark == 0 else 12
            col = _C_WHITE if pitch_mark == 0 else _C_DIM
            p.setPen(QPen(col, 1))
            p.drawLine(QPointF(px-line_w, py), QPointF(px+line_w, py))
            if pitch_mark != 0:
                p.setPen(QPen(_C_DIM))
                p.drawText(int(px)+line_w+2, int(py)-5, 20, 10,
                           Qt.AlignmentFlag.AlignLeft, str(abs(pitch_mark)))

        # Rim + cross
        p.setBrush(Qt.BrushStyle.NoBrush)
        p.setPen(QPen(_C_GREEN, 2))
        p.drawEllipse(QPointF(cx, cy), r, r)
        # Fixed aircraft symbol
        p.setPen(QPen(_C_AMBER, 2))
        p.drawLine(int(cx)-14, int(cy), int(cx)-5, int(cy))
        p.drawLine(int(cx)+5,  int(cy), int(cx)+14, int(cy))
        p.drawLine(int(cx), int(cy)-4, int(cx), int(cy)+4)

        # ADI label
        p.setFont(QFont("Consolas", 7))
        p.setPen(QPen(_C_DIM))
        p.drawText(int(cx)-r, int(cy)+r+2, r*2, 11,
                   Qt.AlignmentFlag.AlignCenter, "ADI")

    # ------------------------------------------------------------------
    # Altitude & throttle ladder tapes — bottom right
    # ------------------------------------------------------------------

    def _hud_altitude_ladder(self, p: QPainter, w: int, h: int,
                              s: DroneState) -> None:
        # Altitude tape (left of the two)
        self._ladder_tape(p, w-100, h-170, 18, 160,
                          s.altitude, 30.0, _C_BLUE,
                          "ALT", "m", step=5)
        # Throttle tape (right)
        thr_pct = (s.throttle - 0.5) * 200   # −100 to +100 centred on hold
        self._ladder_tape(p, w-76, h-170, 18, 160,
                          thr_pct, 100.0, _C_GREEN,
                          "THR", "%", step=25,
                          centre_label="HOLD")

    def _ladder_tape(self, p, x, y, bw, bh, value, max_val,
                     col, label, unit, step=5, centre_label="") -> None:
        # Background
        p.setBrush(QBrush(QColor(0, 10, 0, 200)))
        p.setPen(QPen(_C_DIM, 1))
        p.drawRect(x, y, bw, bh)

        # Fill bar
        frac = max(0.0, min(1.0, (value + max_val) / (2 * max_val)
                   if centre_label else value / max_val))
        fh = int(bh * frac)
        if fh > 0:
            p.setBrush(QBrush(QColor(col.red(), col.green(), col.blue(), 120)))
            p.setPen(_PN)
            p.drawRect(x+1, y+bh-fh, bw-2, fh)

        # Tick marks
        p.setFont(QFont("Consolas", 7))
        for tick in range(0, int(max_val)+1, step):
            frac_t = (tick + max_val)/(2*max_val) if centre_label else tick/max_val
            ty_    = y + bh - int(bh * frac_t)
            is_maj = (tick % (step*2) == 0)
            p.setPen(QPen(col if is_maj else _C_DIM, 1))
            p.drawLine(x+bw-5, ty_, x+bw, ty_)
            if is_maj:
                p.drawText(x-24, ty_-5, 22, 10,
                           Qt.AlignmentFlag.AlignRight, str(tick))

        # Current value pointer
        ptr_y = y + bh - int(bh * frac)
        ptr = QPolygonF([
            QPointF(x, ptr_y),
            QPointF(x-8, ptr_y-6),
            QPointF(x-8, ptr_y+6),
        ])
        p.setBrush(QBrush(col)); p.setPen(_PN)
        p.drawPolygon(ptr)
        # Value box
        p.setBrush(QBrush(QColor(0, 20, 0, 220)))
        p.setPen(QPen(col, 1))
        p.drawRect(x-34, ptr_y-8, 32, 16)
        p.setFont(QFont("Consolas", 8, QFont.Weight.Bold))
        p.setPen(QPen(col))
        p.drawText(x-34, ptr_y-8, 32, 16,
                   Qt.AlignmentFlag.AlignCenter,
                   f"{value:.0f}")

        # Centre hold line
        if centre_label:
            mid_y = y + bh // 2
            p.setPen(QPen(_C_AMBER, 1, Qt.PenStyle.DashLine))
            p.drawLine(x, mid_y, x+bw, mid_y)

        # Label
        p.setFont(QFont("Consolas", 7, QFont.Weight.Bold))
        p.setPen(QPen(col))
        p.drawText(x, y-12, bw, 11, Qt.AlignmentFlag.AlignCenter, label)
        p.setFont(QFont("Consolas", 7))
        p.setPen(QPen(_C_DIM))
        p.drawText(x, y+bh+2, bw, 10, Qt.AlignmentFlag.AlignCenter, unit)

    # ------------------------------------------------------------------
    # Threat / status strip — right edge
    # ------------------------------------------------------------------

    def _hud_threat_strip(self, p: QPainter, w: int, h: int,
                           s: DroneState) -> None:
        x = w - 52
        p.setBrush(QBrush(QColor(0, 8, 0, 190)))
        p.setPen(QPen(_C_GREEN, 1))
        p.drawRect(x, 100, 46, 220)

        p.setFont(QFont("Consolas", 7, QFont.Weight.Bold))
        p.setPen(QPen(_C_AMBER))
        p.drawText(x, 100, 46, 12, Qt.AlignmentFlag.AlignCenter, "STATUS")
        p.setPen(QPen(QColor(_C_AMBER.red(), _C_AMBER.green(),
                              _C_AMBER.blue(), 80)))
        p.drawLine(x, 112, x+46, 112)

        items = []
        items.append(("MODE",  s.mode.value[:6], _MODE_RGB.get(s.mode.value,(60,70,60))))
        items.append(("SPD",   f"{s.speed_h:.1f}", (40, 180, 80)))
        items.append(("ALT",   f"{s.altitude:.1f}", (40, 160, 220)))
        items.append(("HDG",   f"{s.heading:.0f}°", (200, 160, 0)))
        items.append(("THR",   f"{(s.throttle-0.5)*200:+.0f}%",
                      (40,220,80) if abs(s.throttle-0.5)<0.1 else (220,140,0)))
        if s.is_airborne:
            items.append(("AIR",  "BORN", (40, 220, 80)))
        else:
            items.append(("GND",  "LOCK", (200, 80, 40)))

        p.setFont(QFont("Consolas", 7))
        iy = 116
        for key, val, rgb_ in items:
            r2, g2, b2 = rgb_
            p.setPen(QPen(_C_DIM))
            p.drawText(x+2, iy, 44, 11, Qt.AlignmentFlag.AlignLeft, key)
            p.setPen(QPen(QColor(r2, g2, b2)))
            p.drawText(x+2, iy+10, 44, 11, Qt.AlignmentFlag.AlignRight, val)
            p.setPen(QPen(QColor(30, 50, 30, 60)))
            p.drawLine(x+2, iy+21, x+44, iy+21)
            iy += 24

    # ------------------------------------------------------------------
    # Radar mini-map — bottom right corner
    # ------------------------------------------------------------------

    def _hud_minimap(self, p: QPainter, w: int, h: int) -> None:
        s     = self._state
        MR    = 68
        MSCL  = 5.5     # px per metre
        mx    = w - MR - 10
        my    = h - MR - 10

        # Background with gradient
        bg_g = QRadialGradient(QPointF(mx, my), MR)
        bg_g.setColorAt(0.0, QColor(0, 20, 0, 210))
        bg_g.setColorAt(1.0, QColor(0, 8, 0, 220))
        p.setBrush(QBrush(bg_g))
        p.setPen(QPen(_C_GREEN, 1))
        p.drawEllipse(QPointF(mx, my), MR, MR)

        # Radar sweep rings
        for rr in (MR * 0.33, MR * 0.66):
            p.setPen(QPen(QColor(30, 90, 30, 60), 1))
            p.setBrush(Qt.BrushStyle.NoBrush)
            p.drawEllipse(QPointF(mx, my), rr, rr)

        # Grid cross
        p.setPen(QPen(QColor(30, 80, 30, 50), 1))
        p.drawLine(QPointF(mx-MR+4, my), QPointF(mx+MR-4, my))
        p.drawLine(QPointF(mx, my-MR+4), QPointF(mx, my+MR-4))

        # Cardinal labels
        p.setFont(QFont("Consolas", 7, QFont.Weight.Bold))
        for lbl, dx, dy, col in (
            ("N", 0,-1, _C_GREEN), ("S", 0, 1, _C_RED),
            ("E", 1, 0, _C_AMBER), ("W",-1, 0, _C_BLUE),
        ):
            p.setPen(QPen(col))
            p.drawText(int(mx+dx*(MR-9))-6, int(my+dy*(MR-9))-6,
                       12, 12, Qt.AlignmentFlag.AlignCenter, lbl)

        # Trail
        p.setClipRect(int(mx-MR), int(my-MR), MR*2, MR*2)
        trail = list(self._trail)
        n = len(trail)
        for i, (tx, _, tz) in enumerate(trail):
            a  = int(150 * i / max(n, 1))
            rx = mx + (tx - s.x) * MSCL
            ry = my + (tz - s.z) * MSCL
            p.setBrush(QBrush(QColor(40, 200, 80, a)))
            p.setPen(_PN)
            p.drawEllipse(QPointF(rx, ry), 2, 2)

        # Drone dot with heading indicator
        p.setBrush(QBrush(_C_GREEN))
        p.setPen(QPen(QColor(0,0,0,180), 1))
        p.drawEllipse(QPointF(mx, my), 5, 5)
        yr  = math.radians(s.yaw)
        ahx = mx + math.sin(yr) * 16
        ahy = my - math.cos(yr) * 16
        p.setPen(QPen(_C_AMBER, 2))
        p.drawLine(QPointF(mx, my), QPointF(ahx, ahy))
        # Arrowhead
        p.setBrush(QBrush(_C_AMBER))
        _arrow_head(p, QPointF(ahx, ahy), QPointF(mx, my), 6, 4)

        p.setClipping(False)

        # Position readout below map
        p.setFont(QFont("Consolas", 7))
        p.setPen(QPen(_C_DIM))
        coord_txt = f"X{s.x:+6.1f} Z{s.z:+6.1f}"
        p.drawText(int(mx-MR), int(my+MR+2), MR*2, 11,
                   Qt.AlignmentFlag.AlignCenter, coord_txt)

        # Radar label
        p.setPen(QPen(QColor(30, 100, 30, 120)))
        p.setFont(QFont("Consolas", 7, QFont.Weight.Bold))
        p.drawText(int(mx-MR), int(my-MR-12), MR*2, 11,
                   Qt.AlignmentFlag.AlignCenter, "◆ RADAR ◆")
