"""3D QPainter drone renderer — clarity edition.

Changes from previous version
------------------------------
1. Compass rose drawn ON the ground plane with large N/S/E/W labels
2. Coloured grid axes: X-axis = red (East), Z-axis = blue (North/Forward)
3. Direction arrow drawn through drone centre showing current heading
4. Velocity vector arrow (green) showing actual movement direction
5. Flight command banner — large, centred, colour-coded by direction
6. Mini top-down map (bottom-right) showing position + heading at all times
7. Grid coordinate labels every 10 m so you know exact position
8. Drone nose marker — bright yellow dot on the front of the drone
9. Status panel: large readable direction text
"""
from __future__ import annotations

import math
import random
from collections import deque

from PySide6.QtCore import QPointF, Qt
from PySide6.QtGui import (
    QColor, QFont, QLinearGradient,
    QPainter, QPainterPath, QPen, QBrush,
    QPolygonF, QRadialGradient,
)
from PySide6.QtWidgets import QWidget

from app.core.drone_physics import DroneState, FlightMode

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

SCALE     = 65.0    # pixels per metre
TRAIL_LEN = 200

# Mode colours (R,G,B)
_MODE_RGB: dict[str, tuple[int,int,int]] = {
    "DISARMED":  (88,  91, 112),
    "ARMED":     (250, 179, 135),
    "HOVER":     (137, 180, 250),
    "SPORT":     (243, 139, 168),
    "PRECISION": (166, 227, 161),
    "LANDING":   (249, 226, 175),
    "TAKEOFF":   (148, 226, 213),
}

# Direction banner colours
_DIR_COLOURS: dict[str, tuple[int,int,int]] = {
    "Forward":        (166, 227, 161),  # green
    "Backward":       (243, 139, 168),  # red/pink
    "Left":           (137, 180, 250),  # blue
    "Right":          (250, 179, 135),  # orange
    "Forward-Left":   (148, 226, 213),  # teal
    "Forward-Right":  (180, 190, 254),  # lavender
    "Backward-Left":  (249, 226, 175),  # yellow
    "Backward-Right": (245, 194, 231),  # pink
    "Climbing":       (166, 227, 161),
    "Descending":     (243, 139, 168),
    "Stable Hover":   (137, 180, 250),
    "Grounded":       (88,  91, 112),
}

# Pre-built pens — created once, reused every frame
_PN  = Qt.PenStyle.NoPen
_PEN_ARM      = QPen(QColor("#CDD6F4"), 4)
_PEN_BODY     = QPen(QColor("#313244"), 10, Qt.PenStyle.SolidLine, Qt.PenCapStyle.RoundCap)
_PEN_BODY_RIM = QPen(QColor("#89B4FA"), 2)
_PEN_MOTOR    = QPen(QColor("#89B4FA"), 1)
_PEN_GRID     = QPen(QColor(68, 71, 90, 90), 1)
_PEN_GRID_X   = QPen(QColor(243, 139, 168, 100), 1)   # red  = East/West
_PEN_GRID_Z   = QPen(QColor(137, 180, 250, 100), 1)   # blue = North/South
_PEN_ALTLINE  = QPen(QColor(137, 180, 250, 60), 1, Qt.PenStyle.DashLine)
_PEN_VEL      = QPen(QColor(166, 227, 161), 3)         # green velocity arrow
_PEN_HDG      = QPen(QColor(250, 179, 135), 2)         # orange heading arrow
_BRUSH_BODY   = QBrush(QColor("#313244"))
_BRUSH_MOTOR  = QBrush(QColor("#45475A"))
_LED_F        = QColor("#A6E3A1")   # front green
_LED_B        = QColor("#F38BA8")   # back  red
_ROTOR_RGB    = (137, 180, 250)


# ---------------------------------------------------------------------------
# Projection helpers
# ---------------------------------------------------------------------------

def _proj(wx: float, wy: float, wz: float,
          cx: float, cy: float,
          cc: float, sc: float) -> tuple[float, float]:
    rx =  wx * cc + wz * sc
    rz = -wx * sc + wz * cc
    return (cx + rx * SCALE - rz * SCALE * 0.45,
            cy - wy * SCALE + rz * SCALE * 0.22)


def _qpt(wx, wy, wz, cx, cy, cc, sc) -> QPointF:
    sx, sy = _proj(wx, wy, wz, cx, cy, cc, sc)
    return QPointF(sx, sy)


def _arrow(p: QPainter, tip: QPointF, base: QPointF,
           head_len: float = 12, head_w: float = 7) -> None:
    """Draw a filled arrowhead at tip pointing away from base."""
    dx = tip.x() - base.x()
    dy = tip.y() - base.y()
    length = math.sqrt(dx*dx + dy*dy)
    if length < 1:
        return
    ux, uy = dx / length, dy / length
    px, py = -uy, ux   # perpendicular
    p1 = QPointF(tip.x() - ux*head_len + px*head_w,
                 tip.y() - uy*head_len + py*head_w)
    p2 = QPointF(tip.x() - ux*head_len - px*head_w,
                 tip.y() - uy*head_len - py*head_w)
    poly = QPolygonF([tip, p1, p2])
    p.setPen(_PN)
    p.drawPolygon(poly)

# ---------------------------------------------------------------------------
# Widget
# ---------------------------------------------------------------------------

class Drone3DWidget(QWidget):
    """QPainter 3D drone view with clear directional cues."""

    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self._state   = DroneState()
        self._trail: deque[tuple[float,float,float]] = deque(maxlen=TRAIL_LEN)
        self._cam_yaw = 30.0
        self._cam_cos = math.cos(math.radians(30.0))
        self._cam_sin = math.sin(math.radians(30.0))
        self._fx = self._fy = self._fz = 0.0
        rng = random.Random(7)
        self._stars = [(rng.random(), rng.random() * 0.5)
                       for _ in range(70)]
        self.setMinimumSize(500, 400)
        self.setStyleSheet("background:#0D0E1A;")

    def update_state(self, state: DroneState) -> None:
        self._state = state
        if state.is_airborne or state.y > 0.05:
            self._trail.append((state.x, state.y, state.z))
        elif state.y < 0.01:
            self._trail.clear()
        a = 0.10
        self._fx += (state.x - self._fx) * a
        self._fy += (state.y - self._fy) * a
        self._fz += (state.z - self._fz) * a
        self.update()

    def set_camera_yaw(self, deg: float) -> None:
        self._cam_yaw = deg % 360.0
        rad = math.radians(self._cam_yaw)
        self._cam_cos = math.cos(rad)
        self._cam_sin = math.sin(rad)
        self.update()

    def orbit(self, delta: float) -> None:
        self.set_camera_yaw(self._cam_yaw + delta)

    # ------------------------------------------------------------------
    # Paint
    # ------------------------------------------------------------------

    def paintEvent(self, _ev) -> None:          # noqa: N802
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        w, h = self.width(), self.height()
        cc, sc = self._cam_cos, self._cam_sin

        # Screen origin — drone centred
        cx = w * 0.5 - self._fx * SCALE * 0.55
        cy = h * 0.5 + 80 - self._fy * SCALE - self._fz * SCALE * 0.15

        self._sky(p, w, h)
        self._ground_compass(p, cx, cy, cc, sc)
        self._grid(p, cx, cy, cc, sc)
        self._trail_draw(p, cx, cy, cc, sc)
        self._shadow(p, cx, cy, cc, sc)
        self._drone(p, cx, cy, cc, sc)
        self._velocity_arrow(p, cx, cy, cc, sc)
        self._hud(p, w, h)
        self._minimap(p, w, h)
        self._direction_banner(p, w)
        p.end()

    # ------------------------------------------------------------------
    # Sky
    # ------------------------------------------------------------------

    def _sky(self, p: QPainter, w: int, h: int) -> None:
        g = QLinearGradient(0, 0, 0, h)
        g.setColorAt(0.0, QColor("#0D0E1A"))
        g.setColorAt(0.6, QColor("#1A1B2E"))
        g.setColorAt(0.8, QColor("#252640"))
        g.setColorAt(1.0, QColor("#181825"))
        p.fillRect(0, 0, w, h, QBrush(g))
        p.setPen(QPen(QColor(255, 255, 255, 50), 1))
        for fx, fy in self._stars:
            p.drawPoint(int(fx * w), int(fy * h))

    # ------------------------------------------------------------------
    # Ground compass — large N/S/E/W labels ON the ground plane
    # ------------------------------------------------------------------

    def _ground_compass(self, p: QPainter, cx, cy, cc, sc) -> None:
        s   = self._state
        R   = 8.0   # compass radius in metres

        # Four cardinal directions in world space
        dirs = [
            ("N", 0, -R, QColor(166, 227, 161)),   # North = -Z = green
            ("S", 0,  R, QColor(243, 139, 168)),   # South = +Z = red
            ("E", R,  0, QColor(250, 179, 135)),   # East  = +X = orange
            ("W",-R,  0, QColor(137, 180, 250)),   # West  = -X = blue
        ]

        f = QFont("Segoe UI", 10, QFont.Weight.Bold)
        p.setFont(f)

        for lbl, dx, dz, col in dirs:
            sx, sy = _proj(s.x + dx, 0, s.z + dz, cx, cy, cc, sc)
            # Background circle
            p.setBrush(QBrush(QColor(col.red(), col.green(), col.blue(), 160)))
            p.setPen(_PN)
            p.drawEllipse(QPointF(sx, sy), 14, 9)
            # Label
            p.setPen(QPen(QColor("#0D0E1A")))
            p.drawText(int(sx)-14, int(sy)-9, 28, 18,
                       Qt.AlignmentFlag.AlignCenter, lbl)

        # Heading ray from drone (orange line showing which way drone faces)
        yr  = math.radians(s.yaw)
        # Drone nose is in +X * sin(yaw) - Z * cos(yaw) direction
        nx  = s.x + math.sin(yr) * 4.0
        nz  = s.z - math.cos(yr) * 4.0
        p.setPen(QPen(QColor(250, 179, 135, 180), 2))
        p.drawLine(_qpt(s.x, 0, s.z, cx, cy, cc, sc),
                   _qpt(nx,  0, nz,  cx, cy, cc, sc))

    # ------------------------------------------------------------------
    # Grid with coloured axes and coordinate labels
    # ------------------------------------------------------------------

    def _grid(self, p: QPainter, cx, cy, cc, sc) -> None:
        s   = self._state
        GAP = 2.0
        N   = 14
        ox  = round(s.x / GAP) * GAP
        oz  = round(s.z / GAP) * GAP

        for i in range(-N, N + 1):
            # Z-parallel lines (East-West direction) = Red tint
            wz = oz + i * GAP
            if abs(wz - oz) < 0.01:
                p.setPen(_PEN_GRID_X)
            else:
                p.setPen(_PEN_GRID)
            p.drawLine(_qpt(ox - N*GAP, 0, wz, cx, cy, cc, sc),
                       _qpt(ox + N*GAP, 0, wz, cx, cy, cc, sc))

            # X-parallel lines (North-South direction) = Blue tint
            wx = ox + i * GAP
            if abs(wx - ox) < 0.01:
                p.setPen(_PEN_GRID_Z)
            else:
                p.setPen(_PEN_GRID)
            p.drawLine(_qpt(wx, 0, oz - N*GAP, cx, cy, cc, sc),
                       _qpt(wx, 0, oz + N*GAP, cx, cy, cc, sc))

        # Coordinate labels every 10 m
        p.setFont(QFont("Consolas", 7))
        for d in (-10, -5, 0, 5, 10):
            wx = round((s.x + d) / 5) * 5
            sx, sy = _proj(wx, 0, oz, cx, cy, cc, sc)
            p.setPen(QPen(QColor(137, 180, 250, 120)))
            p.drawText(int(sx)-12, int(sy)+2, 24, 12,
                       Qt.AlignmentFlag.AlignCenter, f"{int(wx)}")
            wz = round((s.z + d) / 5) * 5
            sx, sy = _proj(ox, 0, wz, cx, cy, cc, sc)
            p.setPen(QPen(QColor(243, 139, 168, 120)))
            p.drawText(int(sx)+3, int(sy)-6, 24, 12,
                       Qt.AlignmentFlag.AlignLeft, f"{int(wz)}")

    # ------------------------------------------------------------------
    # Trail
    # ------------------------------------------------------------------

    def _trail_draw(self, p, cx, cy, cc, sc) -> None:
        trail = list(self._trail)
        n = len(trail)
        if n < 2:
            return
        for i in range(1, n):
            a = int(220 * i / n)
            p.setPen(QPen(QColor(166, 227, 161, a), 2))
            ax, ay = _proj(*trail[i-1], cx, cy, cc, sc)
            bx, by = _proj(*trail[i],   cx, cy, cc, sc)
            p.drawLine(QPointF(ax, ay), QPointF(bx, by))

    # ------------------------------------------------------------------
    # Shadow
    # ------------------------------------------------------------------

    def _shadow(self, p, cx, cy, cc, sc) -> None:
        s = self._state
        if s.y < 0.1:
            return
        alpha = max(0, int(150 - s.y * 7))
        if alpha <= 0:
            return
        sx, sy = _proj(s.x, 0, s.z, cx, cy, cc, sc)
        rx = max(6, int(44 - s.y * 1.5))
        p.setPen(_PN)
        p.setBrush(QBrush(QColor(0, 0, 0, alpha)))
        p.drawEllipse(QPointF(sx, sy), rx, int(rx * 0.32))

    # ------------------------------------------------------------------
    # Drone
    # ------------------------------------------------------------------

    def _drone(self, p, cx, cy, cc, sc) -> None:
        s   = self._state
        ARM = 0.44
        yr  = math.radians(s.yaw)
        pr  = math.radians(s.pitch)
        rr  = math.radians(s.roll)
        cyr, syr = math.cos(yr), math.sin(yr)
        cpr, spr = math.cos(pr), math.sin(pr)
        crr, srr = math.cos(rr), math.sin(rr)

        def b2w(bx, by, bz):
            # Roll → Pitch → Yaw
            # Roll (around body Z)
            cx2 =  bx*crr - by*srr;  cy2 =  bx*srr + by*crr
            # Pitch (around body X)
            dx  =  cx2;               dy2 =  cy2*cpr - bz*spr; dz = cy2*spr + bz*cpr
            # Yaw (around world Y) — convention: forward = (sin(yaw), 0, -cos(yaw))
            # ex = dx*cyr - dz*syr  (NOT dx*cyr + dz*syr)
            ex  =  dx*cyr - dz*syr;   ez  =  dx*syr + dz*cyr
            return ex + s.x, dy2 + s.y, ez + s.z

        motors = [b2w( ARM, 0, -ARM),   # FR — front-right  (Z=-ARM = NORTH = forward)
                  b2w(-ARM, 0, -ARM),   # FL — front-left
                  b2w(-ARM, 0,  ARM),   # BL — back-left   (Z=+ARM = SOUTH = backward)
                  b2w( ARM, 0,  ARM)]   # BR — back-right

        def wp(wx, wy, wz): return _qpt(wx, wy, wz, cx, cy, cc, sc)
        cpt = wp(s.x, s.y, s.z)

        # Arms
        p.setPen(_PEN_ARM)
        for m in motors:
            p.drawLine(cpt, wp(*m))

        # Body pillar
        p.setPen(_PEN_BODY)
        p.drawLine(wp(*b2w(0,-0.07,0)), wp(*b2w(0, 0.14,0)))
        p.setBrush(_BRUSH_BODY)
        p.setPen(_PEN_BODY_RIM)
        p.drawEllipse(cpt, 15, 9)

        # Motor pods
        p.setBrush(_BRUSH_MOTOR); p.setPen(_PEN_MOTOR)
        for m in motors:
            p.drawEllipse(wp(*m), 8, 5)

        # Rotors
        if s.rotor_speed > 0.02:
            self._rotors(p, cx, cy, cc, sc, motors, s)

        # LEDs — front green, back red
        if s.mode != FlightMode.DISARMED:
            la = 220 if s.rotor_speed > 0.1 else 80
            for i, m in enumerate(motors):
                base = _LED_F if i < 2 else _LED_B
                p.setBrush(QBrush(QColor(base.red(),base.green(),base.blue(),la)))
                p.setPen(_PN)
                p.drawEllipse(wp(*m), 5, 3)

        # NOSE marker — computed directly from heading angle so it ALWAYS
        # matches the orange heading arrow drawn on the ground.
        # sin(yaw) = East component, -cos(yaw) = North component (same as heading arrow).
        nose_dist = ARM * 1.1
        nose_wx = s.x + math.sin(yr) * nose_dist
        nose_wy = s.y
        nose_wz = s.z - math.cos(yr) * nose_dist
        p.setBrush(QBrush(QColor(249, 226, 175, 230)))
        p.setPen(QPen(QColor("#0D0E1A"), 1))
        p.drawEllipse(wp(nose_wx, nose_wy, nose_wz), 6, 4)
        # Label "FWD" near nose
        nsx, nsy = _proj(nose_wx, nose_wy, nose_wz, cx, cy, cc, sc)
        p.setPen(QPen(QColor(249, 226, 175)))
        p.setFont(QFont("Consolas", 8, QFont.Weight.Bold))
        p.drawText(int(nsx)-12, int(nsy)-14, 24, 12,
                   Qt.AlignmentFlag.AlignCenter, "FWD")

        # Altitude dashed line to ground
        if s.y > 0.4:
            p.setPen(_PEN_ALTLINE)
            p.drawLine(wp(s.x, 0, s.z), cpt)

        # Rotor wash glow
        if s.rotor_speed > 0.18 and s.y > 0.3:
            ga = int(s.rotor_speed * 55)
            glow = QRadialGradient(cpt, 72)
            r, g, b = _ROTOR_RGB
            glow.setColorAt(0.0, QColor(r,g,b,ga)); glow.setColorAt(1.0, QColor(r,g,b,0))
            p.setBrush(QBrush(glow)); p.setPen(_PN)
            p.drawEllipse(cpt, 72, 28)

    def _rotors(self, p, cx, cy, cc, sc, motors, s: DroneState) -> None:
        R     = 0.34
        alpha = min(210, int(s.rotor_speed * 210))
        r, g, b = _ROTOR_RGB
        dc    = QColor(r, g, b, alpha)
        bp    = QPen(QColor(205, 214, 244, min(alpha+40, 255)), 2)
        for i, (mx, my, mz) in enumerate(motors):
            mp  = _qpt(mx, my, mz, cx, cy, cc, sc)
            ang = s.rotor_angles[i]
            pts = [_qpt(mx+math.cos(math.radians(ang+k*45))*R, my,
                        mz+math.sin(math.radians(ang+k*45))*R, cx, cy, cc, sc)
                   for k in range(8)]
            p.setBrush(QBrush(dc))
            p.setPen(QPen(QColor(r,g,b,min(alpha,180)),1))
            p.drawPolygon(QPolygonF(pts))
            p.setPen(bp)
            for a_off in (ang, ang+180):
                ar = math.radians(a_off)
                p.drawLine(mp, _qpt(mx+math.cos(ar)*R, my,
                                    mz+math.sin(ar)*R, cx, cy, cc, sc))

    # ------------------------------------------------------------------
    # Velocity vector arrow — shows actual movement direction
    # ------------------------------------------------------------------

    def _velocity_arrow(self, p, cx, cy, cc, sc) -> None:
        s    = self._state
        spd  = s.speed_h
        if spd < 0.3:
            return
        scale = min(3.0, spd / 2.0)   # arrow length scales with speed

        # World velocity direction
        vlen = math.sqrt(s.vx**2 + s.vz**2)
        if vlen < 0.01:
            return
        uvx, uvz = s.vx / vlen, s.vz / vlen

        # Arrow tip in world space
        tx  = s.x + uvx * scale
        tz  = s.z + uvz * scale

        base_pt = _qpt(s.x, s.y, s.z, cx, cy, cc, sc)
        tip_pt  = _qpt(tx,  s.y, tz,  cx, cy, cc, sc)

        p.setPen(_PEN_VEL)
        p.drawLine(base_pt, tip_pt)
        p.setBrush(QBrush(QColor(166, 227, 161)))
        _arrow(p, tip_pt, base_pt, head_len=10, head_w=6)

        # Speed label near tip
        p.setFont(QFont("Consolas", 9, QFont.Weight.Bold))
        p.setPen(QPen(QColor(166, 227, 161)))
        p.drawText(int(tip_pt.x())+4, int(tip_pt.y())-6, 50, 14,
                   Qt.AlignmentFlag.AlignLeft, f"{spd:.1f}m/s")

    # ------------------------------------------------------------------
    # Direction banner — large centred text below mode badge
    # ------------------------------------------------------------------

    def _direction_banner(self, p: QPainter, w: int) -> None:
        s   = self._state
        cmd = s.flight_command
        if not cmd or cmd in ("Disarmed — press Button 1 to ARM",):
            return

        # Get colour for this command
        rgb = (88, 91, 112)
        for key, col in _DIR_COLOURS.items():
            if key.lower() in cmd.lower():
                rgb = col
                break

        r, g, b = rgb
        f = QFont("Segoe UI", 16, QFont.Weight.Bold)
        p.setFont(f)
        fm    = p.fontMetrics()
        tw    = fm.horizontalAdvance(cmd)
        bw    = tw + 24
        bh    = fm.height() + 10
        bx    = (w - bw) // 2
        by    = 52   # below mode badge

        p.setBrush(QBrush(QColor(r, g, b, 200)))
        p.setPen(_PN)
        p.drawRoundedRect(bx, by, bw, bh, 8, 8)
        p.setPen(QPen(QColor("#0D0E1A")))
        p.drawText(bx, by, bw, bh, Qt.AlignmentFlag.AlignCenter, cmd)

    # ------------------------------------------------------------------
    # HUD top strip
    # ------------------------------------------------------------------

    def _hud(self, p: QPainter, w: int, h: int) -> None:
        s = self._state
        self._hud_mode_badge(p, w, s)
        self._hud_telem(p, s)
        self._hud_compass(p, w, s)
        self._hud_attitude(p, h, s)
        self._hud_bars(p, w, h, s)

    def _hud_mode_badge(self, p, w, s: DroneState) -> None:
        mode  = s.mode.value
        rgb   = _MODE_RGB.get(mode, (88,91,112))
        text  = f"  {mode}  "
        f     = QFont("Segoe UI", 11, QFont.Weight.Bold)
        p.setFont(f)
        fm    = p.fontMetrics()
        bw    = fm.horizontalAdvance(text) + 10
        bh    = fm.height() + 6
        bx    = (w - bw) // 2
        by    = 10
        p.setBrush(QBrush(QColor(*rgb, 210)))
        p.setPen(_PN)
        p.drawRoundedRect(bx, by, bw, bh, 5, 5)
        p.setPen(QPen(QColor("#0D0E1A")))
        p.drawText(bx, by, bw, bh, Qt.AlignmentFlag.AlignCenter, text)

    def _hud_telem(self, p, s: DroneState) -> None:
        lines = [
            ("ALT",   f"{s.altitude:6.1f} m"),
            ("H-SPD", f"{s.speed_h:5.1f} m/s"),
            ("V-SPD", f"{s.speed_v:+5.1f} m/s"),
            ("HDG",   f"{s.heading:5.1f}°"),
            ("X",     f"{s.x:6.1f} m"),
            ("Z",     f"{s.z:6.1f} m"),
        ]
        p.setFont(QFont("Consolas", 10))
        x, y = 8, 12
        for lbl, val in lines:
            p.setPen(QPen(QColor("#585B70")))
            p.drawText(x, y, 48, 18, Qt.AlignmentFlag.AlignRight, lbl)
            p.setPen(QPen(QColor("#CDD6F4")))
            p.drawText(x+52, y, 90, 18, Qt.AlignmentFlag.AlignLeft, val)
            y += 19

    def _hud_compass(self, p, w, s: DroneState) -> None:
        cx, cy, r = w-52, 52, 38
        p.setBrush(QBrush(QColor(13,14,26,200)))
        p.setPen(QPen(QColor("#45475A"),1))
        p.drawEllipse(QPointF(cx,cy), r, r)
        p.setFont(QFont("Segoe UI", 8, QFont.Weight.Bold))
        for lbl, ang, col in (
            ("N", 0,   QColor("#A6E3A1")),
            ("E", 90,  QColor(250,179,135)),
            ("S", 180, QColor(243,139,168)),
            ("W", 270, QColor(137,180,250)),
        ):
            rad = math.radians(ang - s.heading)
            lx  = cx + (r-11)*math.sin(rad)
            ly  = cy - (r-11)*math.cos(rad)
            p.setPen(QPen(col))
            p.drawText(int(lx)-7,int(ly)-7,14,14, Qt.AlignmentFlag.AlignCenter, lbl)
        # Heading needle
        rad = math.radians(-s.heading)
        tip = QPointF(cx + (r-6)*math.sin(0), cy - (r-6)*math.cos(0))
        p.setPen(QPen(QColor("#F38BA8"),2))
        p.drawLine(QPointF(cx,cy), tip)
        p.setBrush(QBrush(QColor("#CDD6F4"))); p.setPen(_PN)
        p.drawEllipse(QPointF(cx,cy), 3, 3)
        # HDG text
        p.setPen(QPen(QColor("#CDD6F4")))
        p.setFont(QFont("Consolas", 8))
        p.drawText(int(cx)-18, int(cy)+r+2, 36, 12,
                   Qt.AlignmentFlag.AlignCenter, f"{s.heading:.0f}°")

    def _hud_attitude(self, p, h, s: DroneState) -> None:
        cx, cy, r = 52, h-62, 38
        p.setClipRect(int(cx-r), int(cy-r), r*2, r*2)
        p.setBrush(QBrush(QColor(13,71,161,160))); p.setPen(_PN)
        p.drawEllipse(QPointF(cx,cy), r, r)
        po  = s.pitch * (r/50.0)
        rr  = math.radians(-s.roll)
        cr, sr = math.cos(rr), math.sin(rr)
        path = QPainterPath()
        hx1 = cx - r*cr - po*sr; hy1 = cy - r*sr + po*cr
        hx2 = cx + r*cr + po*sr; hy2 = cy + r*sr - po*cr
        path.moveTo(hx1,hy1); path.lineTo(hx2,hy2)
        path.lineTo(cx+r, cy+r); path.lineTo(cx-r, cy+r)
        path.closeSubpath()
        p.setBrush(QBrush(QColor(95,56,28,160))); p.drawPath(path)
        p.setClipping(False)
        p.setBrush(Qt.BrushStyle.NoBrush)
        p.setPen(QPen(QColor("#45475A"),2))
        p.drawEllipse(QPointF(cx,cy), r, r)
        p.setPen(QPen(QColor("#FAB387"),2))
        p.drawLine(int(cx)-10,int(cy), int(cx)-4,int(cy))
        p.drawLine(int(cx)+4, int(cy), int(cx)+10,int(cy))
        p.drawLine(int(cx),int(cy)-4, int(cx),int(cy)+4)

    def _hud_bars(self, p, w, h, s: DroneState) -> None:
        bh = 100; bw = 12; by = h - bh - 12
        self._vbar(p, w-34, by, bw, bh, s.altitude/30.0, QColor("#89B4FA"), "ALT")
        self._vbar(p, w-18, by, bw, bh, s.throttle,      QColor("#A6E3A1"), "THR")

    def _vbar(self, p, x, y, bw, bh, frac, col, lbl) -> None:
        frac = max(0.0, min(1.0, frac))
        p.setBrush(QBrush(QColor(13,14,26,190)))
        p.setPen(QPen(QColor("#45475A"),1))
        p.drawRect(x, y, bw, bh)
        fh = int(bh * frac)
        if fh > 0:
            p.setBrush(QBrush(col)); p.setPen(_PN)
            p.drawRect(x+1, y+bh-fh, bw-2, fh)
        p.setPen(QPen(QColor("#A6ADC8")))
        p.setFont(QFont("Consolas", 7))
        p.drawText(x-2, y+bh+2, bw+4, 12, Qt.AlignmentFlag.AlignCenter, lbl)

    # ------------------------------------------------------------------
    # Mini top-down map (bottom-right)
    # Shows position, heading, trail and N/S/E/W labels
    # ------------------------------------------------------------------

    def _minimap(self, p: QPainter, w: int, h: int) -> None:
        s     = self._state
        MR    = 70      # map radius px
        MSCALE= 4.5     # px per metre in minimap
        mx    = w - MR - 8
        my    = h - MR - 8

        # Background
        p.setBrush(QBrush(QColor(13, 14, 26, 200)))
        p.setPen(QPen(QColor("#45475A"), 1))
        p.drawEllipse(QPointF(mx, my), MR, MR)

        # Grid lines
        p.setPen(QPen(QColor(68, 71, 90, 80), 1))
        for offset in (-20, -10, 0, 10, 20):
            ox = mx + offset * MSCALE / 10
            oy = my + offset * MSCALE / 10
            p.drawLine(QPointF(ox, my - MR + 4), QPointF(ox, my + MR - 4))
            p.drawLine(QPointF(mx - MR + 4, oy), QPointF(mx + MR - 4, oy))

        # Cardinal labels
        p.setFont(QFont("Segoe UI", 7, QFont.Weight.Bold))
        for lbl, dx, dy, col in (
            ("N", 0, -1, QColor("#A6E3A1")),
            ("S", 0,  1, QColor(243,139,168)),
            ("E", 1,  0, QColor(250,179,135)),
            ("W",-1,  0, QColor(137,180,250)),
        ):
            p.setPen(QPen(col))
            lx = mx + dx * (MR - 8)
            ly = my + dy * (MR - 8)
            p.drawText(int(lx)-6, int(ly)-6, 12, 12,
                       Qt.AlignmentFlag.AlignCenter, lbl)

        # Trail dots
        p.setClipRect(int(mx-MR), int(my-MR), MR*2, MR*2)
        trail = list(self._trail)
        n = len(trail)
        for i, (tx, ty_alt, tz) in enumerate(trail):
            a  = int(160 * i / max(n, 1))
            rx = mx + (tx - s.x) * MSCALE
            ry = my + (tz - s.z) * MSCALE
            p.setBrush(QBrush(QColor(166, 227, 161, a)))
            p.setPen(_PN)
            p.drawEllipse(QPointF(rx, ry), 2, 2)

        # Drone position dot
        p.setBrush(QBrush(QColor("#89B4FA")))
        p.setPen(QPen(QColor("#CDD6F4"), 1))
        p.drawEllipse(QPointF(mx, my), 5, 5)

        # Heading arrow
        yr = math.radians(s.yaw)
        ax = mx + math.sin(yr) * 14
        ay = my - math.cos(yr) * 14
        p.setPen(QPen(QColor(250, 179, 135), 2))
        p.drawLine(QPointF(mx, my), QPointF(ax, ay))
        p.setBrush(QBrush(QColor(250,179,135)))
        _arrow(p, QPointF(ax, ay), QPointF(mx, my), head_len=6, head_w=4)

        p.setClipping(False)

        # Position text below map
        p.setPen(QPen(QColor("#585B70")))
        p.setFont(QFont("Consolas", 7))
        p.drawText(int(mx-MR), int(my+MR+2), MR*2, 12,
                   Qt.AlignmentFlag.AlignCenter,
                   f"X:{s.x:.1f} Z:{s.z:.1f}")
