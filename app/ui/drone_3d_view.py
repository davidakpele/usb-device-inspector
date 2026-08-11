"""3D QPainter drone renderer — drone_3d_view.py

Draws the drone world using a custom oblique/isometric projection.
No OpenGL or third-party 3D library required — pure QPainter.

Projection used:
  screen_x = world_x * SCALE - world_z * SCALE * 0.5
  screen_y = -world_y * SCALE + world_z * SCALE * 0.25
  (origin = centre of widget)

Layers drawn back-to-front:
  1. Sky gradient background
  2. Ground grid (receding perspective lines)
  3. Flight trail (fading polyline)
  4. Ground shadow (ellipse below drone)
  5. Drone body (fuselage cross + arm spars)
  6. Four rotors (spinning ellipses)
  7. Rotor wash glow (altitude-dependent)
  8. HUD overlay (altitude, speed, heading compass, mode badge)
  9. Horizon / attitude indicator (pitch + roll lines)
"""
from __future__ import annotations

import math
from collections import deque
from typing import Sequence

from PySide6.QtCore import QPointF, QRectF, Qt
from PySide6.QtGui import (
    QColor, QFont, QLinearGradient, QPainter,
    QPainterPath, QPen, QBrush, QRadialGradient, QPolygonF,
)
from PySide6.QtWidgets import QWidget

from app.core.drone_physics import DroneState, FlightMode

# ---------------------------------------------------------------------------
# Projection helpers
# ---------------------------------------------------------------------------

SCALE      = 80.0   # pixels per metre  (was 40 — drone was a speck)
TRAIL_LEN  = 180    # number of trail points kept

# Mode badge colours
_MODE_COLOURS: dict[str, str] = {
    "DISARMED":  "#585B70",
    "ARMED":     "#FAB387",
    "HOVER":     "#89B4FA",
    "SPORT":     "#F38BA8",
    "PRECISION": "#A6E3A1",
    "LANDING":   "#F9E2AF",
    "TAKEOFF":   "#94E2D5",
}

# Drone arm colours
_ARM_COLOUR    = QColor("#CDD6F4")
_ROTOR_COLOUR  = QColor(137, 180, 250, 180)   # #89B4FA semi-transparent
_BODY_COLOUR   = QColor("#313244")
_MOTOR_COLOUR  = QColor("#45475A")
_LED_FRONT     = QColor("#A6E3A1")   # green LEDs front
_LED_BACK      = QColor("#F38BA8")   # red  LEDs back


def _project(wx: float, wy: float, wz: float,
             cx: float, cy: float,
             cam_yaw: float = 0.0) -> tuple[float, float]:
    """Oblique projection: world (x, y, z) → screen (sx, sy).

    cam_yaw rotates the whole world around Y so the camera can orbit.
    """
    rad = math.radians(cam_yaw)
    rx = wx * math.cos(rad) + wz * math.sin(rad)
    rz = -wx * math.sin(rad) + wz * math.cos(rad)

    sx = cx + rx * SCALE - rz * SCALE * 0.45
    sy = cy - wy * SCALE + rz * SCALE * 0.22
    return sx, sy


def _pt(wx: float, wy: float, wz: float,
        cx: float, cy: float, cam_yaw: float = 0.0) -> QPointF:
    sx, sy = _project(wx, wy, wz, cx, cy, cam_yaw)
    return QPointF(sx, sy)

# ---------------------------------------------------------------------------
# Main widget
# ---------------------------------------------------------------------------

class Drone3DWidget(QWidget):
    """Pure-QPainter 3D drone visualiser.

    Call ``update_state(state)`` from the simulator every physics frame.
    Camera auto-follows the drone horizontally; the user can orbit with
    ``set_camera_yaw(deg)``.
    """

    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self._state   = DroneState()
        self._trail:  deque[tuple[float, float, float]] = deque(maxlen=TRAIL_LEN)
        self._cam_yaw   = 25.0    # initial 3/4 front-left view
        self._cam_follow_x = 0.0  # world X the camera is currently centred on
        self._cam_follow_y = 0.0  # world Y (altitude) — camera rises with drone
        self._cam_follow_z = 0.0  # world Z depth
        self.setMinimumSize(500, 400)
        self.setStyleSheet("background:#11111B;")

    # ------------------------------------------------------------------
    # Public
    # ------------------------------------------------------------------

    def update_state(self, state: DroneState) -> None:
        self._state = state
        if state.is_airborne or state.y > 0.05:
            self._trail.append((state.x, state.y, state.z))
        elif state.y < 0.01:
            self._trail.clear()

        # Full 3-axis camera follow so the drone stays centred regardless
        # of how far it moves horizontally, vertically, or in depth.
        # Alpha = 0.08 gives smooth lag (~10 frames to close 50 % of the gap).
        alpha = 0.08
        self._cam_follow_x += (state.x - self._cam_follow_x) * alpha
        self._cam_follow_y += (state.y - self._cam_follow_y) * alpha
        self._cam_follow_z += (state.z - self._cam_follow_z) * alpha
        self.update()

    def set_camera_yaw(self, deg: float) -> None:
        self._cam_yaw = deg % 360.0
        self.update()

    def orbit(self, delta_deg: float) -> None:
        self._cam_yaw = (self._cam_yaw + delta_deg) % 360.0
        self.update()

    # ------------------------------------------------------------------
    # Paint
    # ------------------------------------------------------------------

    def paintEvent(self, _event) -> None:   # noqa: N802
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        w, h = self.width(), self.height()

        # Camera origin:
        #   cx follows the drone's world-X position (side-to-side)
        #   cy follows both world-Y (altitude) AND world-Z (depth)
        #   so the drone stays locked in the middle of the screen at
        #   all times — even when flying high or far away.
        cx = w / 2.0 - self._cam_follow_x * SCALE * 0.55
        cy = (h / 2.0 + 80.0
              - self._cam_follow_y * SCALE           # rise with altitude
              - self._cam_follow_z * SCALE * 0.15)   # recede with depth

        self._draw_sky(p, w, h)
        self._draw_ground_grid(p, cx, cy)
        self._draw_trail(p, cx, cy)
        self._draw_shadow(p, cx, cy)
        self._draw_drone(p, cx, cy)
        self._draw_hud(p, w, h)
        p.end()

    # ------------------------------------------------------------------
    # Sky
    # ------------------------------------------------------------------

    def _draw_sky(self, p: QPainter, w: int, h: int) -> None:
        grad = QLinearGradient(0, 0, 0, h)
        grad.setColorAt(0.0,  QColor("#0D0E1A"))   # deep night blue
        grad.setColorAt(0.55, QColor("#1A1B2E"))
        grad.setColorAt(0.75, QColor("#2B2D42"))   # horizon haze
        grad.setColorAt(1.0,  QColor("#181825"))   # ground level
        p.fillRect(0, 0, w, h, QBrush(grad))

        # Stars (static pattern seeded by position hash)
        p.setPen(QPen(QColor(255, 255, 255, 60), 1))
        import random; rng = random.Random(42)
        for _ in range(80):
            sx = rng.randint(0, w)
            sy = rng.randint(0, int(h * 0.55))
            p.drawPoint(sx, sy)

    # ------------------------------------------------------------------
    # Ground grid
    # ------------------------------------------------------------------

    def _draw_ground_grid(self, p: QPainter, cx: float, cy: float) -> None:
        s = self._state
        GRID = 2.0      # grid spacing metres
        HALF = 14       # half-grid lines each side

        grid_pen = QPen(QColor(68, 71, 90, 120), 1)   # #45475A dim
        axis_pen = QPen(QColor(137, 180, 250, 80), 1)

        # Snap grid origin to drone X/Z so the ground scrolls with drone
        orig_x = round(s.x / GRID) * GRID
        orig_z = round(s.z / GRID) * GRID

        for i in range(-HALF, HALF + 1):
            is_axis = (i == 0)
            p.setPen(axis_pen if is_axis else grid_pen)

            # Lines parallel to X axis (Z-varying)
            wz = orig_z + i * GRID
            p1 = _pt(orig_x - HALF * GRID, 0, wz, cx, cy, self._cam_yaw)
            p2 = _pt(orig_x + HALF * GRID, 0, wz, cx, cy, self._cam_yaw)
            p.drawLine(p1, p2)

            # Lines parallel to Z axis (X-varying)
            wx = orig_x + i * GRID
            p3 = _pt(wx, 0, orig_z - HALF * GRID, cx, cy, self._cam_yaw)
            p4 = _pt(wx, 0, orig_z + HALF * GRID, cx, cy, self._cam_yaw)
            p.drawLine(p3, p4)

    # ------------------------------------------------------------------
    # Flight trail
    # ------------------------------------------------------------------

    def _draw_trail(self, p: QPainter, cx: float, cy: float) -> None:
        trail = list(self._trail)
        n = len(trail)
        if n < 2:
            return
        for i in range(1, n):
            alpha = int(200 * i / n)
            pen = QPen(QColor(166, 227, 161, alpha), 2)   # #A6E3A1
            p.setPen(pen)
            a = _pt(*trail[i - 1], cx, cy, self._cam_yaw)
            b = _pt(*trail[i],     cx, cy, self._cam_yaw)
            p.drawLine(a, b)

    # ------------------------------------------------------------------
    # Ground shadow
    # ------------------------------------------------------------------

    def _draw_shadow(self, p: QPainter, cx: float, cy: float) -> None:
        s = self._state
        if s.y < 0.05:
            return
        # Shadow on ground (y=0) — grows fainter with altitude
        alpha = max(0, int(140 - s.y * 8))
        if alpha <= 0:
            return
        sx, sy = _project(s.x, 0, s.z, cx, cy, self._cam_yaw)
        radius_x = int(max(8, 40 - s.y * 2))
        radius_y = int(radius_x * 0.35)
        p.setPen(Qt.PenStyle.NoPen)
        p.setBrush(QBrush(QColor(0, 0, 0, alpha)))
        p.drawEllipse(QPointF(sx, sy), radius_x, radius_y)

    # ------------------------------------------------------------------
    # Drone body
    # ------------------------------------------------------------------

    def _draw_drone(self, p: QPainter, cx: float, cy: float) -> None:
        s = self._state
        dy = s.y   # world altitude

        # Drone body axes in local frame (before pitch/roll/yaw)
        ARM = 0.45    # arm length metres  (was 0.35 — now clearly visible)
        yaw_r   = math.radians(s.yaw)
        pitch_r = math.radians(s.pitch)
        roll_r  = math.radians(s.roll)

        # Motor positions in body frame (X=right, Z=back)
        motors_body = [
            ( ARM,  0,  ARM),   # front-right
            (-ARM,  0,  ARM),   # front-left
            (-ARM,  0, -ARM),   # back-left
            ( ARM,  0, -ARM),   # back-right
        ]

        # Rotate body frame → world frame
        def body_to_world(bx: float, by: float, bz: float) -> tuple[float, float, float]:
            # Roll (around Z axis in body)
            cx2 =  bx * math.cos(roll_r) - by * math.sin(roll_r)
            cy2 =  bx * math.sin(roll_r) + by * math.cos(roll_r)
            cz2 = bz
            # Pitch (around X axis in body)
            dx = cx2
            dy2 =  cy2 * math.cos(pitch_r) - cz2 * math.sin(pitch_r)
            dz =   cy2 * math.sin(pitch_r) + cz2 * math.cos(pitch_r)
            # Yaw (around Y axis in world)
            ex =  dx * math.cos(yaw_r) + dz * math.sin(yaw_r)
            ey =  dy2
            ez = -dx * math.sin(yaw_r) + dz * math.cos(yaw_r)
            return ex + s.x, ey + dy, ez + s.z

        motor_world = [body_to_world(*m) for m in motors_body]

        # Centre world point
        cx_w = s.x
        cy_w = dy
        cz_w = s.z

        def wp(wx, wy, wz):
            return _pt(wx, wy, wz, cx, cy, self._cam_yaw)

        centre_pt = wp(cx_w, cy_w, cz_w)

        # ── Arms ─────────────────────────────────────────────────────
        p.setPen(QPen(_ARM_COLOUR, 4))
        for mx, my, mz in motor_world:
            p.drawLine(centre_pt, wp(mx, my, mz))

        # ── Body fuselage ────────────────────────────────────────────
        body_top    = body_to_world(0,  0.14, 0)
        body_bottom = body_to_world(0, -0.07, 0)
        p.setPen(QPen(_BODY_COLOUR, 10, Qt.PenStyle.SolidLine,
                      Qt.PenCapStyle.RoundCap))
        p.drawLine(wp(*body_bottom), wp(*body_top))

        p.setBrush(QBrush(_BODY_COLOUR))
        p.setPen(QPen(QColor("#89B4FA"), 2))
        p.drawEllipse(centre_pt, 16, 10)

        # ── Motor pods ───────────────────────────────────────────────
        p.setBrush(QBrush(_MOTOR_COLOUR))
        p.setPen(QPen(QColor("#89B4FA"), 1))
        for mx, my, mz in motor_world:
            p.drawEllipse(wp(mx, my, mz), 8, 5)

        # ── Rotors ───────────────────────────────────────────────────
        self._draw_rotors(p, cx, cy, motor_world, s)

        # ── LEDs ─────────────────────────────────────────────────────
        if s.mode.value != "DISARMED":
            led_alpha = 220 if s.rotor_speed > 0.1 else 100
            for i, (mx, my, mz) in enumerate(motor_world):
                colour = _LED_FRONT if i < 2 else _LED_BACK
                colour = QColor(colour.red(), colour.green(),
                                colour.blue(), led_alpha)
                p.setBrush(QBrush(colour))
                p.setPen(Qt.PenStyle.NoPen)
                p.drawEllipse(wp(mx, my, mz), 5, 3)

        # ── Altitude line (vertical line from ground to drone) ───────
        if s.y > 0.3:
            alt_pen = QPen(QColor(137, 180, 250, 60), 1,
                           Qt.PenStyle.DashLine)
            p.setPen(alt_pen)
            ground_pt = wp(s.x, 0.0, s.z)
            p.drawLine(ground_pt, centre_pt)

        # ── Rotor wash / glow ────────────────────────────────────────
        if s.rotor_speed > 0.15 and s.y > 0.2:
            glow_alpha = int(s.rotor_speed * 60)
            glow = QRadialGradient(centre_pt, 75)
            glow.setColorAt(0.0, QColor(137, 180, 250, glow_alpha))
            glow.setColorAt(1.0, QColor(137, 180, 250, 0))
            p.setBrush(QBrush(glow))
            p.setPen(Qt.PenStyle.NoPen)
            p.drawEllipse(centre_pt, 75, 28)

    def _draw_rotors(self, p: QPainter, cx: float, cy: float,
                     motor_world: list[tuple[float, float, float]],
                     s: DroneState) -> None:
        """Draw spinning rotor discs — ellipses tilted with pitch/roll."""
        if s.rotor_speed < 0.02:
            return
        alpha = min(220, int(s.rotor_speed * 220))
        ROTOR_R = 0.36   # rotor disc radius metres  (was 0.28)

        for i, (mx, my, mz) in enumerate(motor_world):
            mp = _pt(mx, my, mz, cx, cy, self._cam_yaw)

            # Ellipse axes in screen space (project 4 rim points)
            angle = s.rotor_angles[i]
            rim_pts = []
            for a in range(0, 360, 45):
                rad = math.radians(a + angle)
                rx = math.cos(rad) * ROTOR_R
                rz = math.sin(rad) * ROTOR_R
                rim_pts.append(_pt(mx + rx, my, mz + rz,
                                   cx, cy, self._cam_yaw))

            # Draw rotor disc as polygon
            poly = QPolygonF(rim_pts)
            colour = QColor(_ROTOR_COLOUR.red(), _ROTOR_COLOUR.green(),
                            _ROTOR_COLOUR.blue(), alpha)
            p.setBrush(QBrush(colour))
            p.setPen(QPen(QColor(137, 180, 250, min(alpha, 180)), 1))
            p.drawPolygon(poly)

            # Blade lines (2 blades visible)
            blade_pen = QPen(QColor(205, 214, 244, min(alpha + 30, 255)), 2)
            p.setPen(blade_pen)
            for a in (angle, angle + 180):
                rad = math.radians(a)
                bx = math.cos(rad) * ROTOR_R
                bz = math.sin(rad) * ROTOR_R
                p.drawLine(mp, _pt(mx + bx, my, mz + bz,
                                   cx, cy, self._cam_yaw))

    # ------------------------------------------------------------------
    # HUD overlay
    # ------------------------------------------------------------------

    def _draw_hud(self, p: QPainter, w: int, h: int) -> None:
        s = self._state

        # ── Mode badge (top centre) ──────────────────────────────────
        mode_str  = s.mode.value
        mode_col  = QColor(_MODE_COLOURS.get(mode_str, "#585B70"))
        badge_txt = f"  {mode_str}  "
        f_badge = QFont("Segoe UI", 11, QFont.Weight.Bold)
        p.setFont(f_badge)
        fm = p.fontMetrics()
        bw = fm.horizontalAdvance(badge_txt) + 10
        bh = fm.height() + 6
        bx = (w - bw) // 2
        by = 12

        p.setBrush(QBrush(QColor(mode_col.red(), mode_col.green(),
                                  mode_col.blue(), 200)))
        p.setPen(Qt.PenStyle.NoPen)
        p.drawRoundedRect(bx, by, bw, bh, 5, 5)
        p.setPen(QPen(QColor("#11111B")))
        p.drawText(bx, by, bw, bh, Qt.AlignmentFlag.AlignCenter, badge_txt)

        # ── Flight command (below badge) ─────────────────────────────
        f_cmd = QFont("Segoe UI", 10)
        p.setFont(f_cmd)
        p.setPen(QPen(QColor("#CDD6F4")))
        p.drawText(0, by + bh + 4, w, 22,
                   Qt.AlignmentFlag.AlignHCenter, s.flight_command)

        # ── Telemetry strip (top-left) ───────────────────────────────
        self._draw_telem_strip(p, s)

        # ── Compass rose (top-right) ─────────────────────────────────
        self._draw_compass(p, w, s)

        # ── Attitude indicator (bottom-left) ─────────────────────────
        self._draw_attitude(p, h, s)

        # ── Altitude / speed bars (bottom-right) ─────────────────────
        self._draw_bars(p, w, h, s)

    def _draw_telem_strip(self, p: QPainter, s: DroneState) -> None:
        lines = [
            ("ALT",  f"{s.altitude:6.1f} m"),
            ("H-SPD", f"{s.speed_h:5.1f} m/s"),
            ("V-SPD", f"{s.speed_v:+5.1f} m/s"),
            ("THR",  f"{s.throttle * 100:4.0f} %"),
            ("HDG",  f"{s.heading:5.1f}°"),
            ("DIST", f"{s.total_distance:6.1f} m"),
        ]
        p.setFont(QFont("Consolas", 10))
        x, y = 10, 14
        for label, value in lines:
            p.setPen(QPen(QColor("#585B70")))
            p.drawText(x, y, 52, 18, Qt.AlignmentFlag.AlignRight, label)
            p.setPen(QPen(QColor("#CDD6F4")))
            p.drawText(x + 56, y, 80, 18, Qt.AlignmentFlag.AlignLeft, value)
            y += 19

    def _draw_compass(self, p: QPainter, w: int, s: DroneState) -> None:
        cx, cy, r = w - 52, 52, 38
        # Background
        p.setBrush(QBrush(QColor(17, 17, 27, 180)))
        p.setPen(QPen(QColor("#45475A"), 1))
        p.drawEllipse(QPointF(cx, cy), r, r)
        # Cardinal labels
        p.setFont(QFont("Segoe UI", 7, QFont.Weight.Bold))
        for label, angle in (("N", 0), ("E", 90), ("S", 180), ("W", 270)):
            rad = math.radians(angle - s.heading)
            lx = cx + (r - 10) * math.sin(rad)
            ly = cy - (r - 10) * math.cos(rad)
            p.setPen(QPen(QColor("#A6E3A1") if label == "N"
                         else QColor("#A6ADC8")))
            p.drawText(int(lx) - 6, int(ly) - 6, 12, 12,
                       Qt.AlignmentFlag.AlignCenter, label)
        # Heading needle
        p.setPen(QPen(QColor("#F38BA8"), 2))
        p.setBrush(QBrush(QColor("#F38BA8")))
        tip = QPointF(cx, cy - r + 8)
        p.drawLine(QPointF(cx, cy), tip)
        p.drawEllipse(tip, 3, 3)
        # Centre dot
        p.setBrush(QBrush(QColor("#CDD6F4")))
        p.setPen(Qt.PenStyle.NoPen)
        p.drawEllipse(QPointF(cx, cy), 3, 3)

    def _draw_attitude(self, p: QPainter, h: int, s: DroneState) -> None:
        cx, cy, r = 52, h - 60, 38
        # Background split sky/ground
        p.setClipRect(int(cx - r), int(cy - r), r * 2, r * 2)
        # Sky
        p.setBrush(QBrush(QColor(13, 71, 161, 160)))
        p.setPen(Qt.PenStyle.NoPen)
        p.drawEllipse(QPointF(cx, cy), r, r)
        # Ground horizon shift by pitch
        pitch_off = s.pitch * (r / 45.0)
        roll_rad  = math.radians(-s.roll)
        # Rotated horizon line
        cos_r = math.cos(roll_rad)
        sin_r = math.sin(roll_rad)
        p.setBrush(QBrush(QColor(95, 56, 28, 160)))
        path = QPainterPath()
        hx1 = cx - r * cos_r - pitch_off * sin_r
        hy1 = cy - r * sin_r + pitch_off * cos_r  # type: ignore[assignment]
        hx2 = cx + r * cos_r + pitch_off * sin_r
        hy2 = cy + r * sin_r - pitch_off * cos_r  # type: ignore[assignment]
        path.moveTo(hx1, hy1)
        path.lineTo(hx2, hy2)
        path.lineTo(cx + r, cy + r)
        path.lineTo(cx - r, cy + r)
        path.closeSubpath()
        p.drawPath(path)
        p.setClipping(False)
        # Rim
        p.setBrush(Qt.BrushStyle.NoBrush)
        p.setPen(QPen(QColor("#45475A"), 2))
        p.drawEllipse(QPointF(cx, cy), r, r)
        # Centre cross
        p.setPen(QPen(QColor("#FAB387"), 2))
        p.drawLine(int(cx) - 10, int(cy), int(cx) - 4, int(cy))
        p.drawLine(int(cx) + 4,  int(cy), int(cx) + 10, int(cy))
        p.drawLine(int(cx), int(cy) - 4, int(cx), int(cy) + 4)

    def _draw_bars(self, p: QPainter, w: int, h: int,
                   s: DroneState) -> None:
        """Altitude and throttle vertical bars (bottom-right)."""
        bar_h = 100
        bar_w = 14
        by = h - bar_h - 14
        # Altitude bar (max 30 m)
        self._vert_bar(p, w - 38, by, bar_w, bar_h,
                       s.altitude / 30.0, QColor("#89B4FA"), "ALT")
        # Throttle bar
        self._vert_bar(p, w - 18, by, bar_w, bar_h,
                       s.throttle, QColor("#A6E3A1"), "THR")

    def _vert_bar(self, p: QPainter, x: int, y: int,
                  bw: int, bh: int, frac: float,
                  colour: QColor, label: str) -> None:
        frac = max(0.0, min(1.0, frac))
        # Background
        p.setBrush(QBrush(QColor(17, 17, 27, 180)))
        p.setPen(QPen(QColor("#45475A"), 1))
        p.drawRect(x, y, bw, bh)
        # Fill
        fill_h = int(bh * frac)
        if fill_h > 0:
            p.setBrush(QBrush(colour))
            p.setPen(Qt.PenStyle.NoPen)
            p.drawRect(x + 1, y + bh - fill_h, bw - 2, fill_h)
        # Label
        p.setPen(QPen(QColor("#A6ADC8")))
        p.setFont(QFont("Consolas", 7))
        p.drawText(x - 2, y + bh + 2, bw + 4, 12,
                   Qt.AlignmentFlag.AlignCenter, label)
