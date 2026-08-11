"""Live controller test panel — axes, buttons, hat, direction, coordinates.

Layout (three-column):
  LEFT   — Axes (bars + %) + Hat compass
  MIDDLE — Motion & Direction panel (joystick canvas, direction label,
           coordinates, angle, magnitude, twist, throttle, motion log)
  RIGHT  — Buttons grid + Statistics + Input Event Log
"""
from __future__ import annotations

import math
from collections import deque
from datetime import datetime

from PySide6.QtCore import Qt, QTimer, Slot
from PySide6.QtGui import QColor, QFont, QPainter, QPen, QBrush, QPolygonF
from PySide6.QtCore import QPointF
from PySide6.QtWidgets import (
    QFrame, QGridLayout, QGroupBox, QHBoxLayout, QLabel,
    QProgressBar, QSizePolicy, QSplitter,
    QTextEdit, QVBoxLayout, QWidget,
)

from app.core.controller_monitor import (
    AxisState, ButtonState, ControllerMonitorThread,
    HatState, InputState, MotionState,
)
from app.models.usb_device import USBDevice
from app.utils.logger import get_logger

logger = get_logger(__name__)

_STYLE = """
QWidget { background-color:#1E1E2E; color:#CDD6F4;
          font-family:"Segoe UI",Arial; font-size:12px; }
QGroupBox { border:1px solid #45475A; border-radius:6px;
            margin-top:8px; padding:6px; }
QGroupBox::title { color:#89B4FA; font-weight:bold;
                   subcontrol-origin:margin; left:10px; }
QProgressBar { background:#313244; border:1px solid #45475A;
               border-radius:3px; height:14px; text-align:right;
               color:#CDD6F4; font-size:11px; }
QProgressBar::chunk { background:#89B4FA; border-radius:2px; }
QProgressBar#axisBar[active="true"]::chunk { background:#A6E3A1; }
QTextEdit { background:#11111B; color:#A6E3A1; border:1px solid #313244;
            font-family:Consolas,monospace; font-size:11px; }
QLabel#axisLabel  { color:#A6ADC8; font-size:11px; }
QLabel#axisValue  { color:#CDD6F4; font-size:11px;
                    font-family:Consolas; min-width:90px; }
QLabel#dirLabel   { color:#A6E3A1; font-size:22px; font-weight:bold; }
QLabel#statusLabel{ font-size:13px; font-weight:bold; }
QLabel#coordLabel { color:#CDD6F4; font-family:Consolas; font-size:11px; }
QLabel#statKey    { color:#A6ADC8; }
QLabel#statVal    { color:#CDD6F4; font-weight:bold; }
QLabel#errorLabel { color:#EF5350; font-weight:bold; }
"""

_BTN_IDLE    = ("background:#313244; border:1px solid #45475A;"
                "border-radius:4px; color:#A6ADC8;")
_BTN_PRESSED = ("background:#A6E3A1; border:1px solid #4CAF50;"
                "border-radius:4px; color:#1E1E2E; font-weight:bold;")
_BTN_SIZE = 36

# Direction → arrow glyph
_DIR_ARROWS: dict[str, str] = {
    "Forward":       "↑",
    "Back":          "↓",
    "Left":          "←",
    "Right":         "→",
    "Forward-Left":  "↖",
    "Forward-Right": "↗",
    "Back-Left":     "↙",
    "Back-Right":    "↘",
    "Center":        "⊙",
}

# Motion status colours
_MOVING_STYLE  = "color:#A6E3A1; font-size:13px; font-weight:bold;"
_STOPPED_STYLE = "color:#585B70; font-size:13px; font-weight:bold;"


# ---------------------------------------------------------------------------
# Hat compass widget
# ---------------------------------------------------------------------------

class HatCompassWidget(QWidget):
    _DIRECTIONS = ["N", "NE", "E", "SE", "S", "SW", "W", "NW"]
    _ANGLES     = [270, 315, 0, 45, 90, 135, 180, 225]

    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self._active_index: int | None = None
        self.setFixedSize(130, 130)

    def set_direction(self, hat: HatState | None) -> None:
        self._active_index = None if (hat is None or hat.raw == 8) else hat.raw
        self.update()

    def paintEvent(self, _event) -> None:          # noqa: N802
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        cx, cy, r = self.width() // 2, self.height() // 2, 50
        p.setPen(QPen(QColor("#45475A"), 1))
        p.setBrush(QBrush(QColor("#181825")))
        p.drawEllipse(cx - r, cy - r, r * 2, r * 2)
        p.setBrush(QBrush(QColor("#45475A")))
        p.setPen(Qt.PenStyle.NoPen)
        p.drawEllipse(cx - 4, cy - 4, 8, 8)
        for i, (lbl, ang) in enumerate(zip(self._DIRECTIONS, self._ANGLES)):
            rad = math.radians(ang)
            lx = cx + int((r - 13) * math.cos(rad))
            ly = cy + int((r - 13) * math.sin(rad))
            active = self._active_index == i
            p.setBrush(QBrush(QColor("#A6E3A1") if active else QColor("#585B70")))
            p.setPen(Qt.PenStyle.NoPen)
            p.drawEllipse(lx - 9, ly - 9, 18, 18)
            p.setPen(QPen(QColor("#1E1E2E") if active else QColor("#CDD6F4")))
            f = p.font(); f.setPixelSize(8); f.setBold(active); p.setFont(f)
            p.drawText(lx - 9, ly - 9, 18, 18, Qt.AlignmentFlag.AlignCenter, lbl)
        p.end()


# ---------------------------------------------------------------------------
# Joystick canvas — draws the 2-D stick position + trail
# ---------------------------------------------------------------------------

class JoystickCanvas(QWidget):
    """Square canvas showing stick X/Y position as a dot on a grid.

    The dot position maps directly to x_coord / y_coord (−1…+1).
    A short trail of previous positions is drawn in fading green.
    Dead-zone circle is shown as a dashed ring.
    """

    _TRAIL_LEN = 40

    def __init__(self, dead_zone: float = 0.15, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self._dead_zone = dead_zone   # fraction 0-1
        self._x = 0.0
        self._y = 0.0
        self._trail: deque[tuple[float, float]] = deque(maxlen=self._TRAIL_LEN)
        self.setFixedSize(200, 200)

    def update_position(self, x: float, y: float) -> None:
        self._trail.append((self._x, self._y))
        self._x = x
        self._y = y
        self.update()

    def paintEvent(self, _event) -> None:          # noqa: N802
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        w, h = self.width(), self.height()
        cx, cy = w // 2, h // 2
        r = min(cx, cy) - 4

        # Background
        p.setBrush(QBrush(QColor("#181825")))
        p.setPen(QPen(QColor("#45475A"), 1))
        p.drawEllipse(cx - r, cy - r, r * 2, r * 2)

        # Grid cross-hairs
        p.setPen(QPen(QColor("#313244"), 1))
        p.drawLine(cx - r, cy, cx + r, cy)
        p.drawLine(cx, cy - r, cx, cy + r)

        # Dead-zone ring
        dz_r = int(r * self._dead_zone)
        pen = QPen(QColor("#585B70"), 1, Qt.PenStyle.DashLine)
        p.setPen(pen)
        p.setBrush(Qt.BrushStyle.NoBrush)
        p.drawEllipse(cx - dz_r, cy - dz_r, dz_r * 2, dz_r * 2)

        # Trail (fading)
        trail = list(self._trail)
        n = len(trail)
        for i, (tx, ty) in enumerate(trail):
            alpha = int(180 * (i + 1) / max(n, 1))
            colour = QColor(166, 227, 161, alpha)   # #A6E3A1 with alpha
            px = cx + int(tx * r)
            py = cy - int(ty * r)   # Y inverted: positive = up
            p.setBrush(QBrush(colour))
            p.setPen(Qt.PenStyle.NoPen)
            p.drawEllipse(px - 2, py - 2, 4, 4)

        # Current dot
        dx = cx + int(self._x * r)
        dy = cy - int(self._y * r)
        p.setBrush(QBrush(QColor("#89B4FA")))
        p.setPen(QPen(QColor("#CDD6F4"), 1))
        p.drawEllipse(dx - 6, dy - 6, 12, 12)

        # Labels
        p.setPen(QPen(QColor("#585B70")))
        f = p.font(); f.setPixelSize(9); p.setFont(f)
        p.drawText(cx - 4, cy - r + 11, "FWD")
        p.drawText(cx - 4, cy + r - 2,  "BCK")
        p.drawText(cx - r + 2, cy + 4,  "L")
        p.drawText(cx + r - 9, cy + 4,  "R")
        p.end()


# ---------------------------------------------------------------------------
# Axis row widget
# ---------------------------------------------------------------------------

class AxisRowWidget(QWidget):
    def __init__(self, name: str, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        layout = QHBoxLayout(self)
        layout.setContentsMargins(0, 2, 0, 2)
        layout.setSpacing(6)

        lbl = QLabel(name)
        lbl.setObjectName("axisLabel")
        lbl.setFixedWidth(130)
        lbl.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
        layout.addWidget(lbl)

        self._bar = QProgressBar()
        self._bar.setObjectName("axisBar")
        self._bar.setRange(0, 1000)
        self._bar.setValue(500)
        self._bar.setFixedHeight(14)
        self._bar.setTextVisible(False)
        layout.addWidget(self._bar)

        self._pct_lbl = QLabel("50.0 %")
        self._pct_lbl.setObjectName("axisValue")
        self._pct_lbl.setFixedWidth(58)
        layout.addWidget(self._pct_lbl)

        self._deg_lbl = QLabel("180.0°")
        self._deg_lbl.setObjectName("axisValue")
        self._deg_lbl.setFixedWidth(66)
        layout.addWidget(self._deg_lbl)

        self._raw_lbl = QLabel("raw:0")
        self._raw_lbl.setObjectName("axisLabel")
        self._raw_lbl.setFixedWidth(80)
        layout.addWidget(self._raw_lbl)

    def update_axis(self, state: AxisState) -> None:
        self._bar.setValue(int(state.percent * 10))
        active = abs(state.percent - 50.0) > 5.0
        self._bar.setProperty("active", "true" if active else "false")
        self._bar.style().unpolish(self._bar)
        self._bar.style().polish(self._bar)
        self._pct_lbl.setText(f"{state.percent:.1f} %")
        self._deg_lbl.setText(f"{state.degrees:.1f}°")
        self._raw_lbl.setText(f"raw:{state.raw}")


# ---------------------------------------------------------------------------
# Main controller test widget
# ---------------------------------------------------------------------------

class ControllerTestWidget(QWidget):

    def __init__(self, device: USBDevice, scan_axes: list[str],
                 button_count: int, has_hat: bool,
                 axis_bit_sizes: list[int] | None = None,
                 field_map: str = "",
                 parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self._device = device
        self._scan_axes = scan_axes
        self._button_count = button_count
        self._has_hat = has_hat
        self._axis_bit_sizes = axis_bit_sizes or []
        self._field_map = field_map
        self._monitor: ControllerMonitorThread | None = None

        self._prev_buttons: dict[int, bool] = {}
        self._press_counts: dict[int, int] = {}
        self._total_events = 0
        self._event_count_window = 0
        self._btn_labels: dict[int, QLabel] = {}
        self._axis_rows: dict[str, AxisRowWidget] = {}

        # Direction change tracking
        self._prev_direction = "Center"
        self._prev_motion_status = "Stopped"

        self.setWindowTitle(
            f"Controller Input Monitor — {device.name or device.device_id}")
        self.resize(1100, 680)
        self.setMinimumSize(900, 520)
        self.setStyleSheet(_STYLE)
        self.setAttribute(Qt.WidgetAttribute.WA_DeleteOnClose, True)

        self._setup_ui()
        self._start_monitor()

    # ------------------------------------------------------------------
    # UI construction
    # ------------------------------------------------------------------

    def _setup_ui(self) -> None:
        root = QVBoxLayout(self)
        root.setContentsMargins(8, 8, 8, 8)
        root.setSpacing(6)

        # ── Header ──────────────────────────────────────────────────
        hdr = QHBoxLayout()
        title = QLabel(f"🎮  {self._device.name or self._device.device_id}")
        title.setStyleSheet("font-size:14px; font-weight:bold; color:#89B4FA;")
        hdr.addWidget(title)
        hdr.addStretch()
        self._status_lbl = QLabel("● Connecting…")
        self._status_lbl.setStyleSheet("color:#FFA726; font-weight:bold;")
        hdr.addWidget(self._status_lbl)
        root.addLayout(hdr)

        # ── Three-column splitter ────────────────────────────────────
        splitter = QSplitter(Qt.Orientation.Horizontal)

        splitter.addWidget(self._build_left_panel())
        splitter.addWidget(self._build_middle_panel())
        splitter.addWidget(self._build_right_panel())

        splitter.setStretchFactor(0, 3)
        splitter.setStretchFactor(1, 4)
        splitter.setStretchFactor(2, 3)
        root.addWidget(splitter)

        # ── Error label ──────────────────────────────────────────────
        self._error_lbl = QLabel()
        self._error_lbl.setObjectName("errorLabel")
        self._error_lbl.setWordWrap(True)
        self._error_lbl.hide()
        root.addWidget(self._error_lbl)

        # Rate timer
        self._rate_timer = QTimer(self)
        self._rate_timer.setInterval(1000)
        self._rate_timer.timeout.connect(self._update_rate)
        self._rate_timer.start()

    def _build_left_panel(self) -> QWidget:
        """Axes bars + Hat compass."""
        w = QWidget()
        layout = QVBoxLayout(w)
        layout.setContentsMargins(0, 0, 4, 0)
        layout.setSpacing(6)

        axes_box = QGroupBox("Axes")
        axes_vbox = QVBoxLayout(axes_box)
        axes_vbox.setSpacing(2)
        if self._scan_axes:
            for name in self._scan_axes:
                row = AxisRowWidget(name)
                self._axis_rows[name] = row
                axes_vbox.addWidget(row)
        else:
            axes_vbox.addWidget(QLabel("No axis data from scan."))
        layout.addWidget(axes_box)

        if self._has_hat:
            hat_box = QGroupBox("Hat Switch")
            hat_h = QHBoxLayout(hat_box)
            self._hat_widget = HatCompassWidget()
            self._hat_dir_lbl = QLabel("Centered")
            self._hat_dir_lbl.setStyleSheet(
                "font-size:13px; font-weight:bold; color:#CDD6F4;")
            self._hat_deg_lbl = QLabel("—")
            self._hat_deg_lbl.setObjectName("coordLabel")
            hat_h.addWidget(self._hat_widget)
            col = QVBoxLayout()
            col.addWidget(QLabel("Direction:"))
            col.addWidget(self._hat_dir_lbl)
            col.addWidget(QLabel("Angle:"))
            col.addWidget(self._hat_deg_lbl)
            col.addStretch()
            hat_h.addLayout(col)
            layout.addWidget(hat_box)
        else:
            self._hat_widget = None
            self._hat_dir_lbl = None
            self._hat_deg_lbl = None

        layout.addStretch()
        return w

    def _build_middle_panel(self) -> QWidget:
        """Motion & Direction: joystick canvas, direction label, coordinates,
        angle, magnitude, twist, throttle, motion log."""
        w = QWidget()
        layout = QVBoxLayout(w)
        layout.setContentsMargins(4, 0, 4, 0)
        layout.setSpacing(6)

        # ── Direction command ────────────────────────────────────────
        dir_box = QGroupBox("Direction Command")
        dir_vbox = QVBoxLayout(dir_box)

        self._dir_arrow_lbl = QLabel("⊙")
        self._dir_arrow_lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self._dir_arrow_lbl.setStyleSheet(
            "font-size:36px; color:#A6E3A1; font-weight:bold;")
        dir_vbox.addWidget(self._dir_arrow_lbl)

        self._dir_text_lbl = QLabel("Center")
        self._dir_text_lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self._dir_text_lbl.setObjectName("dirLabel")
        dir_vbox.addWidget(self._dir_text_lbl)

        self._motion_status_lbl = QLabel("Stopped")
        self._motion_status_lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self._motion_status_lbl.setStyleSheet(_STOPPED_STYLE)
        dir_vbox.addWidget(self._motion_status_lbl)

        layout.addWidget(dir_box)

        # ── Joystick canvas ──────────────────────────────────────────
        canvas_box = QGroupBox("Stick Position")
        canvas_h = QHBoxLayout(canvas_box)
        canvas_h.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self._joystick_canvas = JoystickCanvas(dead_zone=0.15)
        canvas_h.addWidget(self._joystick_canvas)
        layout.addWidget(canvas_box)

        # ── Coordinates & measurements ───────────────────────────────
        coord_box = QGroupBox("Measurements")
        cg = QGridLayout(coord_box)
        cg.setSpacing(4)

        def _coord_row(label: str, row: int) -> QLabel:
            k = QLabel(label)
            k.setObjectName("statKey")
            v = QLabel("—")
            v.setObjectName("coordLabel")
            cg.addWidget(k, row, 0)
            cg.addWidget(v, row, 1)
            return v

        self._cx_lbl  = _coord_row("X Coordinate:", 0)
        self._cy_lbl  = _coord_row("Y Coordinate:", 1)
        self._ang_lbl = _coord_row("Angle (from fwd):", 2)
        self._mag_lbl = _coord_row("Magnitude:", 3)
        self._tw_lbl  = _coord_row("Twist / Rudder:", 4)
        self._th_lbl  = _coord_row("Throttle / Slider:", 5)
        layout.addWidget(coord_box)

        # ── Motion event log ─────────────────────────────────────────
        mlog_box = QGroupBox("Motion Log")
        mlog_vbox = QVBoxLayout(mlog_box)
        self._motion_log = QTextEdit()
        self._motion_log.setReadOnly(True)
        self._motion_log.setMinimumHeight(100)
        mlog_vbox.addWidget(self._motion_log)
        layout.addWidget(mlog_box)

        return w

    def _build_right_panel(self) -> QWidget:
        """Buttons grid + statistics + input event log."""
        w = QWidget()
        layout = QVBoxLayout(w)
        layout.setContentsMargins(4, 0, 0, 0)
        layout.setSpacing(6)

        # Buttons
        btn_box = QGroupBox(f"Buttons  ({self._button_count})")
        btn_grid = QGridLayout(btn_box)
        btn_grid.setSpacing(5)
        # Scale columns: up to 8 wide for many buttons, 4 wide for few
        cols = 8 if self._button_count > 8 else max(4, self._button_count)
        for i in range(self._button_count):
            lbl = QLabel(str(i + 1))
            lbl.setFixedSize(_BTN_SIZE, _BTN_SIZE)
            lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
            lbl.setStyleSheet(_BTN_IDLE)
            lbl.setToolTip(f"Button {i + 1}")
            self._btn_labels[i + 1] = lbl
            self._prev_buttons[i + 1] = False
            btn_grid.addWidget(lbl, i // cols, i % cols)
        layout.addWidget(btn_box)

        # Statistics
        stats_box = QGroupBox("Statistics")
        sg = QGridLayout(stats_box)
        sg.setSpacing(4)
        self._total_events_lbl = self._stat_pair(sg, 0, "Total Events:")
        self._rate_lbl         = self._stat_pair(sg, 1, "Event Rate:")
        self._direction_lbl    = self._stat_pair(sg, 2, "Last Direction:")
        layout.addWidget(stats_box)

        # Input event log
        log_box = QGroupBox("Input Event Log")
        log_vbox = QVBoxLayout(log_box)
        self._event_log = QTextEdit()
        self._event_log.setReadOnly(True)
        self._event_log.setMinimumHeight(140)
        log_vbox.addWidget(self._event_log)
        layout.addWidget(log_box)

        layout.addStretch()
        return w

    def _stat_pair(self, grid: QGridLayout, row: int, label: str) -> QLabel:
        k = QLabel(label); k.setObjectName("statKey")
        v = QLabel("—");   v.setObjectName("statVal")
        grid.addWidget(k, row, 0)
        grid.addWidget(v, row, 1)
        return v

    # ------------------------------------------------------------------
    # Monitor start / stop
    # ------------------------------------------------------------------

    def _start_monitor(self) -> None:
        if not self._device.vendor_id or not self._device.product_id:
            self._show_error("VID/PID not available — cannot start live monitoring.")
            return
        try:
            vid = int(self._device.vendor_id, 16)
            pid = int(self._device.product_id, 16)
        except ValueError:
            self._show_error("Invalid VID/PID values.")
            return

        self._monitor = ControllerMonitorThread(
            vid=vid, pid=pid,
            axis_names=self._scan_axes,
            button_count=self._button_count,
            has_hat=self._has_hat,
            axis_bit_sizes=self._axis_bit_sizes or None,
            field_map=self._field_map,
            parent=self,
        )
        self._monitor.state_updated.connect(self._on_state)
        self._monitor.monitor_error.connect(self._on_monitor_error)
        self._monitor.start()

    def _stop_monitor(self) -> None:
        if self._monitor and self._monitor.isRunning():
            self._monitor.stop()
            self._monitor = None

    # ------------------------------------------------------------------
    # State handler
    # ------------------------------------------------------------------

    @Slot(object)
    def _on_state(self, state: InputState) -> None:
        self._status_lbl.setText("● Live")
        self._status_lbl.setStyleSheet("color:#A6E3A1; font-weight:bold;")

        # ── Axes ─────────────────────────────────────────────────────
        for axis in state.axes:
            row = self._axis_rows.get(axis.name)
            if row:
                row.update_axis(axis)

        # ── Hat ──────────────────────────────────────────────────────
        if state.hat and self._hat_widget:
            self._hat_widget.set_direction(state.hat)
            self._hat_dir_lbl.setText(state.hat.direction)
            self._hat_deg_lbl.setText(
                f"{state.hat.degrees:.0f}°" if state.hat.degrees is not None else "—")

        # ── Motion & Direction ────────────────────────────────────────
        m = state.motion
        self._joystick_canvas.update_position(m.x_coord, m.y_coord)

        arrow = _DIR_ARROWS.get(m.direction, "⊙")
        self._dir_arrow_lbl.setText(arrow)
        self._dir_text_lbl.setText(m.direction)

        if m.motion_status == "Moving":
            self._motion_status_lbl.setText("▶  Moving")
            self._motion_status_lbl.setStyleSheet(_MOVING_STYLE)
        else:
            self._motion_status_lbl.setText("■  Stopped")
            self._motion_status_lbl.setStyleSheet(_STOPPED_STYLE)

        self._cx_lbl.setText(f"{m.x_coord:+.3f}  ({m.x_percent:.1f}%)")
        self._cy_lbl.setText(f"{m.y_coord:+.3f}  ({m.y_percent:.1f}%)")
        self._ang_lbl.setText(f"{m.angle_deg:.1f}°")
        self._mag_lbl.setText(f"{m.magnitude:.3f}  ({m.magnitude * 100:.1f}%)")
        self._tw_lbl.setText( f"{m.twist_degrees:.1f}°  ({m.twist_percent:.1f}%)")
        self._th_lbl.setText( f"{m.throttle_percent:.1f}%")
        self._direction_lbl.setText(m.direction)

        # Log direction changes
        if m.direction != self._prev_direction:
            self._log_motion(
                f"→  Direction: {self._prev_direction}  ▶  {m.direction}"
                f"  |  angle {m.angle_deg:.1f}°  mag {m.magnitude:.2f}")
            self._prev_direction = m.direction
            self._total_events += 1
            self._event_count_window += 1

        if m.motion_status != self._prev_motion_status:
            status_str = "▶ Moving" if m.motion_status == "Moving" else "■ Stopped"
            self._log_motion(f"  {status_str}  (X={m.x_coord:+.2f}, Y={m.y_coord:+.2f})")
            self._prev_motion_status = m.motion_status

        # ── Buttons ──────────────────────────────────────────────────
        for btn in state.buttons:
            prev = self._prev_buttons.get(btn.index, False)
            lbl  = self._btn_labels.get(btn.index)
            if lbl:
                lbl.setStyleSheet(_BTN_PRESSED if btn.pressed else _BTN_IDLE)
            if btn.pressed and not prev:
                self._log_event(f"▼  Button {btn.index} pressed")
                self._press_counts[btn.index] = \
                    self._press_counts.get(btn.index, 0) + 1
                self._total_events += 1
                self._event_count_window += 1
            elif not btn.pressed and prev:
                self._log_event(f"▲  Button {btn.index} released")
                self._total_events += 1
                self._event_count_window += 1
            self._prev_buttons[btn.index] = btn.pressed

        self._total_events_lbl.setText(str(self._total_events))

    @Slot(str)
    def _on_monitor_error(self, message: str) -> None:
        self._show_error(message)
        self._status_lbl.setText("● Error")
        self._status_lbl.setStyleSheet("color:#EF5350; font-weight:bold;")

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _log_event(self, message: str) -> None:
        ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        self._event_log.append(f"{ts}  {message}")
        sb = self._event_log.verticalScrollBar()
        sb.setValue(sb.maximum())

    def _log_motion(self, message: str) -> None:
        ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        self._motion_log.append(f"{ts}  {message}")
        sb = self._motion_log.verticalScrollBar()
        sb.setValue(sb.maximum())

    def _show_error(self, message: str) -> None:
        self._error_lbl.setText(f"⚠  {message}")
        self._error_lbl.show()

    @Slot()
    def _update_rate(self) -> None:
        self._rate_lbl.setText(f"{self._event_count_window} events/s")
        self._event_count_window = 0

    # ------------------------------------------------------------------
    # Window lifecycle
    # ------------------------------------------------------------------

    def closeEvent(self, event) -> None:  # noqa: N802
        self._rate_timer.stop()
        self._stop_monitor()
        event.accept()
