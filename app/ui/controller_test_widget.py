"""Live controller test panel — smooth 60 Hz rendering edition.

Architecture fix
----------------
Previous version: signal handler (_on_state) ran ALL UI updates at 120 Hz.
    Problem: Qt stylesheet unpolish/polish + label setText + canvas update at
    120 Hz = ~600 layout/style operations per second → glitch, lag, dropped
    frames.

New version:
    - ControllerMonitorThread emits at 120 Hz (hardware rate, unchanged)
    - _on_state() does ONE thing: stores the latest InputState in self._latest
    - A QTimer at 60 Hz calls _render_frame() which updates the UI ONCE per
      frame from the latest snapshot
    - AxisRowWidget no longer calls unpolish/polish — uses stylesheet property
      replacement instead (single string op, no Qt layout pass)
    - JoystickCanvas only calls update() when x or y actually changed
"""
from __future__ import annotations

import math
from collections import deque
from datetime import datetime

from PySide6.QtCore import Qt, QTimer, Slot
from PySide6.QtGui import QColor, QFont, QPainter, QPen, QBrush, QPolygonF
from PySide6.QtCore import QPointF
from PySide6.QtWidgets import (
    QGridLayout, QGroupBox, QHBoxLayout, QLabel,
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
               border-radius:3px; height:14px; }
QTextEdit { background:#11111B; color:#A6E3A1; border:1px solid #313244;
            font-family:Consolas,monospace; font-size:11px; }
QLabel#axisLabel  { color:#A6ADC8; font-size:11px; }
QLabel#axisValue  { color:#CDD6F4; font-size:11px; font-family:Consolas; }
QLabel#dirLabel   { color:#A6E3A1; font-size:22px; font-weight:bold; }
QLabel#coordLabel { color:#CDD6F4; font-family:Consolas; font-size:11px; }
QLabel#statKey    { color:#A6ADC8; }
QLabel#statVal    { color:#CDD6F4; font-weight:bold; }
QLabel#errorLabel { color:#EF5350; font-weight:bold; }
"""

# Axis bar stylesheet templates — built once, swapped by string replace
_BAR_IDLE   = ("QProgressBar{background:#313244;border:1px solid #45475A;"
               "border-radius:3px;height:14px;}"
               "QProgressBar::chunk{background:#89B4FA;border-radius:2px;}")
_BAR_ACTIVE = ("QProgressBar{background:#313244;border:1px solid #45475A;"
               "border-radius:3px;height:14px;}"
               "QProgressBar::chunk{background:#A6E3A1;border-radius:2px;}")

_BTN_IDLE    = ("background:#313244;border:1px solid #45475A;"
                "border-radius:4px;color:#A6ADC8;")
_BTN_PRESSED = ("background:#A6E3A1;border:1px solid #4CAF50;"
                "border-radius:4px;color:#1E1E2E;font-weight:bold;")
_BTN_SIZE = 36

_DIR_ARROWS = {
    "Forward":"↑", "Back":"↓", "Left":"←", "Right":"→",
    "Forward-Left":"↖", "Forward-Right":"↗",
    "Back-Left":"↙",   "Back-Right":"↘",
    "Center":"⊙",
}
_MOVING_STYLE  = "color:#A6E3A1; font-size:13px; font-weight:bold;"
_STOPPED_STYLE = "color:#585B70; font-size:13px; font-weight:bold;"


# ---------------------------------------------------------------------------
# Axis row widget  (no unpolish/polish — just swap stylesheet strings)
# ---------------------------------------------------------------------------

class AxisRowWidget(QWidget):
    def __init__(self, name: str, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        h = QHBoxLayout(self)
        h.setContentsMargins(0, 2, 0, 2)
        h.setSpacing(6)

        lbl = QLabel(name)
        lbl.setObjectName("axisLabel")
        lbl.setFixedWidth(130)
        lbl.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
        h.addWidget(lbl)

        self._bar = QProgressBar()
        self._bar.setRange(0, 1000)
        self._bar.setValue(500)
        self._bar.setFixedHeight(14)
        self._bar.setTextVisible(False)
        self._bar.setStyleSheet(_BAR_IDLE)
        h.addWidget(self._bar)

        self._pct = QLabel("50.0 %")
        self._pct.setObjectName("axisValue")
        self._pct.setFixedWidth(58)
        h.addWidget(self._pct)

        self._deg = QLabel("180.0°")
        self._deg.setObjectName("axisValue")
        self._deg.setFixedWidth(66)
        h.addWidget(self._deg)

        self._raw = QLabel("raw:0")
        self._raw.setObjectName("axisLabel")
        self._raw.setFixedWidth(80)
        h.addWidget(self._raw)

        # Cache last values to skip redundant setText calls
        self._last_pct = -1.0
        self._last_raw = -1
        self._last_active = False

    def update_axis(self, state: AxisState) -> None:
        # Update bar value (fast int operation)
        val = int(state.percent * 10)
        self._bar.setValue(val)

        # Swap stylesheet only when active state changes — avoids Qt style
        # engine overhead on every frame
        active = abs(state.percent - 50.0) > 5.0
        if active != self._last_active:
            self._bar.setStyleSheet(_BAR_ACTIVE if active else _BAR_IDLE)
            self._last_active = active

        # Update text labels only when value actually changed
        if state.raw != self._last_raw:
            self._pct.setText(f"{state.percent:.1f} %")
            self._deg.setText(f"{state.degrees:.1f}°")
            self._raw.setText(f"raw:{state.raw}")
            self._last_raw = state.raw


# ---------------------------------------------------------------------------
# Hat compass widget
# ---------------------------------------------------------------------------

class HatCompassWidget(QWidget):
    _DIRS   = ["N","NE","E","SE","S","SW","W","NW"]
    _ANGLES = [270,315,0,45,90,135,180,225]

    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self._active: int | None = None
        self.setFixedSize(130, 130)

    def set_direction(self, hat: HatState | None) -> None:
        new = None if (hat is None or hat.raw == 8) else hat.raw
        if new != self._active:
            self._active = new
            self.update()

    def paintEvent(self, _ev) -> None:           # noqa: N802
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        cx, cy, r = self.width()//2, self.height()//2, 50
        p.setPen(QPen(QColor("#45475A"), 1))
        p.setBrush(QBrush(QColor("#181825")))
        p.drawEllipse(cx-r, cy-r, r*2, r*2)
        p.setBrush(QBrush(QColor("#45475A")))
        p.setPen(Qt.PenStyle.NoPen)
        p.drawEllipse(cx-4, cy-4, 8, 8)
        for i, (lbl, ang) in enumerate(zip(self._DIRS, self._ANGLES)):
            rad = math.radians(ang)
            lx = cx + int((r-13)*math.cos(rad))
            ly = cy + int((r-13)*math.sin(rad))
            active = self._active == i
            p.setBrush(QBrush(QColor("#A6E3A1") if active else QColor("#585B70")))
            p.setPen(Qt.PenStyle.NoPen)
            p.drawEllipse(lx-9, ly-9, 18, 18)
            p.setPen(QPen(QColor("#1E1E2E") if active else QColor("#CDD6F4")))
            f = p.font(); f.setPixelSize(8); f.setBold(active); p.setFont(f)
            p.drawText(lx-9, ly-9, 18, 18, Qt.AlignmentFlag.AlignCenter, lbl)
        p.end()


# ---------------------------------------------------------------------------
# Joystick canvas  (only repaints when position actually changes)
# ---------------------------------------------------------------------------

class JoystickCanvas(QWidget):
    _TRAIL = 40

    def __init__(self, dead_zone: float = 0.12,
                 parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self._dz    = dead_zone
        self._x     = 0.0
        self._y     = 0.0
        self._trail: deque[tuple[float,float]] = deque(maxlen=self._TRAIL)
        self.setFixedSize(200, 200)

    def update_position(self, x: float, y: float) -> None:
        if abs(x - self._x) < 0.002 and abs(y - self._y) < 0.002:
            return   # skip repaint if nothing moved
        self._trail.append((self._x, self._y))
        self._x, self._y = x, y
        self.update()

    def paintEvent(self, _ev) -> None:           # noqa: N802
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        w, h = self.width(), self.height()
        cx, cy = w//2, h//2
        r = min(cx, cy) - 4

        p.setBrush(QBrush(QColor("#181825")))
        p.setPen(QPen(QColor("#45475A"), 1))
        p.drawEllipse(cx-r, cy-r, r*2, r*2)

        p.setPen(QPen(QColor("#313244"), 1))
        p.drawLine(cx-r, cy, cx+r, cy)
        p.drawLine(cx, cy-r, cx, cy+r)

        dz_r = int(r * self._dz)
        p.setPen(QPen(QColor("#585B70"), 1, Qt.PenStyle.DashLine))
        p.setBrush(Qt.BrushStyle.NoBrush)
        p.drawEllipse(cx-dz_r, cy-dz_r, dz_r*2, dz_r*2)

        trail = list(self._trail)
        n = len(trail)
        for i, (tx, ty) in enumerate(trail):
            a = int(180 * (i+1) / max(n, 1))
            p.setBrush(QBrush(QColor(166, 227, 161, a)))
            p.setPen(Qt.PenStyle.NoPen)
            p.drawEllipse(cx+int(tx*r)-2, cy-int(ty*r)-2, 4, 4)

        p.setBrush(QBrush(QColor("#89B4FA")))
        p.setPen(QPen(QColor("#CDD6F4"), 1))
        p.drawEllipse(cx+int(self._x*r)-6, cy-int(self._y*r)-6, 12, 12)

        p.setPen(QPen(QColor("#585B70")))
        f = p.font(); f.setPixelSize(9); p.setFont(f)
        p.drawText(cx-4, cy-r+11, "FWD")
        p.drawText(cx-4, cy+r-2,  "BCK")
        p.drawText(cx-r+2, cy+4,  "L")
        p.drawText(cx+r-9, cy+4,  "R")
        p.end()


# ---------------------------------------------------------------------------
# Main widget
# ---------------------------------------------------------------------------

class ControllerTestWidget(QWidget):
    """Live controller test panel.

    Signal flow:
        ControllerMonitorThread  →  _on_state()  →  stores self._latest
        QTimer (60 Hz)           →  _render_frame()  →  updates all UI
    """

    _RENDER_HZ = 60

    def __init__(self, device: USBDevice, scan_axes: list[str],
                 button_count: int, has_hat: bool,
                 axis_bit_sizes: list[int] | None = None,
                 field_map: str = "",
                 parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self._device        = device
        self._scan_axes     = scan_axes
        self._button_count  = button_count
        self._has_hat       = has_hat
        self._axis_bit_sizes = axis_bit_sizes or []
        self._field_map     = field_map
        self._monitor: ControllerMonitorThread | None = None

        # Latest state snapshot — written by signal, read by timer
        self._latest: InputState | None = None

        self._prev_btns: dict[int, bool] = {}
        self._press_counts: dict[int, int] = {}
        self._total_events = 0
        self._events_this_sec = 0
        self._btn_labels: dict[int, QLabel] = {}
        self._axis_rows: dict[str, AxisRowWidget] = {}
        self._prev_direction = "Center"
        self._prev_status    = "Stopped"

        self.setWindowTitle(
            f"Controller Input Monitor — {device.name or device.device_id}")
        self.resize(1100, 680)
        self.setMinimumSize(900, 520)
        self.setStyleSheet(_STYLE)
        self.setAttribute(Qt.WidgetAttribute.WA_DeleteOnClose, True)

        self._build_ui()
        self._start_monitor()
        self._start_render_timer()

    # ------------------------------------------------------------------
    # UI construction
    # ------------------------------------------------------------------

    def _build_ui(self) -> None:
        root = QVBoxLayout(self)
        root.setContentsMargins(8, 8, 8, 8)
        root.setSpacing(6)

        hdr = QHBoxLayout()
        title = QLabel(f"🎮  {self._device.name or self._device.device_id}")
        title.setStyleSheet("font-size:14px; font-weight:bold; color:#89B4FA;")
        hdr.addWidget(title)
        hdr.addStretch()
        self._status_lbl = QLabel("● Connecting…")
        self._status_lbl.setStyleSheet("color:#FFA726; font-weight:bold;")
        hdr.addWidget(self._status_lbl)
        root.addLayout(hdr)

        sp = QSplitter(Qt.Orientation.Horizontal)
        sp.addWidget(self._build_left())
        sp.addWidget(self._build_middle())
        sp.addWidget(self._build_right())
        sp.setStretchFactor(0, 3)
        sp.setStretchFactor(1, 4)
        sp.setStretchFactor(2, 3)
        root.addWidget(sp)

        self._error_lbl = QLabel()
        self._error_lbl.setObjectName("errorLabel")
        self._error_lbl.setWordWrap(True)
        self._error_lbl.hide()
        root.addWidget(self._error_lbl)

    def _build_left(self) -> QWidget:
        w = QWidget(); lay = QVBoxLayout(w)
        lay.setContentsMargins(0,0,4,0); lay.setSpacing(6)

        axes_box = QGroupBox("Axes")
        av = QVBoxLayout(axes_box); av.setSpacing(2)
        for name in (self._scan_axes or ["No axis data from scan."]):
            if self._scan_axes:
                row = AxisRowWidget(name)
                self._axis_rows[name] = row
                av.addWidget(row)
            else:
                av.addWidget(QLabel(name))
        lay.addWidget(axes_box)

        if self._has_hat:
            hat_box = QGroupBox("Hat Switch")
            hh = QHBoxLayout(hat_box)
            self._hat_widget = HatCompassWidget()
            self._hat_dir_lbl = QLabel("Centered")
            self._hat_dir_lbl.setStyleSheet("font-size:13px;font-weight:bold;color:#CDD6F4;")
            self._hat_deg_lbl = QLabel("—")
            self._hat_deg_lbl.setObjectName("coordLabel")
            hh.addWidget(self._hat_widget)
            col = QVBoxLayout()
            col.addWidget(QLabel("Direction:"))
            col.addWidget(self._hat_dir_lbl)
            col.addWidget(QLabel("Angle:"))
            col.addWidget(self._hat_deg_lbl)
            col.addStretch()
            hh.addLayout(col)
            lay.addWidget(hat_box)
        else:
            self._hat_widget = self._hat_dir_lbl = self._hat_deg_lbl = None

        lay.addStretch()
        return w

    def _build_middle(self) -> QWidget:
        w = QWidget(); lay = QVBoxLayout(w)
        lay.setContentsMargins(4,0,4,0); lay.setSpacing(6)

        # Direction
        db = QGroupBox("Direction Command")
        dv = QVBoxLayout(db)
        self._dir_arrow = QLabel("⊙")
        self._dir_arrow.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self._dir_arrow.setStyleSheet("font-size:36px;color:#A6E3A1;font-weight:bold;")
        dv.addWidget(self._dir_arrow)
        self._dir_text = QLabel("Center")
        self._dir_text.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self._dir_text.setObjectName("dirLabel")
        dv.addWidget(self._dir_text)
        self._motion_lbl = QLabel("Stopped")
        self._motion_lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self._motion_lbl.setStyleSheet(_STOPPED_STYLE)
        dv.addWidget(self._motion_lbl)
        lay.addWidget(db)

        # Canvas
        cb = QGroupBox("Stick Position")
        ch = QHBoxLayout(cb); ch.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self._canvas = JoystickCanvas(dead_zone=0.12)
        ch.addWidget(self._canvas)
        lay.addWidget(cb)

        # Measurements
        mb = QGroupBox("Measurements")
        mg = QGridLayout(mb); mg.setSpacing(4)
        def cr(lbl, row):
            k = QLabel(lbl); k.setObjectName("statKey")
            v = QLabel("—"); v.setObjectName("coordLabel")
            mg.addWidget(k, row, 0); mg.addWidget(v, row, 1)
            return v
        self._cx  = cr("X Coordinate:", 0)
        self._cy  = cr("Y Coordinate:", 1)
        self._ang = cr("Angle:", 2)
        self._mag = cr("Magnitude:", 3)
        self._tw  = cr("Twist / Rudder:", 4)
        self._th  = cr("Throttle:", 5)
        lay.addWidget(mb)

        mlog = QGroupBox("Motion Log")
        mv = QVBoxLayout(mlog)
        self._motion_log = QTextEdit()
        self._motion_log.setReadOnly(True)
        self._motion_log.setMinimumHeight(100)
        mv.addWidget(self._motion_log)
        lay.addWidget(mlog)
        return w

    def _build_right(self) -> QWidget:
        w = QWidget(); lay = QVBoxLayout(w)
        lay.setContentsMargins(4,0,0,0); lay.setSpacing(6)

        cols = 8 if self._button_count > 8 else max(4, self._button_count)
        bb = QGroupBox(f"Buttons  ({self._button_count})")
        bg = QGridLayout(bb); bg.setSpacing(5)
        for i in range(self._button_count):
            lbl = QLabel(str(i+1))
            lbl.setFixedSize(_BTN_SIZE, _BTN_SIZE)
            lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
            lbl.setStyleSheet(_BTN_IDLE)
            lbl.setToolTip(f"Button {i+1}")
            self._btn_labels[i+1] = lbl
            self._prev_btns[i+1] = False
            bg.addWidget(lbl, i//cols, i%cols)
        lay.addWidget(bb)

        sb = QGroupBox("Statistics")
        sg = QGridLayout(sb); sg.setSpacing(4)
        self._total_lbl = self._sp(sg, 0, "Total Events:")
        self._rate_lbl  = self._sp(sg, 1, "Event Rate:")
        self._last_dir  = self._sp(sg, 2, "Last Direction:")
        lay.addWidget(sb)

        lb = QGroupBox("Input Event Log")
        lv = QVBoxLayout(lb)
        self._event_log = QTextEdit()
        self._event_log.setReadOnly(True)
        self._event_log.setMinimumHeight(140)
        lv.addWidget(self._event_log)
        lay.addWidget(lb)
        lay.addStretch()
        return w

    def _sp(self, g, row, lbl):
        k = QLabel(lbl); k.setObjectName("statKey")
        v = QLabel("—"); v.setObjectName("statVal")
        g.addWidget(k, row, 0); g.addWidget(v, row, 1)
        return v

    # ------------------------------------------------------------------
    # Monitor
    # ------------------------------------------------------------------

    def _start_monitor(self) -> None:
        if not self._device.vendor_id or not self._device.product_id:
            self._show_error("VID/PID not available.")
            return
        try:
            vid = int(self._device.vendor_id, 16)
            pid = int(self._device.product_id, 16)
        except ValueError:
            self._show_error("Invalid VID/PID.")
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
        self._monitor.monitor_error.connect(self._on_error)
        self._monitor.start()

    # ------------------------------------------------------------------
    # 60 Hz render timer
    # ------------------------------------------------------------------

    def _start_render_timer(self) -> None:
        """Drive UI updates at 60 Hz, decoupled from the 120 Hz HID thread."""
        self._render_timer = QTimer(self)
        self._render_timer.setTimerType(Qt.TimerType.PreciseTimer)
        self._render_timer.setInterval(1000 // self._RENDER_HZ)  # 16 ms
        self._render_timer.timeout.connect(self._render_frame)
        self._render_timer.start()

        # Rate counter (1 Hz)
        self._rate_timer = QTimer(self)
        self._rate_timer.setInterval(1000)
        self._rate_timer.timeout.connect(self._update_rate)
        self._rate_timer.start()

    # ------------------------------------------------------------------
    # Signal: just store the latest state — NO UI work here
    # ------------------------------------------------------------------

    @Slot(object)
    def _on_state(self, state: InputState) -> None:
        self._latest = state

    @Slot(str)
    def _on_error(self, msg: str) -> None:
        self._show_error(msg)
        self._status_lbl.setText("● Error")
        self._status_lbl.setStyleSheet("color:#EF5350; font-weight:bold;")

    # ------------------------------------------------------------------
    # Render frame (60 Hz) — all UI updates happen here
    # ------------------------------------------------------------------

    def _render_frame(self) -> None:
        state = self._latest
        if state is None:
            return

        self._status_lbl.setText("● Live")
        self._status_lbl.setStyleSheet("color:#A6E3A1; font-weight:bold;")

        # Axes
        for ax in state.axes:
            row = self._axis_rows.get(ax.name)
            if row:
                row.update_axis(ax)

        # Hat
        if state.hat and self._hat_widget:
            self._hat_widget.set_direction(state.hat)
            if self._hat_dir_lbl:
                self._hat_dir_lbl.setText(state.hat.direction)
            if self._hat_deg_lbl:
                self._hat_deg_lbl.setText(
                    f"{state.hat.degrees:.0f}°"
                    if state.hat.degrees is not None else "—")

        # Motion
        m = state.motion
        self._canvas.update_position(m.x_coord, m.y_coord)
        self._dir_arrow.setText(_DIR_ARROWS.get(m.direction, "⊙"))
        self._dir_text.setText(m.direction)
        self._motion_lbl.setText(
            "▶  Moving" if m.motion_status == "Moving" else "■  Stopped")
        self._motion_lbl.setStyleSheet(
            _MOVING_STYLE if m.motion_status == "Moving" else _STOPPED_STYLE)

        self._cx.setText(f"{m.x_coord:+.3f}  ({m.x_percent:.1f}%)")
        self._cy.setText(f"{m.y_coord:+.3f}  ({m.y_percent:.1f}%)")
        self._ang.setText(f"{m.angle_deg:.1f}°")
        self._mag.setText(f"{m.magnitude:.3f}  ({m.magnitude*100:.1f}%)")
        self._tw.setText(f"{m.twist_degrees:.1f}°  ({m.twist_percent:.1f}%)")
        self._th.setText(f"{m.throttle_percent:.1f}%")
        self._last_dir.setText(m.direction)

        # Log direction / status changes
        if m.direction != self._prev_direction:
            self._log_motion(
                f"→ {self._prev_direction} ▶ {m.direction}"
                f"  angle={m.angle_deg:.1f}° mag={m.magnitude:.2f}")
            self._prev_direction = m.direction
            self._total_events += 1
            self._events_this_sec += 1

        if m.motion_status != self._prev_status:
            self._log_motion(
                f"{'▶ Moving' if m.motion_status=='Moving' else '■ Stopped'}"
                f"  X={m.x_coord:+.2f} Y={m.y_coord:+.2f}")
            self._prev_status = m.motion_status

        # Buttons
        for btn in state.buttons:
            prev = self._prev_btns.get(btn.index, False)
            lbl  = self._btn_labels.get(btn.index)
            if lbl:
                lbl.setStyleSheet(_BTN_PRESSED if btn.pressed else _BTN_IDLE)
            if btn.pressed and not prev:
                self._log_event(f"▼ Button {btn.index} pressed")
                self._press_counts[btn.index] = \
                    self._press_counts.get(btn.index, 0) + 1
                self._total_events += 1
                self._events_this_sec += 1
            elif not btn.pressed and prev:
                self._log_event(f"▲ Button {btn.index} released")
                self._total_events += 1
                self._events_this_sec += 1
            self._prev_btns[btn.index] = btn.pressed

        self._total_lbl.setText(str(self._total_events))

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _log_event(self, msg: str) -> None:
        ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        self._event_log.append(f"{ts}  {msg}")
        self._event_log.verticalScrollBar().setValue(
            self._event_log.verticalScrollBar().maximum())

    def _log_motion(self, msg: str) -> None:
        ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        self._motion_log.append(f"{ts}  {msg}")
        self._motion_log.verticalScrollBar().setValue(
            self._motion_log.verticalScrollBar().maximum())

    def _show_error(self, msg: str) -> None:
        self._error_lbl.setText(f"⚠  {msg}")
        self._error_lbl.show()

    @Slot()
    def _update_rate(self) -> None:
        self._rate_lbl.setText(f"{self._events_this_sec} events/s")
        self._events_this_sec = 0

    # ------------------------------------------------------------------
    # Window lifecycle
    # ------------------------------------------------------------------

    def closeEvent(self, event) -> None:   # noqa: N802
        self._render_timer.stop()
        self._rate_timer.stop()
        if self._monitor and self._monitor.isRunning():
            self._monitor.stop()
        event.accept()
