"""Drone Simulator Window — drone_simulator.py

Wires together:
  ControllerMonitorThread  →  DronePhysics  →  Drone3DWidget
                                             →  Telemetry panel
                                             →  Flight log
                                             →  Control-mapping panel

Layout
------
  ┌──────────────────────────────────────────────────────┐
  │  Header: device name + status badge + camera buttons │
  ├────────────────────────────┬─────────────────────────┤
  │                            │  Control Mapping        │
  │   Drone3DWidget  (3D view) │  (axes + button legend) │
  │        (60 % width)        ├─────────────────────────┤
  │                            │  Telemetry grid         │
  ├────────────────────────────┴─────────────────────────┤
  │  Flight Log (scrolling timestamped event list)       │
  └──────────────────────────────────────────────────────┘

Physics is stepped at 60 Hz by a QTimer.  Controller inputs are read from
the latest InputState (set via a slot connected to ControllerMonitorThread).
"""
from __future__ import annotations

import time
from datetime import datetime

from PySide6.QtCore import Qt, QTimer, Slot
from PySide6.QtGui import QColor, QFont, QKeyEvent
from PySide6.QtWidgets import (
    QGridLayout, QGroupBox, QHBoxLayout, QLabel,
    QProgressBar, QPushButton, QScrollArea, QSizePolicy,
    QSplitter, QTextEdit, QVBoxLayout, QWidget,
)

from app.core.controller_monitor import (
    ControllerMonitorThread, InputState, MotionState,
)
from app.core.drone_physics import DroneInput, DronePhysics, FlightMode
from app.models.usb_device import USBDevice
from app.ui.drone_3d_view import Drone3DWidget
from app.utils.flight_logger import FlightLogger
from app.utils.logger import get_logger

logger = get_logger(__name__)

# ---------------------------------------------------------------------------
# Styles
# ---------------------------------------------------------------------------

_STYLE = """
QWidget { background:#1E1E2E; color:#CDD6F4;
          font-family:"Segoe UI",Arial; font-size:12px; }
QGroupBox { border:1px solid #45475A; border-radius:6px;
            margin-top:8px; padding:4px; }
QGroupBox::title { color:#89B4FA; font-weight:bold;
                   subcontrol-origin:margin; left:8px; }
QPushButton { background:#313244; color:#CDD6F4;
              border:1px solid #45475A; border-radius:5px;
              padding:5px 12px; }
QPushButton:hover  { background:#45475A; }
QPushButton:pressed{ background:#585B70; }
QPushButton#armBtn { background:#A6E3A1; color:#1E1E2E; font-weight:bold; }
QPushButton#armBtn:hover  { background:#94D1A0; }
QPushButton#landBtn{ background:#FAB387; color:#1E1E2E; font-weight:bold; }
QPushButton#landBtn:hover { background:#E8A07A; }
QTextEdit { background:#11111B; color:#A6E3A1; border:1px solid #313244;
            font-family:Consolas,monospace; font-size:11px; }
QLabel#keyLabel  { color:#89B4FA; font-weight:bold; }
QLabel#valLabel  { color:#CDD6F4; font-family:Consolas; }
QLabel#dimLabel  { color:#585B70; font-size:10px; }
QLabel#titleLbl  { font-size:15px; font-weight:bold; color:#89B4FA; }
QLabel#statusLbl { font-size:12px; font-weight:bold; }
QLabel#modeLbl   { font-size:13px; font-weight:bold; padding:2px 8px;
                   border-radius:4px; }
QSplitter::handle { background:#313244; }
"""

_MODE_STYLE: dict[str, str] = {
    "DISARMED":  "background:#45475A; color:#CDD6F4;",
    "ARMED":     "background:#FAB387; color:#1E1E2E;",
    "HOVER":     "background:#89B4FA; color:#1E1E2E;",
    "SPORT":     "background:#F38BA8; color:#1E1E2E;",
    "PRECISION": "background:#A6E3A1; color:#1E1E2E;",
    "LANDING":   "background:#F9E2AF; color:#1E1E2E;",
    "TAKEOFF":   "background:#94E2D5; color:#1E1E2E;",
}

# ---------------------------------------------------------------------------
# Control mapping data
# ---------------------------------------------------------------------------

_AXIS_MAP = [
    ("X Axis",          "Roll       (left ← / right →)"),
    ("Y Axis",          "Pitch      (forward ↑ / back ↓)"),
    ("Rz (Z Rotation)", "Yaw / Spin (rotate CCW ↺ / CW ↻)"),
    ("Slider",          "Altitude   (50 % = hold  ↑ climb  ↓ descend)"),
    ("Hat N/S",         "Altitude trim  (fine adjust)"),
]

_BTN_MAP = [
    ("Button 1", "ARM / DISARM",      "#FAB387"),
    ("Button 2", "Emergency LAND",    "#F38BA8"),
    ("Button 3", "HOVER / STABLE",    "#89B4FA"),
    ("Button 4", "Reset drone",       "#CDD6F4"),
    ("Button 5", "SPORT mode",        "#F38BA8"),
    ("Button 6", "PRECISION mode",    "#A6E3A1"),
    ("Button 7", "Auto TAKE-OFF",     "#94E2D5"),
]

# ---------------------------------------------------------------------------
# Simulator window
# ---------------------------------------------------------------------------

class DroneSimulatorWindow(QWidget):
    """Main drone simulator window.

    Parameters
    ----------
    device       : USBDevice   the connected joystick / controller
    scan_axes    : list[str]   axis names from the HID scan
    button_count : int
    has_hat      : bool
    axis_bit_sizes, field_map  : passed through to ControllerMonitorThread
    """

    _PHYSICS_HZ = 60          # physics + render update rate
    _LOG_MAX    = 400          # max log lines before truncation

    def __init__(
        self,
        device:          USBDevice,
        scan_axes:       list[str],
        button_count:    int,
        has_hat:         bool,
        axis_bit_sizes:  list[int] | None = None,
        field_map:       str = "",
        parent:          QWidget | None = None,
    ) -> None:
        super().__init__(parent)
        self._device       = device
        self._scan_axes    = scan_axes
        self._button_count = button_count
        self._has_hat      = has_hat

        self._physics  = DronePhysics()
        self._input    = DroneInput()
        self._last_input_state: InputState | None = None
        self._last_buttons: dict[int, bool] = {}
        self._last_hat_raw: int = 8          # 8 = centred

        # Button debounce: track which buttons were pressed last tick
        # so we only fire a rising edge (False→True) once per press.
        self._btn_prev: dict[int, bool] = {}

        self._prev_mode   = ""
        self._prev_cmd    = ""
        self._log_count   = 0
        self._sim_start   = time.monotonic()
        self._last_step   = time.monotonic()

        # Markdown flight logger — records everything to a .md file
        self._flight_logger = FlightLogger(
            device_name=device.name or device.device_id
        )
        self._calibrated_logged = False   # log calibration data once

        self.setWindowTitle(
            f"🚁  Drone Simulator — {device.name or device.device_id}")
        self.resize(1280, 760)
        self.setMinimumSize(900, 580)
        self.setStyleSheet(_STYLE)
        self.setAttribute(Qt.WidgetAttribute.WA_DeleteOnClose, True)
        self.setFocusPolicy(Qt.FocusPolicy.StrongFocus)

        self._build_ui()
        self._start_monitor(axis_bit_sizes, field_map)
        self._start_timer()
        self._log("Simulator ready.  Press Button 1 to ARM.")
        self._flight_logger.log_event(
            "INFO",
            f"Session started — controller: **{device.name or device.device_id}**  "
            f"VID:{device.vendor_id or '?'} PID:{device.product_id or '?'}"
        )
        self._log(f"📄 Log: {self._flight_logger.path}")

    # ------------------------------------------------------------------
    # UI construction
    # ------------------------------------------------------------------

    def _build_ui(self) -> None:
        root = QVBoxLayout(self)
        root.setContentsMargins(8, 8, 8, 6)
        root.setSpacing(6)

        root.addLayout(self._build_header())

        mid_splitter = QSplitter(Qt.Orientation.Horizontal)
        mid_splitter.addWidget(self._build_3d_panel())
        mid_splitter.addWidget(self._build_right_panel())
        mid_splitter.setStretchFactor(0, 3)
        mid_splitter.setStretchFactor(1, 2)
        mid_splitter.setSizes([820, 440])
        root.addWidget(mid_splitter, stretch=5)

        root.addWidget(self._build_log_panel(), stretch=1)

    # ------------------------------------------------------------------
    # Header
    # ------------------------------------------------------------------

    def _build_header(self) -> QHBoxLayout:
        row = QHBoxLayout()
        row.setSpacing(10)

        icon_lbl = QLabel("🚁")
        icon_lbl.setStyleSheet("font-size:22px;")
        row.addWidget(icon_lbl)

        title = QLabel(self._device.name or self._device.device_id)
        title.setObjectName("titleLbl")
        row.addWidget(title)

        self._mode_lbl = QLabel("DISARMED")
        self._mode_lbl.setObjectName("modeLbl")
        self._mode_lbl.setStyleSheet(_MODE_STYLE["DISARMED"])
        row.addWidget(self._mode_lbl)

        self._cmd_lbl = QLabel("Idle")
        self._cmd_lbl.setObjectName("statusLbl")
        self._cmd_lbl.setStyleSheet("color:#A6ADC8;")
        row.addWidget(self._cmd_lbl)

        row.addStretch()

        # Camera orbit buttons
        for label, delta in (("◁ Orbit", -15), ("Orbit ▷", 15),
                              ("⟳ Reset cam", None)):
            btn = QPushButton(label)
            btn.setFixedHeight(28)
            if delta is not None:
                d = delta
                btn.clicked.connect(lambda _, d=d: self._view.orbit(d))
            else:
                btn.clicked.connect(lambda: self._view.set_camera_yaw(25.0))
            row.addWidget(btn)

        # Zoom buttons
        zi_btn = QPushButton("🔍+")
        zi_btn.setFixedHeight(28); zi_btn.setFixedWidth(40)
        zi_btn.clicked.connect(lambda: self._view.zoom_in())
        row.addWidget(zi_btn)
        zo_btn = QPushButton("🔍−")
        zo_btn.setFixedHeight(28); zo_btn.setFixedWidth(40)
        zo_btn.clicked.connect(lambda: self._view.zoom_out())
        row.addWidget(zo_btn)
        zr_btn = QPushButton("1:1")
        zr_btn.setFixedHeight(28); zr_btn.setFixedWidth(36)
        zr_btn.clicked.connect(lambda: self._view.zoom_reset())
        row.addWidget(zr_btn)

        # Quick-action buttons
        arm_btn = QPushButton("ARM / DISARM")
        arm_btn.setObjectName("armBtn")
        arm_btn.setFixedHeight(28)
        arm_btn.clicked.connect(self._on_arm_click)
        row.addWidget(arm_btn)

        takeoff_btn = QPushButton("🚀 TAKE-OFF")
        takeoff_btn.setObjectName("armBtn")
        takeoff_btn.setFixedHeight(28)
        takeoff_btn.clicked.connect(self._on_takeoff_click)
        row.addWidget(takeoff_btn)

        land_btn = QPushButton("⬇ LAND")
        land_btn.setObjectName("landBtn")
        land_btn.setFixedHeight(28)
        land_btn.clicked.connect(self._on_land_click)
        row.addWidget(land_btn)

        self._ctrl_status = QLabel("● No controller")
        self._ctrl_status.setStyleSheet("color:#FFA726; font-weight:bold;")
        row.addWidget(self._ctrl_status)

        return row

    # ------------------------------------------------------------------
    # 3-D panel
    # ------------------------------------------------------------------

    def _build_3d_panel(self) -> QWidget:
        box = QGroupBox("3D View  (◁ ▷ Orbit  |  +/− Zoom  |  Scroll = Zoom  |  Arrow Keys = orbit)")
        outer = QVBoxLayout(box)
        outer.setContentsMargins(2, 2, 2, 2)
        outer.setSpacing(4)

        # View + guide side by side
        view_row = QHBoxLayout()
        view_row.setSpacing(4)

        self._view = Drone3DWidget()
        self._view.setSizePolicy(
            QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        view_row.addWidget(self._view, stretch=1)

        # ── Live control guide (right side of 3D view) ───────────────
        guide = QWidget()
        guide.setFixedWidth(190)
        guide.setStyleSheet("background:#11111B; border-radius:4px;")
        glayout = QVBoxLayout(guide)
        glayout.setContentsMargins(6, 6, 6, 6)
        glayout.setSpacing(5)

        guide_title = QLabel("CONTROLS")
        guide_title.setStyleSheet(
            "color:#89B4FA; font-weight:bold; font-size:11px;")
        glayout.addWidget(guide_title)

        # Mode-dependent instruction
        self._guide_mode_lbl = QLabel("Press ARM / DISARM to start")
        self._guide_mode_lbl.setWordWrap(True)
        self._guide_mode_lbl.setStyleSheet(
            "color:#FAB387; font-size:10px;")
        glayout.addWidget(self._guide_mode_lbl)

        # Calibration status
        self._cal_status_lbl = QLabel("⏳ Calibrating axes…")
        self._cal_status_lbl.setWordWrap(True)
        self._cal_status_lbl.setStyleSheet("color:#585B70; font-size:9px;")
        glayout.addWidget(self._cal_status_lbl)

        glayout.addWidget(self._hline())

        # Live axis indicators
        axis_lbl = QLabel("LIVE INPUTS")
        axis_lbl.setStyleSheet(
            "color:#89B4FA; font-weight:bold; font-size:10px;")
        glayout.addWidget(axis_lbl)

        self._axis_bars: dict[str, QProgressBar] = {}
        for name, key in (
            ("Roll (X)",   "roll"),
            ("Pitch (Y)",  "pitch"),
            ("Yaw (Rz)",   "yaw"),
            ("Throttle",   "thr"),
        ):
            row_w = QWidget()
            row_h = QHBoxLayout(row_w)
            row_h.setContentsMargins(0, 0, 0, 0)
            row_h.setSpacing(4)

            lbl = QLabel(name)
            lbl.setFixedWidth(72)
            lbl.setStyleSheet("color:#A6ADC8; font-size:10px;")
            row_h.addWidget(lbl)

            bar = QProgressBar()
            bar.setRange(0, 200)   # 0=full-left/down, 100=centre, 200=full-right/up
            bar.setValue(100)
            bar.setTextVisible(False)
            bar.setFixedHeight(10)
            bar.setStyleSheet(
                "QProgressBar{background:#313244;border:none;border-radius:2px;}"
                "QProgressBar::chunk{background:#89B4FA;border-radius:2px;}")
            row_h.addWidget(bar)
            self._axis_bars[key] = bar
            glayout.addWidget(row_w)

        glayout.addWidget(self._hline())

        # Button state indicators (1–7)
        btn_grid_lbl = QLabel("BUTTONS")
        btn_grid_lbl.setStyleSheet(
            "color:#89B4FA; font-weight:bold; font-size:10px;")
        glayout.addWidget(btn_grid_lbl)

        btn_container = QWidget()
        btn_grid = QGridLayout(btn_container)
        btn_grid.setSpacing(3)
        btn_grid.setContentsMargins(0, 0, 0, 0)

        self._guide_btn_labels: dict[int, QLabel] = {}
        short_names = {
            1: "ARM",  2: "LAND",   3: "HOVER",
            4: "RESET",5: "SPORT",  6: "PREC.", 7: "T.OFF",
        }
        for i in range(1, 8):
            lbl = QLabel(f"{i}:{short_names[i]}")
            lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
            lbl.setFixedHeight(20)
            lbl.setStyleSheet(
                "background:#1E1E2E; color:#585B70; border:1px solid #313244;"
                "border-radius:3px; font-size:9px;")
            btn_grid.addWidget(lbl, (i - 1) // 4, (i - 1) % 4)
            self._guide_btn_labels[i] = lbl
        glayout.addWidget(btn_container)

        glayout.addStretch()

        # Quick-start tip
        tip = QLabel(
            "Slider 50% = hold alt\n"
            "> 50% = climb\n"
            "< 50% = descend\n\n"
            "ARM → Btn 7 (auto\n"
            "take-off to 3 m)")
        tip.setStyleSheet("color:#585B70; font-size:9px;")
        tip.setWordWrap(True)
        glayout.addWidget(tip)

        view_row.addWidget(guide)
        outer.addLayout(view_row)
        return box

    # ------------------------------------------------------------------
    # Right panel (control map + telemetry)
    # ------------------------------------------------------------------

    def _build_right_panel(self) -> QWidget:
        w = QWidget()
        lay = QVBoxLayout(w)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(6)
        lay.addWidget(self._build_control_map())
        lay.addWidget(self._build_telemetry())
        return w

    def _build_control_map(self) -> QGroupBox:
        box = QGroupBox("Controller Mapping")
        grid = QGridLayout(box)
        grid.setSpacing(3)
        grid.setColumnMinimumWidth(0, 110)
        grid.setColumnMinimumWidth(1, 200)

        row = 0
        hdr = QLabel("AXES")
        hdr.setObjectName("keyLabel")
        grid.addWidget(hdr, row, 0, 1, 2); row += 1

        for axis_name, description in _AXIS_MAP:
            k = QLabel(axis_name)
            k.setObjectName("keyLabel")
            v = QLabel(description)
            v.setObjectName("valLabel")
            grid.addWidget(k, row, 0)
            grid.addWidget(v, row, 1)
            row += 1

        spacer = QLabel("")
        grid.addWidget(spacer, row, 0); row += 1

        hdr2 = QLabel("BUTTONS")
        hdr2.setObjectName("keyLabel")
        grid.addWidget(hdr2, row, 0, 1, 2); row += 1

        self._btn_map_labels: dict[int, QLabel] = {}
        for i, (btn_name, action, colour) in enumerate(_BTN_MAP, start=1):
            k = QLabel(btn_name)
            k.setObjectName("keyLabel")
            k.setStyleSheet(f"color:{colour};")
            v = QLabel(action)
            v.setObjectName("valLabel")
            grid.addWidget(k, row, 0)
            grid.addWidget(v, row, 1)
            self._btn_map_labels[i] = k
            row += 1

        box.setSizePolicy(QSizePolicy.Policy.Preferred,
                          QSizePolicy.Policy.Fixed)
        return box

    def _build_telemetry(self) -> QGroupBox:
        box = QGroupBox("Telemetry")
        grid = QGridLayout(box)
        grid.setSpacing(4)

        fields = [
            ("Altitude",    "m",   "alt"),
            ("H Speed",     "m/s", "hspd"),
            ("V Speed",     "m/s", "vspd"),
            ("Throttle",    "%",   "thr"),
            ("Heading",     "°",   "hdg"),
            ("Pitch",       "°",   "pitch"),
            ("Roll",        "°",   "roll"),
            ("X pos",       "m",   "xpos"),
            ("Z pos",       "m",   "zpos"),
            ("Distance",    "m",   "dist"),
            ("Flight time", "s",   "ftime"),
        ]
        self._telem: dict[str, QLabel] = {}
        for i, (label, unit, key) in enumerate(fields):
            col = (i % 2) * 3
            r   = i // 2
            lk = QLabel(f"{label}:")
            lk.setObjectName("keyLabel")
            lv = QLabel("—")
            lv.setObjectName("valLabel")
            lu = QLabel(unit)
            lu.setObjectName("dimLabel")
            grid.addWidget(lk, r, col)
            grid.addWidget(lv, r, col + 1)
            grid.addWidget(lu, r, col + 2)
            self._telem[key] = lv

        return box

    @staticmethod
    def _hline() -> QWidget:
        """Return a thin horizontal separator line."""
        line = QWidget()
        line.setFixedHeight(1)
        line.setStyleSheet("background:#313244;")
        return line

    def _build_log_panel(self) -> QGroupBox:
        box = QGroupBox("Flight Log")
        lay = QVBoxLayout(box)
        lay.setContentsMargins(4, 4, 4, 4)
        self._log_view = QTextEdit()
        self._log_view.setReadOnly(True)
        self._log_view.setFixedHeight(110)
        lay.addWidget(self._log_view)
        return box

    # ------------------------------------------------------------------
    # Controller monitor
    # ------------------------------------------------------------------

    def _start_monitor(self, axis_bit_sizes, field_map) -> None:
        if not self._device.vendor_id or not self._device.product_id:
            self._log("⚠  No VID/PID — keyboard fallback only.")
            return
        try:
            vid = int(self._device.vendor_id, 16)
            pid = int(self._device.product_id, 16)
        except ValueError:
            return

        self._monitor = ControllerMonitorThread(
            vid=vid, pid=pid,
            axis_names=self._scan_axes,
            button_count=self._button_count,
            has_hat=self._has_hat,
            axis_bit_sizes=axis_bit_sizes or None,
            field_map=field_map,
            parent=self,
        )
        self._monitor.state_updated.connect(self._on_controller_state)
        self._monitor.monitor_error.connect(self._on_monitor_error)
        self._monitor.start()

    # ------------------------------------------------------------------
    # Physics timer
    # ------------------------------------------------------------------

    def _start_timer(self) -> None:
        self._timer = QTimer(self)
        # 16 ms = ~60 Hz. PreciseTimer reduces OS timer jitter so physics
        # steps are more uniform and control response feels tighter.
        self._timer.setTimerType(Qt.TimerType.PreciseTimer)
        self._timer.setInterval(16)
        self._timer.timeout.connect(self._physics_tick)
        self._timer.start()

    @Slot()
    def _physics_tick(self) -> None:
        now  = time.monotonic()
        dt   = min(now - self._last_step, 0.05)
        self._last_step = now

        # Advance logger clock FIRST so log_input and log_state
        # both see the same tick value this frame.
        self._flight_logger.tick()

        inp, btns_now = self._build_drone_input()
        state = self._physics.step(inp, dt)
        self._view.update_state(state)
        self._update_telemetry(state)
        self._update_mode_label(state)
        self._update_guide_inputs(inp, btns_now)

    # ------------------------------------------------------------------
    # Build DroneInput from latest controller state
    # ------------------------------------------------------------------

    def _build_drone_input(self) -> tuple[DroneInput, dict[int, bool]]:
        """Build a DroneInput from the latest controller state.

        Mapping to the altitude-hold physics model
        ------------------------------------------
        inp.pitch > 0  → FORWARD   (no sign inversion — y_coord>0 = forward)
        inp.throttle   → 0.5 = hold altitude, >0.5 = climb, <0.5 = descend
                         Direct pass-through of 0-1 slider value.
                         The physics engine handles the hold dead-zone.
        """
        s = self._last_input_state
        inp = DroneInput()
        btns_now: dict[int, bool] = {}

        if s is not None:
            m = s.motion

            # ── Axes ────────────────────────────────────────────────
            # Roll: right → positive (bank right)
            inp.roll  =  m.x_coord

            # Pitch: y_coord > 0 = forward (MotionInterpreter convention)
            # Physics: pitch > 0 = FORWARD — same sign, no inversion needed
            inp.pitch =  m.y_coord

            # Yaw: twist CW = positive, 8 % dead-zone
            twist_raw = (m.twist_percent - 50.0) / 50.0
            inp.yaw   = twist_raw if abs(twist_raw) > 0.08 else 0.0

            # ── Throttle (altitude-hold model) ───────────────────────
            # Pass 0-1 directly. Physics dead-zone ±0.08 around 0.5
            # makes 0.5 = "hold altitude." No floor dead-zone here —
            # that was causing instant crashes at idle slider position.
            inp.throttle = m.throttle_percent / 100.0

            # ── Buttons — rising-edge only ───────────────────────────
            btns_now = {b.index: b.pressed for b in s.buttons}

            def rose(idx: int) -> bool:
                now_  = btns_now.get(idx, False)
                prev_ = self._btn_prev.get(idx, False)
                return now_ and not prev_

            inp.btn_arm       = rose(1)
            inp.btn_land      = rose(2)
            inp.btn_hover     = rose(3)
            inp.btn_reset     = rose(4)
            inp.btn_sport     = rose(5)
            inp.btn_precision = rose(6)
            inp.btn_takeoff   = rose(7)

            # Store for next frame's edge detection
            self._btn_prev = btns_now

            # ── Hat ──────────────────────────────────────────────────
            if s.hat:
                inp.hat_up   = (s.hat.raw == 0)   # North
                inp.hat_down = (s.hat.raw == 4)   # South

        return inp, btns_now

    # ------------------------------------------------------------------
    # Controller signal handlers
    # ------------------------------------------------------------------

    @Slot(object)
    def _on_controller_state(self, state: InputState) -> None:
        self._last_input_state = state
        self._ctrl_status.setText("● Live")
        self._ctrl_status.setStyleSheet("color:#A6E3A1; font-weight:bold;")

        # Check calibration completion (fires once)
        if not self._calibrated_logged and hasattr(self, "_monitor"):
            decoder = getattr(self._monitor, "_decoder", None)
            if decoder:
                interp = getattr(decoder, "_interpreter", None)
                if interp and interp.is_calibrated():
                    offsets = interp.get_calibration_info()
                    self._calibrated_logged = True
                    self._flight_logger.log_calibration(offsets, frames_used=60)
                    # Build calibration summary for UI
                    lines = []
                    for ax, off in sorted(offsets.items()):
                        if abs(off) > 0.5:
                            short = ax.split()[0] if " " in ax else ax
                            lines.append(f"{short}: {off:+.1f}%")
                    summary = "  ".join(lines) if lines else "all centred"
                    self._cal_status_lbl.setText(f"✓ Cal: {summary}")
                    self._cal_status_lbl.setStyleSheet(
                        "color:#A6E3A1; font-size:9px;")
                    self._log(f"📐 Calibrated — offsets: {summary}")
                    # Log raw vs calibrated axis snapshot
                    axes_raw = {ax.name: ax.percent
                                for ax in state.axes}
                    self._flight_logger.log_axis_diagnostic(
                        axes_raw=axes_raw,
                        axes_cal={ax: round(50.0 + (axes_raw.get(ax, 50.0)
                                   - (50.0 + off)), 2)
                                  for ax, off in offsets.items()},
                    )

    @Slot(str)
    def _on_monitor_error(self, msg: str) -> None:
        self._ctrl_status.setText("● No signal")
        self._ctrl_status.setStyleSheet("color:#EF5350; font-weight:bold;")
        self._log(f"⚠  Controller: {msg}")
        self._flight_logger.log_error("ControllerMonitor", msg)

    # ------------------------------------------------------------------
    # UI state updates
    # ------------------------------------------------------------------

    def _update_telemetry(self, state) -> None:
        t = self._telem
        t["alt"]  .setText(f"{state.altitude:7.2f}")
        t["hspd"] .setText(f"{state.speed_h:7.2f}")
        t["vspd"] .setText(f"{state.speed_v:+7.2f}")
        t["thr"]  .setText(f"{state.throttle * 100:5.1f}")
        t["hdg"]  .setText(f"{state.heading:6.1f}")
        t["pitch"].setText(f"{state.pitch:+6.1f}")
        t["roll"] .setText(f"{state.roll:+6.1f}")
        t["xpos"] .setText(f"{state.x:7.2f}")
        t["zpos"] .setText(f"{state.z:7.2f}")
        t["dist"] .setText(f"{state.total_distance:7.1f}")
        t["ftime"].setText(f"{state.flight_time:7.1f}")

    def _update_mode_label(self, state) -> None:
        mode = state.mode.value
        cmd  = state.flight_command

        if mode != self._prev_mode:
            self._mode_lbl.setText(mode)
            self._mode_lbl.setStyleSheet(_MODE_STYLE.get(mode, ""))
            self._log(f"▶  Mode changed → {mode}")
            self._flight_logger.log_event("MODE",
                f"Mode changed: **{self._prev_mode or 'INIT'}** → **{mode}**")
            self._prev_mode = mode
            self._update_guide_instructions(mode)

        if cmd != self._prev_cmd:
            self._cmd_lbl.setText(cmd)
            if cmd not in ("Grounded", "Idle", "Stable Hover",
                           "Disarmed — press Button 1 to ARM"):
                self._log(f"✈  {cmd}")
                self._flight_logger.log_event("COMMAND", cmd)
            self._prev_cmd = cmd

        # Per-second telemetry snapshot to MD log
        self._flight_logger.log_state(
            mode=mode, command=cmd,
            altitude=state.altitude, speed_h=state.speed_h,
            speed_v=state.speed_v,  heading=state.heading,
            pitch=state.pitch,      roll=state.roll,
            x=state.x,              z=state.z,
            flight_time=state.flight_time,
        )

    def _update_guide_instructions(self, mode: str) -> None:
        """Update the guide panel's mode-specific instruction text."""
        instructions = {
            "DISARMED":  "Click ARM or press Button 1 to arm the drone.",
            "ARMED":     "Press Button 7 or 🚀 for auto take-off.\n\n"
                         "Or: move Slider above 50 % to climb manually.",
            "HOVER":     "Stick forward/back/left/right to fly.\n"
                         "Twist = Yaw (rotate).\n"
                         "Slider 50 % = hold altitude.\n"
                         "Above 50 % = climb, below = descend.",
            "SPORT":     "⚡ SPORT: up to 18 m/s.\n"
                         "Same controls as HOVER.",
            "PRECISION": "🎯 PRECISION: max 3 m/s.\n"
                         "For close-in manoeuvres.",
            "TAKEOFF":   "⬆ Auto-climbing to 3 m…\n"
                         "Hands off — will enter HOVER.",
            "LANDING":   "⬇ Auto-landing in progress…\n"
                         "Keep clear.",
        }
        text = instructions.get(mode, "")
        self._guide_mode_lbl.setText(text)
        colour = _MODE_STYLE.get(mode, "color:#A6ADC8;")
        # Extract just the colour part for the label
        bg = {
            "DISARMED":  "#585B70", "ARMED":    "#FAB387",
            "HOVER":     "#89B4FA", "SPORT":    "#F38BA8",
            "PRECISION": "#A6E3A1", "TAKEOFF":  "#94E2D5",
            "LANDING":   "#F9E2AF",
        }.get(mode, "#A6ADC8")
        self._guide_mode_lbl.setStyleSheet(f"color:{bg}; font-size:10px;")

    def _update_guide_inputs(self, inp: DroneInput,
                             btns_now: dict[int, bool]) -> None:
        """Update live axis bars, button lights, and write input log row."""
        bars = self._axis_bars

        # Roll / Pitch / Yaw: -1..+1 → 0..200 (centre = 100)
        bars["roll"] .setValue(int((inp.roll  + 1.0) * 100))
        bars["pitch"].setValue(int((inp.pitch + 1.0) * 100))
        bars["yaw"]  .setValue(int((inp.yaw   + 1.0) * 100))
        bars["thr"]  .setValue(int(inp.throttle * 200))

        # Colour: blue when near centre, green when actively deflected
        for key, bar in bars.items():
            v      = bar.value()
            active = (abs(v - 100) > 20 if key == "thr"
                      else abs(v - 100) > 15)
            colour = "#A6E3A1" if active else "#89B4FA"
            bar.setStyleSheet(
                f"QProgressBar{{background:#313244;border:none;border-radius:2px;}}"
                f"QProgressBar::chunk{{background:{colour};border-radius:2px;}}")

        # Guide button lights
        for idx, lbl in self._guide_btn_labels.items():
            pressed = btns_now.get(idx, False)
            if pressed:
                lbl.setStyleSheet(
                    "background:#A6E3A1; color:#1E1E2E; border:1px solid #4CAF50;"
                    "border-radius:3px; font-size:9px; font-weight:bold;")
            else:
                lbl.setStyleSheet(
                    "background:#1E1E2E; color:#585B70; border:1px solid #313244;"
                    "border-radius:3px; font-size:9px;")

        # Right-panel control mapping labels
        for idx, lbl in self._btn_map_labels.items():
            if btns_now.get(idx, False):
                lbl.setStyleSheet(
                    "color:#1E1E2E; background:#A6E3A1;"
                    " border-radius:3px; padding:1px 3px;")
            else:
                colour = _BTN_MAP[idx - 1][2] if idx <= len(_BTN_MAP) else "#CDD6F4"
                lbl.setStyleSheet(f"color:{colour};")

        # Log button presses (each press = one event line)
        for idx, pressed in btns_now.items():
            if pressed and not self._btn_prev.get(idx, False):
                btn_name = _BTN_MAP[idx - 1][1] if idx <= len(_BTN_MAP) else f"Btn {idx}"
                self._flight_logger.log_event(
                    "BUTTON", f"Button **{idx}** pressed — {btn_name}")

        # Sample raw input row to MD log (logger handles 1 Hz decimation)
        m = self._last_input_state.motion if self._last_input_state else None
        self._flight_logger.log_input(
            roll=inp.roll, pitch=inp.pitch, yaw=inp.yaw,
            throttle=inp.throttle,
            x_coord=inp.roll,    y_coord=inp.pitch,
            twist_pct=m.twist_percent if m else 50.0,
            thr_pct=m.throttle_percent if m else 50.0,
            btns_pressed=[i for i, v in btns_now.items() if v],
        )

    # ------------------------------------------------------------------
    # Manual button handlers (on-screen buttons)
    # ------------------------------------------------------------------

    def _on_arm_click(self) -> None:
        """Inject a single ARM pulse directly into the physics engine."""
        inp = DroneInput()
        inp.btn_arm = True
        # Use current slider position; default to 0.5 (hold altitude)
        if self._last_input_state:
            inp.throttle = self._last_input_state.motion.throttle_percent / 100.0
        else:
            inp.throttle = 0.5
        self._physics.step(inp, 1 / self._PHYSICS_HZ)

    def _on_land_click(self) -> None:
        """Inject a single LAND pulse."""
        inp = DroneInput()
        inp.btn_land = True
        inp.throttle = 0.5
        self._physics.step(inp, 1 / self._PHYSICS_HZ)

    def _on_takeoff_click(self) -> None:
        """Inject a single TAKEOFF pulse."""
        inp = DroneInput()
        inp.btn_takeoff = True
        inp.throttle = 0.5
        self._physics.step(inp, 1 / self._PHYSICS_HZ)

    # ------------------------------------------------------------------
    # Flight log helpers
    # ------------------------------------------------------------------

    def _log(self, msg: str) -> None:
        ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        line = f"{ts}  {msg}"
        self._log_view.append(line)
        sb = self._log_view.verticalScrollBar()
        sb.setValue(sb.maximum())
        self._log_count += 1
        if self._log_count > self._LOG_MAX:
            # Trim oldest 25 %
            doc  = self._log_view.document()
            cur  = self._log_view.textCursor()
            from PySide6.QtGui import QTextCursor
            cur.movePosition(QTextCursor.MoveOperation.Start)
            for _ in range(self._LOG_MAX // 4):
                cur.select(QTextCursor.SelectionType.LineUnderCursor)
                cur.removeSelectedText()
                cur.deleteChar()
            self._log_count -= self._LOG_MAX // 4

    # ------------------------------------------------------------------
    # Keyboard fallback (WASD + QE + RF)
    # ------------------------------------------------------------------

    def keyPressEvent(self, event: QKeyEvent) -> None:   # noqa: N802
        key = event.key()
        inp = self._physics.state
        step = 0.05

        # ARM on Space
        if key == Qt.Key.Key_Space:
            from app.core.drone_physics import DroneInput as _DI
            di = DroneInput(); di.btn_arm = True
            self._physics.step(di, 0.016)
            self._flight_logger.log_event("BUTTON", "Keyboard Space → ARM/DISARM")

        elif key == Qt.Key.Key_T:
            di = DroneInput(); di.btn_takeoff = True
            self._physics.step(di, 0.016)
            self._flight_logger.log_event("TAKEOFF", "Keyboard T → TAKEOFF")
        elif key == Qt.Key.Key_L:
            di = DroneInput(); di.btn_land = True
            self._physics.step(di, 0.016)
            self._flight_logger.log_event("LAND", "Keyboard L → LAND")
        elif key == Qt.Key.Key_H:
            di = DroneInput(); di.btn_hover = True
            self._physics.step(di, 0.016)
            self._flight_logger.log_event("MODE", "Keyboard H → HOVER toggle")

        # Camera orbit
        elif key == Qt.Key.Key_Left:
            self._view.orbit(-5)
        elif key == Qt.Key.Key_Right:
            self._view.orbit(5)
        # Zoom
        elif key == Qt.Key.Key_Plus or key == Qt.Key.Key_Equal:
            self._view.zoom_in()
        elif key == Qt.Key.Key_Minus:
            self._view.zoom_out()
        elif key == Qt.Key.Key_0:
            self._view.zoom_reset()
        else:
            super().keyPressEvent(event)

    # ------------------------------------------------------------------
    # Window lifecycle
    # ------------------------------------------------------------------

    def closeEvent(self, event) -> None:   # noqa: N802
        self._timer.stop()
        if hasattr(self, "_monitor") and self._monitor.isRunning():
            self._monitor.stop()
        self._flight_logger.log_event("INFO", "Simulator window closed")
        self._flight_logger.close()
        event.accept()
