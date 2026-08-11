"""Main application window (spec sections 12, 13, 20, 26).

Layout:
  ┌─────────────────────────────────────────┐
  │  Toolbar: title label + action buttons  │
  ├─────────────────────────────────────────┤
  │  DeviceListWidget  (top, ~60 % height)  │
  ├─────────────────────────────────────────┤
  │  DeviceDetailsWidget (middle, hideable) │
  ├─────────────────────────────────────────┤
  │  EventLogWidget  (bottom, ~25 % height) │
  └─────────────────────────────────────────┘

DeviceService is created here and owned for the lifetime of the window.
All background → UI signal connections are made in ``_connect_signals``.
"""
from __future__ import annotations

from PySide6.QtCore import Qt, QTimer, Slot
from PySide6.QtGui import QAction, QCloseEvent, QFont, QIcon
from PySide6.QtWidgets import (
    QDialog,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QSizePolicy,
    QSplitter,
    QStatusBar,
    QToolBar,
    QVBoxLayout,
    QWidget,
)

from app.models.device_details import DeviceDetails
from app.models.usb_device import USBDevice
from app.services.device_service import DeviceService
from app.services.scanning_service import ScanSignals
from app.ui.device_details import DeviceDetailsWidget
from app.ui.device_list import DeviceListWidget
from app.ui.event_log import EventLogWidget
from app.ui.scan_dialog import ScanDialog
from app.usb.usb_constants import DeviceCategory
from app.utils.logger import get_logger

logger = get_logger(__name__)

_APP_TITLE = "USB Device Inspector"
_STYLE = """
QMainWindow, QWidget {
    background-color: #1E1E2E;
    color: #CDD6F4;
    font-family: "Segoe UI", Arial, sans-serif;
    font-size: 13px;
}
QTableWidget {
    background-color: #181825;
    alternate-background-color: #1E1E2E;
    gridline-color: #313244;
    border: 1px solid #313244;
    border-radius: 4px;
    selection-background-color: #45475A;
    selection-color: #CDD6F4;
}
QHeaderView::section {
    background-color: #313244;
    color: #CDD6F4;
    padding: 4px 8px;
    border: none;
    border-right: 1px solid #45475A;
    font-weight: bold;
}
QPushButton {
    background-color: #313244;
    color: #CDD6F4;
    border: 1px solid #45475A;
    border-radius: 5px;
    padding: 6px 16px;
}
QPushButton:hover { background-color: #45475A; }
QPushButton:pressed { background-color: #585B70; }
QPushButton:disabled { color: #585B70; border-color: #313244; }
QPushButton#primaryButton {
    background-color: #89B4FA;
    color: #1E1E2E;
    font-weight: bold;
    border: none;
}
QPushButton#primaryButton:hover { background-color: #74C7EC; }
QPushButton#smallButton {
    padding: 2px 10px;
    font-size: 11px;
}
QTextEdit#eventLog {
    background-color: #11111B;
    color: #A6E3A1;
    border: 1px solid #313244;
    border-radius: 4px;
    font-family: "Consolas", "Courier New", monospace;
    font-size: 12px;
}
QTabWidget::pane {
    border: 1px solid #313244;
    border-radius: 4px;
}
QTabBar::tab {
    background: #313244;
    color: #CDD6F4;
    padding: 5px 14px;
    margin-right: 2px;
    border-top-left-radius: 4px;
    border-top-right-radius: 4px;
}
QTabBar::tab:selected { background: #45475A; color: #89B4FA; }
QTabBar::tab:hover { background: #45475A; }
QProgressBar {
    background-color: #313244;
    border: 1px solid #45475A;
    border-radius: 4px;
    text-align: center;
    color: #CDD6F4;
}
QProgressBar::chunk { background-color: #89B4FA; border-radius: 3px; }
QScrollArea, QScrollBar {
    background-color: #1E1E2E;
    border: none;
}
QScrollBar:vertical { width: 8px; background: #181825; }
QScrollBar::handle:vertical { background: #45475A; border-radius: 4px; }
QLabel#sectionLabel { font-size: 14px; font-weight: bold; color: #89B4FA; }
QLabel#fieldLabel { color: #A6ADC8; }
QLabel#fieldValue { color: #CDD6F4; }
QLabel#warningLabel {
    color: #FAB387;
    background-color: #2A1F1A;
    border: 1px solid #FAB387;
    border-radius: 4px;
    padding: 4px 8px;
}
QLabel#scanStatusLabel { font-size: 13px; }
QSplitter::handle { background-color: #313244; }
QStatusBar { background-color: #181825; color: #A6ADC8; font-size: 12px; }
"""


class MainWindow(QMainWindow):
    """The top-level application window."""

    def __init__(self) -> None:
        super().__init__()
        self._service = DeviceService(parent=self)
        self._selected_device: USBDevice | None = None
        self._last_scan_details: DeviceDetails | None = None

        self._setup_ui()
        self._connect_signals()
        self.setStyleSheet(_STYLE)

        # Defer initialization so the window is visible before the first WMI call.
        QTimer.singleShot(100, self._service.initialize)
        logger.info("Main window created")

    # ------------------------------------------------------------------
    # UI construction
    # ------------------------------------------------------------------

    def _setup_ui(self) -> None:
        self.setWindowTitle(_APP_TITLE)
        self.resize(960, 720)
        self.setMinimumSize(720, 520)

        central = QWidget()
        self.setCentralWidget(central)
        root = QVBoxLayout(central)
        root.setContentsMargins(8, 8, 8, 8)
        root.setSpacing(8)

        # ── Title bar ───────────────────────────────────────────────
        title_row = QHBoxLayout()
        app_title = QLabel(_APP_TITLE)
        title_font = QFont()
        title_font.setPointSize(16)
        title_font.setBold(True)
        app_title.setFont(title_font)
        app_title.setStyleSheet("color: #89B4FA;")
        title_row.addWidget(app_title)
        title_row.addStretch()
        root.addLayout(title_row)

        # ── Action buttons ───────────────────────────────────────────
        btn_row = QHBoxLayout()
        btn_row.setSpacing(8)

        self._inspect_btn = QPushButton("Inspect Device")
        self._inspect_btn.setObjectName("primaryButton")
        self._inspect_btn.setEnabled(False)
        self._inspect_btn.clicked.connect(self._on_inspect)
        btn_row.addWidget(self._inspect_btn)

        self._scan_btn = QPushButton("Scan Device")
        self._scan_btn.setEnabled(False)
        self._scan_btn.clicked.connect(self._on_scan)
        btn_row.addWidget(self._scan_btn)

        self._test_btn = QPushButton("🎮  Test Controller Live")
        self._test_btn.setEnabled(False)
        self._test_btn.setVisible(False)
        self._test_btn.clicked.connect(self._on_test_controller)
        btn_row.addWidget(self._test_btn)

        self._refresh_btn = QPushButton("Refresh")
        self._refresh_btn.clicked.connect(self._on_refresh)
        btn_row.addWidget(self._refresh_btn)

        self._history_btn = QPushButton("Device History")
        self._history_btn.clicked.connect(self._on_show_history)
        btn_row.addWidget(self._history_btn)

        btn_row.addStretch()
        root.addLayout(btn_row)
        # ── Splitter: device list / details / event log ──────────────
        splitter = QSplitter(Qt.Orientation.Vertical)

        # Device list (top panel)
        self._device_list = DeviceListWidget()
        splitter.addWidget(self._device_list)

        # Device details (middle — hidden until a device is selected/scanned)
        self._details_container = QWidget()
        details_layout = QVBoxLayout(self._details_container)
        details_layout.setContentsMargins(0, 0, 0, 0)
        details_header = QHBoxLayout()
        details_title = QLabel("Device Details")
        details_title.setObjectName("sectionLabel")
        details_header.addWidget(details_title)
        details_header.addStretch()
        self._close_details_btn = QPushButton("✕")
        self._close_details_btn.setObjectName("smallButton")
        self._close_details_btn.setFixedWidth(28)
        self._close_details_btn.clicked.connect(self._hide_details)
        details_header.addWidget(self._close_details_btn)
        details_layout.addLayout(details_header)

        self._device_details = DeviceDetailsWidget()
        details_layout.addWidget(self._device_details)
        self._details_container.hide()
        splitter.addWidget(self._details_container)

        # Event log (bottom panel)
        self._event_log = EventLogWidget()
        splitter.addWidget(self._event_log)

        splitter.setStretchFactor(0, 3)
        splitter.setStretchFactor(1, 4)
        splitter.setStretchFactor(2, 2)
        splitter.setSizes([320, 0, 200])

        root.addWidget(splitter)

        # ── Status bar ───────────────────────────────────────────────
        self._status_bar = QStatusBar()
        self.setStatusBar(self._status_bar)
        self._status_label = QLabel("Ready")
        self._status_bar.addWidget(self._status_label)

    # ------------------------------------------------------------------
    # Signal connections
    # ------------------------------------------------------------------

    def _connect_signals(self) -> None:
        svc = self._service
        svc.devices_changed.connect(self._on_devices_changed)
        svc.device_connected.connect(self._on_device_connected)
        svc.device_removed.connect(self._on_device_removed)
        svc.event_logged.connect(self._event_log.append_event)
        svc.monitor_error.connect(self._on_monitor_error)

        self._device_list.device_selected.connect(self._on_device_selected)

    # ------------------------------------------------------------------
    # Service signal handlers
    # ------------------------------------------------------------------

    @Slot()
    def _on_devices_changed(self) -> None:
        previous_id = self._selected_device.device_id if self._selected_device else None
        devices = self._service.all_devices()
        self._device_list.refresh(devices)
        self._update_status()
        # Restore selection if the device is still present.
        if previous_id:
            self._device_list.select_device_by_id(previous_id)

    @Slot(object)
    def _on_device_connected(self, device: USBDevice) -> None:
        self._set_status(f"Connected: {device.name or device.device_id}")

    @Slot(str)
    def _on_device_removed(self, device_id: str) -> None:
        self._set_status(f"Removed: {device_id}")
        if self._selected_device and self._selected_device.device_id == device_id:
            self._selected_device = None
            self._inspect_btn.setEnabled(False)
            self._scan_btn.setEnabled(False)
            self._test_btn.setVisible(False)
            self._test_btn.setEnabled(False)
            self._hide_details()

    @Slot(str)
    def _on_monitor_error(self, message: str) -> None:
        self._set_status(f"Monitor error: {message}", error=True)

    # ------------------------------------------------------------------
    # Device selection
    # ------------------------------------------------------------------

    @Slot(object)
    def _on_device_selected(self, device: USBDevice) -> None:
        self._selected_device = device
        self._inspect_btn.setEnabled(True)
        self._scan_btn.setEnabled(True)

        # Show "Test Controller Live" only for Game Controller category
        is_controller = device.category == DeviceCategory.GAME_CONTROLLER.value
        self._test_btn.setVisible(is_controller)
        self._test_btn.setEnabled(is_controller)

        self._set_status(
            f"{device.name or device.device_id}  |  "
            f"{device.category}  |  "
            f"VID: {device.vendor_id or '—'}  PID: {device.product_id or '—'}"
        )

    # ------------------------------------------------------------------
    # Button handlers
    # ------------------------------------------------------------------

    @Slot()
    def _on_inspect(self) -> None:
        """Show last scan details (or the most recent device info) immediately."""
        device = self._selected_device
        if not device:
            return
        if self._last_scan_details and self._last_scan_details.device.device_id == device.device_id:
            self._show_details(self._last_scan_details)
        else:
            # Perform a quick synchronous scan to populate the details panel
            # (non-blocking relative to the monitor thread; runs on this call).
            self._set_status("Inspecting device…")
            self._on_scan()

    @Slot()
    def _on_scan(self) -> None:
        device = self._selected_device
        if not device:
            return
        if self._service.is_scanning(device.device_id):
            self._set_status("Scan already in progress…")
            return

        signals = ScanSignals()
        dialog = ScanDialog(device, signals, parent=self)
        signals.scan_complete.connect(self._on_scan_complete)
        signals.scan_failed.connect(self._on_scan_failed)

        self._service.start_scan(device, signals)
        self._scan_btn.setEnabled(False)
        dialog.exec()
        self._scan_btn.setEnabled(True)

    @Slot()
    def _on_refresh(self) -> None:
        self._set_status("Refreshing device list…")
        self._service.refresh()

    @Slot()
    def _on_test_controller(self) -> None:
        """Open the live controller test window for the selected device."""
        device = self._selected_device
        if not device:
            return
        self._open_controller_test(device)

    def _open_controller_test(self, device: USBDevice) -> None:
        """Extract scan-time axis/button info and open ControllerTestWidget."""
        from app.ui.controller_test_widget import ControllerTestWidget

        scan_axes: list[str] = []
        button_count = 0
        has_hat = False
        axis_bit_sizes: list[int] = []

        if self._last_scan_details and self._last_scan_details.device.device_id == device.device_id:
            details = self._last_scan_details
            for section in details.sections:
                if section.title == "HID Analysis":
                    for f in section.fields:
                        if f.label == "Axes" and f.value:
                            scan_axes = [a.strip() for a in f.value.split(",") if a.strip()]
                        if f.label == "Button Count" and f.value and f.value.isdigit():
                            button_count = int(f.value)
                        if f.label == "Hat Switch(es)" and f.value and f.value not in ("Not Available", "None"):
                            try:
                                has_hat = int(f.value) > 0
                            except ValueError:
                                has_hat = False
                        if f.label == "Axis Bit Sizes" and f.value:
                            try:
                                axis_bit_sizes = [int(x) for x in f.value.split(",")]
                            except ValueError:
                                axis_bit_sizes = []
                    break

        if not scan_axes:
            scan_axes = ["X Axis", "Y Axis", "Z Axis", "Rz (Z Rotation)"]
        if button_count == 0:
            button_count = 12

        win = ControllerTestWidget(
            device=device,
            scan_axes=scan_axes,
            button_count=button_count,
            has_hat=has_hat,
            axis_bit_sizes=axis_bit_sizes,
            parent=None,
        )
        win.setStyleSheet(self.styleSheet())
        win.show()
        if not hasattr(self, "_test_windows"):
            self._test_windows: list = []
        self._test_windows.append(win)
        win.destroyed.connect(lambda: self._test_windows.remove(win) if win in self._test_windows else None)

    @Slot()
    def _on_show_history(self) -> None:
        entries = self._service.history_entries()
        self._show_history_dialog(entries)

    # ------------------------------------------------------------------
    # Scan result handlers
    # ------------------------------------------------------------------

    @Slot(object)
    def _on_scan_complete(self, details: DeviceDetails) -> None:
        self._last_scan_details = details
        self._show_details(details)
        self._set_status(
            f"Scan complete: {details.device.name or details.device.device_id}"
        )

    @Slot(str)
    def _on_scan_failed(self, message: str) -> None:
        self._set_status(f"Scan failed: {message}", error=True)

    # ------------------------------------------------------------------
    # Details panel
    # ------------------------------------------------------------------

    def _show_details(self, details: DeviceDetails) -> None:
        self._device_details.show_details(details)
        self._details_container.show()

    def _hide_details(self) -> None:
        self._details_container.hide()
        self._device_details.clear()
        self._last_scan_details = None

    # ------------------------------------------------------------------
    # History dialog
    # ------------------------------------------------------------------

    def _show_history_dialog(self, entries: list) -> None:
        from PySide6.QtWidgets import (
            QDialog, QDialogButtonBox, QHeaderView, QTableWidget, QTableWidgetItem, QVBoxLayout
        )
        dlg = QDialog(self)
        dlg.setWindowTitle("Device History")
        dlg.setMinimumSize(700, 400)
        dlg.setStyleSheet(_STYLE)

        layout = QVBoxLayout(dlg)
        table = QTableWidget()
        table.setColumnCount(5)
        table.setHorizontalHeaderLabels(["Device Name", "Category", "VID", "First Seen", "Last Seen"])
        table.setEditTriggers(QTableWidget.EditTrigger.NoEditTriggers)
        table.setSelectionBehavior(QTableWidget.SelectionBehavior.SelectRows)
        table.verticalHeader().setVisible(False)
        table.setAlternatingRowColors(True)
        table.horizontalHeader().setStretchLastSection(True)
        table.setRowCount(len(entries))

        for row, entry in enumerate(entries):
            for col, key in enumerate(["name", "category", "vendor_id", "first_seen", "last_seen"]):
                val = entry.get(key) or "—"
                table.setItem(row, col, QTableWidgetItem(val))

        layout.addWidget(table)

        btn_row = QHBoxLayout()
        clear_btn = QPushButton("Clear History")
        clear_btn.clicked.connect(lambda: (self._service.clear_history(), dlg.accept()))
        btn_row.addWidget(clear_btn)
        btn_row.addStretch()
        close_btn = QDialogButtonBox(QDialogButtonBox.StandardButton.Close)
        close_btn.rejected.connect(dlg.reject)
        btn_row.addWidget(close_btn)
        layout.addLayout(btn_row)

        dlg.exec()

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _update_status(self) -> None:
        count = self._service.device_count()
        self._set_status(f"{count} device(s) connected")

    def _set_status(self, message: str, error: bool = False) -> None:
        self._status_label.setText(message)
        colour = "#EF5350" if error else "#A6ADC8"
        self._status_label.setStyleSheet(f"color: {colour};")

    # ------------------------------------------------------------------
    # Window lifecycle
    # ------------------------------------------------------------------

    def closeEvent(self, event: QCloseEvent) -> None:  # noqa: N802
        logger.info("Application closing")
        self._service.shutdown()
        event.accept()
