"""Scan progress dialog (spec section 10).

Shows a modal dialog with a progress bar and status label while a
background scan is running.  Switches to a "Done / Error" state when the
ScanSignals fire, then lets the user dismiss it.

Usage::

    signals = ScanSignals()
    dialog = ScanDialog(device, signals, parent=self)

    # Connect your result handler BEFORE starting the scan.
    signals.scan_complete.connect(my_handler)

    service.start_scan(device, signals)
    dialog.exec()   # blocks the UI thread but keeps the event loop running
"""
from __future__ import annotations

from PySide6.QtCore import Qt, Slot
from PySide6.QtWidgets import (
    QDialog,
    QDialogButtonBox,
    QLabel,
    QProgressBar,
    QVBoxLayout,
    QWidget,
)

from app.models.device_details import DeviceDetails
from app.models.usb_device import USBDevice
from app.services.scanning_service import ScanSignals


class ScanDialog(QDialog):
    """Modal progress dialog for a device scan (spec section 10).

    The dialog accepts or rejects itself when the scan completes or fails,
    allowing the caller to do ``result = dialog.exec()`` and then inspect
    ``dialog.result_details`` / ``dialog.error_message``.
    """

    def __init__(
        self,
        device: USBDevice,
        signals: ScanSignals,
        parent: QWidget | None = None,
    ) -> None:
        super().__init__(parent)
        self._device = device
        self._signals = signals
        self.result_details: DeviceDetails | None = None
        self.error_message: str | None = None

        self._setup_ui(device)
        self._connect_signals()

    # ------------------------------------------------------------------
    # Construction
    # ------------------------------------------------------------------

    def _setup_ui(self, device: USBDevice) -> None:
        self.setWindowTitle("Scanning Device")
        self.setFixedWidth(420)
        self.setModal(True)
        self.setWindowFlags(
            self.windowFlags() & ~Qt.WindowType.WindowContextHelpButtonHint
        )

        layout = QVBoxLayout(self)
        layout.setSpacing(14)
        layout.setContentsMargins(20, 20, 20, 16)

        device_name = device.name or device.device_id
        title = QLabel(f"Scanning: <b>{device_name}</b>")
        title.setAlignment(Qt.AlignmentFlag.AlignCenter)
        layout.addWidget(title)

        self._status_label = QLabel("Initialising scan…")
        self._status_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self._status_label.setObjectName("scanStatusLabel")
        layout.addWidget(self._status_label)

        self._progress = QProgressBar()
        self._progress.setRange(0, 100)
        self._progress.setValue(0)
        self._progress.setTextVisible(True)
        self._progress.setFixedHeight(22)
        layout.addWidget(self._progress)

        self._warning_label = QLabel()
        self._warning_label.setObjectName("warningLabel")
        self._warning_label.setWordWrap(True)
        self._warning_label.hide()
        layout.addWidget(self._warning_label)

        # Buttons — only "Close" visible, disabled while scanning
        self._buttons = QDialogButtonBox(QDialogButtonBox.StandardButton.Close)
        self._buttons.button(QDialogButtonBox.StandardButton.Close).setEnabled(False)
        self._buttons.rejected.connect(self.reject)
        layout.addWidget(self._buttons)

    def _connect_signals(self) -> None:
        self._signals.scan_progress.connect(self._on_progress)
        self._signals.scan_complete.connect(self._on_complete)
        self._signals.scan_failed.connect(self._on_failed)

    # ------------------------------------------------------------------
    # Slots
    # ------------------------------------------------------------------

    @Slot(int)
    def _on_progress(self, value: int) -> None:
        self._progress.setValue(value)
        labels = {
            10: "Starting scan…",
            50: "Inspecting device…",
            90: "Finalising results…",
            100: "Scan complete",
        }
        if value in labels:
            self._status_label.setText(labels[value])

    @Slot(object)
    def _on_complete(self, details: DeviceDetails) -> None:
        self.result_details = details
        self._progress.setValue(100)
        self._status_label.setText("✓  Scan Complete")
        self._status_label.setStyleSheet("color: #4CAF50; font-weight: bold;")

        if details.warnings:
            self._warning_label.setText("⚠  " + "\n⚠  ".join(details.warnings))
            self._warning_label.show()

        close_btn = self._buttons.button(QDialogButtonBox.StandardButton.Close)
        close_btn.setEnabled(True)
        close_btn.setDefault(True)
        close_btn.setFocus()

    @Slot(str)
    def _on_failed(self, message: str) -> None:
        self.error_message = message
        self._progress.setValue(0)
        self._status_label.setText("✗  Scan Failed")
        self._status_label.setStyleSheet("color: #EF5350; font-weight: bold;")
        self._warning_label.setText(message)
        self._warning_label.show()

        close_btn = self._buttons.button(QDialogButtonBox.StandardButton.Close)
        close_btn.setEnabled(True)
        close_btn.setDefault(True)
        close_btn.setFocus()
