"""Asynchronous scan execution service (spec sections 10, 20).

``ScanWorker`` is a QRunnable that runs ``DeviceScanner.scan()`` on a
QThreadPool thread so the UI stays responsive during potentially long
driver queries. It communicates back to the UI via a ``ScanSignals``
QObject (QRunnable itself cannot have signals).

Usage::

    signals = ScanSignals()
    signals.scan_complete.connect(my_slot)
    signals.scan_failed.connect(my_error_slot)

    worker = ScanWorker(device, signals)
    QThreadPool.globalInstance().start(worker)

Progress reporting: we emit ``scan_progress`` at discrete milestones
(10 % = started, 50 % = inspector running, 90 % = finalizing, 100 % = done)
to drive the progress bar in ``ScanDialog`` (spec section 10).
"""
from __future__ import annotations

from PySide6.QtCore import QObject, QRunnable, Signal

from app.core.device_scanner import DeviceScanner
from app.models.device_details import DeviceDetails
from app.models.usb_device import USBDevice
from app.utils.logger import get_logger

logger = get_logger(__name__)


class ScanSignals(QObject):
    """Signals emitted by ScanWorker (must live on a QObject, not QRunnable)."""

    # int: 0-100 percentage complete
    scan_progress = Signal(int)
    # DeviceDetails: full inspection result
    scan_complete = Signal(object)
    # str: human-readable error message
    scan_failed = Signal(str)


class ScanWorker(QRunnable):
    """Runs a device scan on a background thread pool thread.

    The scan is entirely read-only; it never modifies the device or its
    driver state (spec section 19).
    """

    def __init__(self, device: USBDevice, signals: ScanSignals) -> None:
        super().__init__()
        self._device = device
        self._signals = signals
        self.setAutoDelete(True)

    def run(self) -> None:  # QRunnable override
        device = self._device
        signals = self._signals
        logger.info("ScanWorker started for %s", device.device_id)

        try:
            signals.scan_progress.emit(10)

            scanner = DeviceScanner()
            signals.scan_progress.emit(50)

            details: DeviceDetails = scanner.scan(device)
            signals.scan_progress.emit(90)

            signals.scan_progress.emit(100)
            signals.scan_complete.emit(details)

        except Exception as exc:  # noqa: BLE001
            logger.exception("ScanWorker unhandled exception for %s: %s", device.device_id, exc)
            signals.scan_failed.emit(
                f"Scan failed: {exc}\n"
                "The device may have been disconnected or may not support inspection."
            )


class ScanningService:
    """Thin facade that submits ScanWorker jobs to QThreadPool.

    Maintains one active worker reference per device_id so callers can
    detect if a scan is already in progress (and avoid double-submitting).
    """

    def __init__(self) -> None:
        from PySide6.QtCore import QThreadPool
        self._pool = QThreadPool.globalInstance()
        self._active: set[str] = set()

    def is_scanning(self, device_id: str) -> bool:
        return device_id in self._active

    def start_scan(self, device: USBDevice, signals: ScanSignals) -> bool:
        """Submit a scan for *device*. Returns False if already in progress."""
        if device.device_id in self._active:
            logger.debug("Scan already in progress for %s", device.device_id)
            return False

        self._active.add(device.device_id)

        # Wrap completion/failure signals to remove from active set.
        original_complete = signals.scan_complete
        original_failed = signals.scan_failed

        def on_done(details: DeviceDetails) -> None:
            self._active.discard(device.device_id)
            original_complete.emit(details)  # type: ignore[attr-defined]

        def on_fail(msg: str) -> None:
            self._active.discard(device.device_id)
            original_failed.emit(msg)  # type: ignore[attr-defined]

        signals.scan_complete.connect(lambda d: self._active.discard(device.device_id))
        signals.scan_failed.connect(lambda m: self._active.discard(device.device_id))

        worker = ScanWorker(device, signals)
        self._pool.start(worker)
        logger.info("Scan submitted for %s", device.device_id)
        return True
