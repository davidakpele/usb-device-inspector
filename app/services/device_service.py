"""Top-level orchestration service (spec sections 4, 13, 16).

``DeviceService`` wires together:
  * ``DeviceManager``     – in-memory device registry
  * ``DeviceMonitorThread`` – WMI event listener
  * ``DeviceScanner``     – per-device detail inspection (sync, used directly for refresh)
  * ``HistoryService``    – persistent device history
  * ``ScanningService``   – async scan submission

It is a QObject so it can own Qt signals and be the natural mediator between
the background WMI thread and the PySide6 UI (spec section 20). All WMI work
runs on the monitor thread or the thread-pool; the UI never touches WMI.

Signal inventory (consumed by MainWindow / DeviceListWidget / EventLogWidget):
  devices_changed  – full device list changed; UI should reload the list
  device_connected – a new device arrived; carries USBDevice
  device_removed   – a device departed; carries device_id string
  event_logged     – a new DeviceEvent was created for the event log
  monitor_error    – non-fatal monitor error message string
"""
from __future__ import annotations

from datetime import datetime

from PySide6.QtCore import QObject, Signal

from app.core.device_detector import normalize
from app.core.device_manager import DeviceManager
from app.core.device_monitor import DeviceMonitorThread
from app.core.device_scanner import DeviceScanner
from app.models.device_event import DeviceEvent, DeviceEventType
from app.models.usb_device import USBDevice
from app.services.history_service import HistoryService
from app.services.scanning_service import ScanSignals, ScanningService
from app.usb.usb_enumerator import enumerate_usb_descriptors
from app.utils.logger import get_logger

logger = get_logger(__name__)


class DeviceService(QObject):
    """Central service: owns all backend objects, exposes Qt signals to the UI."""

    devices_changed = Signal()                    # device list changed
    device_connected = Signal(object)             # USBDevice
    device_removed = Signal(str)                  # device_id
    event_logged = Signal(object)                 # DeviceEvent
    monitor_error = Signal(str)                   # error message

    def __init__(self, parent: QObject | None = None) -> None:
        super().__init__(parent)
        self._manager = DeviceManager()
        self._history = HistoryService()
        self._scanner = DeviceScanner()
        self._scanning = ScanningService()
        self._monitor: DeviceMonitorThread | None = None

    # ------------------------------------------------------------------
    # Startup / shutdown
    # ------------------------------------------------------------------

    def initialize(self) -> None:
        """Run initial USB enumeration then start the real-time monitor.

        Called once from the main window after the UI is shown so the app
        appears immediately without blocking on WMI.
        """
        logger.info("Application startup: beginning initial USB enumeration")
        self._emit_event(DeviceEventType.SCAN_STARTED, "Initial USB enumeration started")
        self.refresh()
        self._start_monitor()

    def shutdown(self) -> None:
        """Stop the background monitor thread cleanly."""
        if self._monitor and self._monitor.isRunning():
            logger.info("Stopping device monitor thread")
            self._monitor.stop()

    # ------------------------------------------------------------------
    # Device list
    # ------------------------------------------------------------------

    def all_devices(self) -> list[USBDevice]:
        return self._manager.all_devices()

    def get_device(self, device_id: str) -> USBDevice | None:
        return self._manager.get(device_id)

    def device_count(self) -> int:
        return self._manager.count()

    # ------------------------------------------------------------------
    # Refresh (re-enumerate)
    # ------------------------------------------------------------------

    def refresh(self) -> None:
        """Re-enumerate all USB devices and update the registry."""
        logger.info("Refreshing USB device list")
        try:
            descriptors = enumerate_usb_descriptors()
            devices = [normalize(d) for d in descriptors]
            connected, removed = self._manager.apply_snapshot(devices)

            for device in connected:
                self._history.record(device)
                self._emit_event(
                    DeviceEventType.CONNECTED,
                    f"{device.name or device.device_id} detected",
                    device_id=device.device_id,
                )

            for device in removed:
                self._emit_event(
                    DeviceEventType.REMOVED,
                    f"{device.name or device.device_id} removed",
                    device_id=device.device_id,
                )

            self._emit_event(
                DeviceEventType.SCAN_COMPLETED,
                f"Enumeration complete — {self._manager.count()} device(s) found",
            )
            self.devices_changed.emit()

        except Exception as exc:  # noqa: BLE001
            logger.exception("Refresh failed: %s", exc)
            self._emit_event(DeviceEventType.SCAN_FAILED, f"Enumeration error: {exc}")

    # ------------------------------------------------------------------
    # Scanning
    # ------------------------------------------------------------------

    def start_scan(self, device: USBDevice, signals: ScanSignals) -> bool:
        """Submit an async scan for *device*. Returns False if already scanning."""
        return self._scanning.start_scan(device, signals)

    def is_scanning(self, device_id: str) -> bool:
        return self._scanning.is_scanning(device_id)

    # ------------------------------------------------------------------
    # History
    # ------------------------------------------------------------------

    def history_entries(self):
        return self._history.all_entries()

    def clear_history(self) -> None:
        self._history.clear()
        self._emit_event(DeviceEventType.INFO, "Device history cleared")

    # ------------------------------------------------------------------
    # Monitor thread
    # ------------------------------------------------------------------

    def _start_monitor(self) -> None:
        self._monitor = DeviceMonitorThread(parent=self)
        self._monitor.device_arrived.connect(self._on_device_arrived)
        self._monitor.device_departed.connect(self._on_device_departed)
        self._monitor.monitor_error.connect(self._on_monitor_error)
        self._monitor.start()
        logger.info("Device monitor thread started")

    def _on_device_arrived(self, device_id: str) -> None:
        """Called on UI thread (via Qt signal), so safe to update UI state."""
        logger.info("USB DEVICE CONNECTED (monitor): %s", device_id)
        # Re-enumerate to get full descriptor for the new device.
        descriptors = enumerate_usb_descriptors()
        devices = [normalize(d) for d in descriptors]
        connected, removed = self._manager.apply_snapshot(devices)

        for device in connected:
            self._history.record(device)
            self._emit_event(
                DeviceEventType.CONNECTED,
                f"{device.name or device.device_id} detected",
                device_id=device.device_id,
            )
            self.device_connected.emit(device)

        for device in removed:
            self._emit_event(
                DeviceEventType.REMOVED,
                f"{device.name or device.device_id} removed",
                device_id=device.device_id,
            )
            self.device_removed.emit(device.device_id)

        self.devices_changed.emit()

    def _on_device_departed(self, device_id: str) -> None:
        """Handle a WMI deletion event."""
        logger.info("USB DEVICE REMOVED (monitor): %s", device_id)
        removed_device = self._manager.mark_removed(device_id)
        if removed_device:
            self._emit_event(
                DeviceEventType.REMOVED,
                f"{removed_device.name or device_id} removed",
                device_id=device_id,
            )
            self.device_removed.emit(device_id)
            self.devices_changed.emit()
        else:
            # Device wasn't in our registry - trigger a full refresh to sync.
            self._on_device_arrived(device_id)

    def _on_monitor_error(self, message: str) -> None:
        logger.error("Device monitor error: %s", message)
        self._emit_event(DeviceEventType.ERROR, f"Monitor error: {message}")
        self.monitor_error.emit(message)

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _emit_event(
        self,
        event_type: DeviceEventType,
        message: str,
        device_id: str | None = None,
    ) -> None:
        event = DeviceEvent(
            event_type=event_type,
            message=message,
            timestamp=datetime.now(),
            device_id=device_id,
        )
        logger.debug("Event: %s", event.formatted())
        self.event_logged.emit(event)
