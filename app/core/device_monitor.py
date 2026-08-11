"""Real-time USB connect/disconnect monitoring (spec sections 4 and 13).

Uses WMI's event-notification query mechanism
(``SELECT * FROM __InstanceCreationEvent WITHIN n WHERE TargetInstance
ISA 'Win32_PnPEntity'``) rather than polling ``Win32_PnPEntity`` in a loop.

This *is* still technically a short-interval WMI subscription poll under
the hood (WMI's event provider re-checks every ``WITHIN`` seconds), but it
is the standard, documented Windows event-notification mechanism for PnP
changes reachable from Python without writing a native message-window
handler for ``WM_DEVICECHANGE`` via ``pywin32``'s ``win32gui``. A
``WM_DEVICECHANGE`` hidden-window listener is noted as a future
enhancement (lower latency, zero polling interval) but WMI eventing is the
pragmatic, reliable default given the cross-device-class requirement.

Runs on a dedicated QThread so the blocking WMI ``NextEvent()`` call never
touches the UI thread (spec section 20).
"""
from __future__ import annotations

from PySide6.QtCore import QThread, Signal

from app.usb.usb_enumerator import WMIUnavailableError, _connect
from app.utils.logger import get_logger

logger = get_logger(__name__)

_POLL_WITHIN_SECONDS = 2
_EVENT_TIMEOUT_MS = 3000


class DeviceMonitorThread(QThread):
    """Watches for USB PnP creation/deletion events and emits Qt signals.

    Signals carry only the raw PNPDeviceID string; the caller (DeviceService)
    is responsible for re-enumerating/normalizing, since a single WMI event
    does not carry the full descriptor reliably for every device class.
    """

    device_arrived = Signal(str)  # PNPDeviceID
    device_departed = Signal(str)  # PNPDeviceID
    monitor_error = Signal(str)

    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        self._running = False

    def run(self) -> None:  # noqa: D102 - QThread override
        self._running = True
        try:
            conn = _connect()
        except WMIUnavailableError as exc:
            logger.error("Device monitor could not start: %s", exc)
            self.monitor_error.emit(str(exc))
            return

        try:
            creation_watcher = conn.Win32_PnPEntity.watch_for(
                notification_type="creation", delay_secs=_POLL_WITHIN_SECONDS
            )
            deletion_watcher = conn.Win32_PnPEntity.watch_for(
                notification_type="deletion", delay_secs=_POLL_WITHIN_SECONDS
            )
        except Exception as exc:  # noqa: BLE001
            logger.error("Failed to set up WMI watchers: %s", exc)
            self.monitor_error.emit(f"Failed to set up device watchers: {exc}")
            return

        logger.info("USB device monitor started")
        while self._running:
            self._poll_watcher(creation_watcher, self.device_arrived)
            self._poll_watcher(deletion_watcher, self.device_departed)
        logger.info("USB device monitor stopped")

    def _poll_watcher(self, watcher, signal: Signal) -> None:
        try:
            event = watcher(timeout_ms=_EVENT_TIMEOUT_MS)
        except Exception as exc:  # noqa: BLE001
            # wmi raises x_wmi_timed_out on no-event-within-timeout; that is
            # the expected, common case, not an error.
            if type(exc).__name__ != "x_wmi_timed_out":
                logger.debug("Watcher poll error (non-fatal): %s", exc)
            return

        device_id = getattr(event, "PNPDeviceID", None) or getattr(event, "DeviceID", None)
        if device_id and str(device_id).upper().startswith(("USB\\", "USBSTOR\\")):
            signal.emit(device_id)

    def stop(self) -> None:
        self._running = False
        self.wait(_EVENT_TIMEOUT_MS + 500)