"""Central, thread-safe registry of currently known USB devices.

DeviceManager owns the authoritative in-memory device list. Both the
initial-enumeration path and the real-time monitor path funnel through
``apply_snapshot`` / ``mark_connected`` / ``mark_removed`` so there is a
single place that decides what counts as a "connect" or "remove" event
(spec section 4).
"""
from __future__ import annotations

import threading
from datetime import datetime

from app.models.usb_device import ConnectionStatus, USBDevice
from app.utils.logger import get_logger

logger = get_logger(__name__)


class DeviceManager:
    """Not a Qt object on purpose — kept UI-framework agnostic (spec section 16)."""

    def __init__(self) -> None:
        self._devices: dict[str, USBDevice] = {}
        self._lock = threading.RLock()

    def apply_snapshot(
        self, devices: list[USBDevice]
    ) -> tuple[list[USBDevice], list[USBDevice]]:
        """Replace the full device set (used on startup / manual refresh).

        Returns (newly_connected, newly_removed) relative to the previous
        state, so callers can emit the appropriate events without
        duplicating diff logic.
        """
        now = datetime.now()
        with self._lock:
            previous_ids = set(self._devices.keys())
            incoming_ids = {d.device_id for d in devices}

            newly_connected: list[USBDevice] = []
            for device in devices:
                existing = self._devices.get(device.device_id)
                if existing is None:
                    device.first_seen = now
                    device.last_seen = now
                    self._devices[device.device_id] = device
                    newly_connected.append(device)
                else:
                    existing.last_seen = now
                    existing.status = ConnectionStatus.CONNECTED
                    existing.connected = True

            removed_ids = previous_ids - incoming_ids
            newly_removed: list[USBDevice] = []
            for device_id in removed_ids:
                device = self._devices[device_id]
                device.status = ConnectionStatus.DISCONNECTED
                device.connected = False
                newly_removed.append(device)
                # Keep the record (for history) but drop it from the active set.
                del self._devices[device_id]

            return newly_connected, newly_removed

    def mark_connected(self, device: USBDevice) -> None:
        with self._lock:
            now = datetime.now()
            device.first_seen = device.first_seen or now
            device.last_seen = now
            device.status = ConnectionStatus.CONNECTED
            device.connected = True
            self._devices[device.device_id] = device
        logger.info("USB DEVICE CONNECTED: %s (%s)", device.name or device.device_id, device.category)

    def mark_removed(self, device_id: str) -> USBDevice | None:
        with self._lock:
            device = self._devices.pop(device_id, None)
        if device:
            device.status = ConnectionStatus.DISCONNECTED
            device.connected = False
            logger.info("USB DEVICE REMOVED: %s", device.name or device.device_id)
        return device

    def get(self, device_id: str) -> USBDevice | None:
        with self._lock:
            return self._devices.get(device_id)

    def all_devices(self) -> list[USBDevice]:
        with self._lock:
            return list(self._devices.values())

    def count(self) -> int:
        with self._lock:
            return len(self._devices)