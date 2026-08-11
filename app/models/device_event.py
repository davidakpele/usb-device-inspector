"""Event record for the real-time monitoring log (spec section 13)."""
from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime
from enum import Enum


class DeviceEventType(str, Enum):
    CONNECTED = "USB device connected"
    REMOVED = "USB device removed"
    SCAN_STARTED = "Scan started"
    SCAN_COMPLETED = "Scan completed"
    SCAN_FAILED = "Scan failed"
    ERROR = "Error"
    INFO = "Info"


@dataclass(frozen=True)
class DeviceEvent:
    event_type: DeviceEventType
    message: str
    timestamp: datetime
    device_id: str | None = None

    def formatted(self) -> str:
        ts = self.timestamp.strftime("%H:%M:%S")
        return f"{ts}  {self.message}"