"""Normalized internal representation of a USB device (spec section 17)."""
from __future__ import annotations

from dataclasses import dataclass, field
from datetime import datetime
from enum import Enum

NOT_AVAILABLE = "Not Available"
NOT_REPORTED = "Not Reported by Device"


class FieldSource(str, Enum):
    """Provenance of a piece of information (spec section 23).

    Every non-trivial displayed fact should be traceable to one of these so
    the UI can show "Directly Reported / Detected / Derived / Unknown".
    """

    DIRECTLY_REPORTED = "Directly Reported"  # verbatim from USB descriptor / WMI
    DETECTED = "Detected"  # computed deterministically from reported data
    DERIVED = "Derived"  # heuristic inference (e.g. VID-based dev-board guess)
    UNKNOWN = "Unknown"


class ConnectionStatus(str, Enum):
    CONNECTED = "Connected"
    DISCONNECTED = "Disconnected"
    ERROR = "Error"
    UNKNOWN = "Unknown"


@dataclass
class USBDevice:
    """Normalized USB device record used throughout the application."""

    device_id: str  # Windows PNPDeviceID - stable unique key while present
    name: str | None = None
    manufacturer: str | None = None
    description: str | None = None
    vendor_id: str | None = None
    product_id: str | None = None
    serial_number: str | None = None
    serial_source: FieldSource = FieldSource.UNKNOWN
    device_class: str | None = None
    device_subclass: str | None = None
    device_protocol: str | None = None
    usb_version: str | None = None
    hardware_id: str | None = None
    hardware_ids: list[str] = field(default_factory=list)
    compatible_ids: list[str] = field(default_factory=list)
    instance_id: str | None = None
    parent_instance_id: str | None = None
    driver_name: str | None = None
    driver_provider: str | None = None
    driver_version: str | None = None
    driver_date: str | None = None
    status: ConnectionStatus = ConnectionStatus.UNKNOWN
    category: str = "Unknown"
    category_source: FieldSource = FieldSource.UNKNOWN
    connected: bool = True
    first_seen: datetime | None = None
    last_seen: datetime | None = None

    def display(self, value: str | None) -> str:
        """Render an optional field for the UI, never fabricating data."""
        return value if value else NOT_AVAILABLE

    def to_summary_dict(self) -> dict:
        """Compact dict for list views / logging (not full detail)."""
        return {
            "device_id": self.device_id,
            "name": self.display(self.name),
            "category": self.category,
            "vendor_id": self.display(self.vendor_id),
            "product_id": self.display(self.product_id),
            "status": self.status.value,
        }