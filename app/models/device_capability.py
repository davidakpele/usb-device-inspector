"""Represents a capability we can *prove* a device has (spec section 8)."""
from __future__ import annotations

from dataclasses import dataclass

from app.models.usb_device import FieldSource


@dataclass(frozen=True)
class DeviceCapability:
    """A single, evidenced capability flag.

    ``evidence`` should name the concrete field/heuristic that justified the
    flag (e.g. "device_class == HID", "PNPClass == DiskDrive") so capability
    claims stay auditable and we never assert a capability "because the
    category suggests it" (explicitly disallowed by the spec).
    """

    label: str
    source: FieldSource
    evidence: str

    def __str__(self) -> str:  # pragma: no cover - trivial
        return f"✓ {self.label}"