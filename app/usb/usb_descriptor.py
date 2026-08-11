"""Raw, unopinionated snapshot of what Windows reported for a PnP entity.

This intentionally mirrors WMI's Win32_PnPEntity field names rather than our
own domain model (see models/usb_device.py). Keeping the raw snapshot
separate from the normalized USBDevice lets us:
  * re-derive/re-classify without re-querying WMI
  * clearly track "what did the OS actually say" vs "what we inferred"
"""
from __future__ import annotations

from dataclasses import dataclass, field


@dataclass
class RawPnPDescriptor:
    """Verbatim fields as returned by WMI Win32_PnPEntity (may be None)."""

    device_id: str  # PNPDeviceID / DeviceID - unique instance identifier
    name: str | None = None
    description: str | None = None
    manufacturer: str | None = None
    pnp_class: str | None = None  # e.g. "USB", "HIDClass", "DiskDrive"
    class_guid: str | None = None
    hardware_ids: list[str] = field(default_factory=list)
    compatible_ids: list[str] = field(default_factory=list)
    service: str | None = None  # driver service name
    status: str | None = None  # ConfigManagerErrorCode-derived status
    present: bool = True

    @property
    def primary_hardware_id(self) -> str | None:
        return self.hardware_ids[0] if self.hardware_ids else None