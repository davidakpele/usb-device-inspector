"""Rich, sectioned detail model produced by inspectors (spec section 8-9)."""
from __future__ import annotations

from dataclasses import dataclass, field

from app.models.device_capability import DeviceCapability
from app.models.usb_device import USBDevice

NOT_AVAILABLE = "Not Available"


@dataclass
class DetailField:
    """A single labeled value plus its provenance, for display."""

    label: str
    value: str | None
    source: str = "Unknown"

    def display_value(self) -> str:
        return self.value if self.value else NOT_AVAILABLE


@dataclass
class DetailSection:
    """A named group of fields, e.g. 'General', 'USB Information'."""

    title: str
    fields: list[DetailField] = field(default_factory=list)

    def add(self, label: str, value: str | None, source: str = "Unknown") -> None:
        self.fields.append(DetailField(label=label, value=value, source=source))


@dataclass
class DeviceDetails:
    """Full inspection result for one device: sections + capabilities.

    ``device`` is the normalized record this detail view was built from.
    ``sections`` follow the spec's section 8 ordering (General, USB
    Information, Hardware Identification, Driver, Capabilities) plus any
    device-type-specific section appended by a specialized inspector
    (spec section 9).
    """

    device: USBDevice
    sections: list[DetailSection] = field(default_factory=list)
    capabilities: list[DeviceCapability] = field(default_factory=list)
    warnings: list[str] = field(default_factory=list)

    def get_or_create_section(self, title: str) -> DetailSection:
        for s in self.sections:
            if s.title == title:
                return s
        section = DetailSection(title=title)
        self.sections.append(section)
        return section