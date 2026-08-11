"""Base contract for device-specific inspectors (spec section 9)."""
from __future__ import annotations

from abc import ABC, abstractmethod

from app.models.device_capability import DeviceCapability
from app.models.device_details import DeviceDetails
from app.models.usb_device import FieldSource, USBDevice
from app.usb.usb_constants import USB_CLASS_NAMES


class BaseInspector(ABC):
    """One inspector per device category (spec section 9's "new inspectors
    must be addable later" requirement). Register new subclasses in
    ``core/device_scanner.py``'s ``_INSPECTOR_REGISTRY`` — nothing else
    needs to change.
    """

    #: DeviceCategory value(s) this inspector applies to.
    categories: tuple[str, ...] = ()

    @abstractmethod
    def inspect(self, device: USBDevice) -> DeviceDetails:
        """Build a full DeviceDetails for the given device.

        Must never raise for missing/absent fields — always fill sections
        with "Not Available"/"Not Reported by Device" instead. Only raise
        for truly exceptional conditions (e.g. device vanished mid-scan);
        the scanning service will catch and report those gracefully.
        """
        raise NotImplementedError

    # --- shared helpers available to all subclasses -----------------
    def build_general_section(self, details: DeviceDetails, device: USBDevice) -> None:
        section = details.get_or_create_section("General")
        section.add("Name", device.name)
        section.add("Manufacturer", device.manufacturer)
        section.add("Description", device.description)
        section.add("Category", device.category, source=device.category_source.value)
        section.add("Status", device.status.value)
        section.add("Connection State", "Connected" if device.connected else "Disconnected")

    def build_usb_information_section(self, details: DeviceDetails, device: USBDevice) -> None:
        section = details.get_or_create_section("USB Information")
        section.add("Vendor ID", device.vendor_id, source="Directly Reported" if device.vendor_id else "Unknown")
        section.add("Product ID", device.product_id, source="Directly Reported" if device.product_id else "Unknown")
        section.add("USB Version", device.usb_version)
        class_name = USB_CLASS_NAMES.get(int(device.device_class, 16)) if _is_hex(device.device_class) else None
        section.add("Device Class", class_name or device.device_class)
        section.add("Subclass", device.device_subclass)
        section.add("Protocol", device.device_protocol)

    def build_hardware_section(self, details: DeviceDetails, device: USBDevice) -> None:
        section = details.get_or_create_section("Hardware Identification")
        section.add("Hardware ID", device.hardware_id)
        section.add("Device Instance ID", device.instance_id)
        section.add(
            "Compatible IDs",
            ", ".join(device.compatible_ids) if device.compatible_ids else None,
        )
        section.add("Serial Number", device.serial_number, source=device.serial_source.value)
        section.add("Parent Device", device.parent_instance_id)

    def build_driver_section(self, details: DeviceDetails, device: USBDevice) -> None:
        section = details.get_or_create_section("Driver")
        section.add("Driver Name", device.driver_name)
        section.add("Driver Provider", device.driver_provider)
        section.add("Driver Version", device.driver_version)
        section.add("Driver Date", device.driver_date)
        section.add("Driver Status", device.status.value)

    def add_capability_if(
        self, details: DeviceDetails, condition: bool, label: str, evidence: str,
        source: FieldSource = FieldSource.DETECTED,
    ) -> None:
        """Only append a capability when ``condition`` is backed by real evidence.

        This is the single choke point that enforces spec section 8's rule:
        "Do not claim a capability simply because the device category
        suggests it" — callers must pass a concrete boolean derived from
        actual reported data, plus a human-readable justification string.
        """
        if condition:
            details.capabilities.append(
                DeviceCapability(label=label, source=source, evidence=evidence)
            )


def _is_hex(value: str | None) -> bool:
    if not value:
        return False
    try:
        int(value, 16)
        return True
    except ValueError:
        return False