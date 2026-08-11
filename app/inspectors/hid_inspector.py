"""Inspector for HID devices - keyboards, mice, and generic HID peripherals.

HID usage page/usage require querying the HID report descriptor, which
Windows does not expose through WMI. We use the optional ``hid`` package
(hidapi bindings) when available and fall back to "Not Available" rather
than failing the whole inspection when it is not installed or the device
denies the open (exclusive-access devices like some mice/keyboards commonly
do — this is expected, not an error).
"""
from __future__ import annotations

from app.inspectors.base_inspector import BaseInspector
from app.models.device_details import DeviceDetails
from app.models.usb_device import USBDevice
from app.usb.usb_constants import DeviceCategory
from app.utils.logger import get_logger

logger = get_logger(__name__)


class HIDInspector(BaseInspector):
    categories = (DeviceCategory.INPUT_DEVICE.value,)

    def inspect(self, device: USBDevice) -> DeviceDetails:
        details = DeviceDetails(device=device)
        self.build_general_section(details, device)
        self.build_usb_information_section(details, device)
        self.build_hardware_section(details, device)
        self.build_driver_section(details, device)

        section = details.get_or_create_section("HID")
        section.add("HID Device Name", device.name)
        section.add("Manufacturer", device.manufacturer)

        usage_page, usage = self._read_hid_usage(device)
        section.add("HID Usage Page", usage_page, source="Directly Reported" if usage_page else "Unknown")
        section.add("HID Usage", usage, source="Directly Reported" if usage else "Unknown")
        if usage_page is None:
            details.warnings.append(
                "Some device information could not be retrieved. "
                "The device may not expose this information to Windows, "
                "or it may be held exclusively by another driver (common for "
                "standard keyboards/mice)."
            )

        name_upper = (device.name or "").upper()
        is_keyboard = "KEYBOARD" in name_upper or usage == "0006"
        is_mouse = "MOUSE" in name_upper or usage == "0002"

        section.add(
            "Input Type",
            "Keyboard" if is_keyboard else "Mouse" if is_mouse else "Generic HID",
            source="Detected",
        )

        self.add_capability_if(details, True, "HID Input", evidence="Category classified as Input Device (HID)")
        self.add_capability_if(
            details, is_keyboard, "Keyboard", evidence="Name/usage indicates keyboard"
        )
        self.add_capability_if(details, is_mouse, "Mouse", evidence="Name/usage indicates mouse")
        return details

    @staticmethod
    def _read_hid_usage(device: USBDevice) -> tuple[str | None, str | None]:
        if not device.vendor_id or not device.product_id:
            return None, None
        try:
            import hid  # type: ignore  # optional dependency (hidapi)
        except ImportError:
            logger.debug("hidapi not installed; skipping HID usage page lookup")
            return None, None

        try:
            vid = int(device.vendor_id, 16)
            pid = int(device.product_id, 16)
            for info in hid.enumerate(vid, pid):
                usage_page = info.get("usage_page")
                usage = info.get("usage")
                if usage_page is not None:
                    return f"{usage_page:#06x}", f"{usage:#06x}" if usage is not None else None
        except Exception as exc:  # noqa: BLE001 - hidapi errors vary by platform
            logger.debug("HID usage lookup failed for %s: %s", device.device_id, exc)
        return None, None