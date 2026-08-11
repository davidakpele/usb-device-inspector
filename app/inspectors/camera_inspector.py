"""Inspector for USB cameras and imaging devices (spec section 9 - Cameras).

On Windows, cameras and scanners present under PNPClass "Image" or "Camera".
Detailed imaging parameters (resolution, frame-rate, etc.) require the
Windows Media Foundation / DirectShow APIs, which are out of scope for this
read-only inspection tool. We expose everything Windows makes available via
WMI/PnP and clearly mark other fields as Not Available.
"""
from __future__ import annotations

from app.inspectors.base_inspector import BaseInspector
from app.models.device_details import DeviceDetails
from app.models.usb_device import FieldSource, USBDevice
from app.usb.usb_constants import DeviceCategory
from app.utils.logger import get_logger

logger = get_logger(__name__)


class CameraInspector(BaseInspector):
    categories = (DeviceCategory.CAMERA.value,)

    def inspect(self, device: USBDevice) -> DeviceDetails:
        details = DeviceDetails(device=device)
        self.build_general_section(details, device)
        self.build_usb_information_section(details, device)
        self.build_hardware_section(details, device)
        self.build_driver_section(details, device)

        section = details.get_or_create_section("Camera")
        section.add("Camera Name", device.name, source="Directly Reported" if device.name else "Unknown")
        section.add("Manufacturer", device.manufacturer, source="Directly Reported" if device.manufacturer else "Unknown")
        section.add("Vendor ID", device.vendor_id, source="Directly Reported" if device.vendor_id else "Unknown")
        section.add("Product ID", device.product_id, source="Directly Reported" if device.product_id else "Unknown")
        section.add("Device Class", device.device_class, source="Directly Reported" if device.device_class else "Unknown")

        # Resolution, frame-rate, and format information require DirectShow /
        # Windows Media Foundation enumeration which is beyond read-only PnP
        # inspection scope — we report them as Not Available rather than guess.
        section.add("Resolution", None)
        section.add("Frame Rate", None)
        section.add("Video Format", None)

        details.warnings.append(
            "Detailed imaging parameters (resolution, frame rate, video format) "
            "are not available through the Windows PnP layer. "
            "They require DirectShow/Media Foundation enumeration."
        )

        # Determine if this is a still-image or video capture device.
        device_type = self._determine_device_type(device)
        section.add("Device Type", device_type, source="Detected")

        # Capabilities
        self.add_capability_if(
            details, True, "USB Camera / Imaging Device",
            evidence="Category classified as Camera (PNPClass Image or Camera)",
        )
        self.add_capability_if(
            details,
            device_type == "Webcam / Video Camera",
            "Video Capture",
            evidence="Device class or name indicates video capture",
        )
        self.add_capability_if(
            details,
            device_type == "Still Image / Scanner",
            "Still Image Capture",
            evidence="Device class indicates still image (WIA)",
        )

        return details

    @staticmethod
    def _determine_device_type(device: USBDevice) -> str:
        name_lower = (device.name or "").lower()
        desc_lower = (device.description or "").lower()
        combined = name_lower + " " + desc_lower
        if any(k in combined for k in ("scanner", "still", "wia", "flatbed")):
            return "Still Image / Scanner"
        if any(k in combined for k in ("webcam", "camera", "video", "capture", "cam")):
            return "Webcam / Video Camera"
        # USB class 0x0E = Video, 0x06 = Image (still)
        class_val = (device.device_class or "").upper()
        if class_val in ("0E", "14"):
            return "Webcam / Video Camera"
        if class_val == "06":
            return "Still Image / Scanner"
        return "Camera / Imaging Device"
