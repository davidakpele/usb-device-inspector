"""USB class/subclass constants and Windows device-class mapping tables.

Source: USB-IF "Defined Class Codes" (https://www.usb.org/defined-class-codes).
These are used only to *classify* devices, never to fabricate information
that the device/OS did not actually report.
"""
from __future__ import annotations

from enum import Enum


class USBClassCode(Enum):
    """Standard USB base class codes (bDeviceClass / bInterfaceClass)."""

    DEVICE_LEVEL = 0x00
    AUDIO = 0x01
    CDC_COMM = 0x02
    HID = 0x03
    PHYSICAL = 0x05
    IMAGE = 0x06
    PRINTER = 0x07
    MASS_STORAGE = 0x08
    HUB = 0x09
    CDC_DATA = 0x0A
    SMART_CARD = 0x0B
    CONTENT_SECURITY = 0x0D
    VIDEO = 0x0E
    PERSONAL_HEALTHCARE = 0x0F
    AUDIO_VIDEO = 0x10
    BILLBOARD = 0x11
    TYPE_C_BRIDGE = 0x12
    DIAGNOSTIC = 0xDC
    WIRELESS_CONTROLLER = 0xE0
    MISCELLANEOUS = 0xEF
    APPLICATION_SPECIFIC = 0xFE
    VENDOR_SPECIFIC = 0xFF


# Human readable names for the class codes above.
USB_CLASS_NAMES: dict[int, str] = {
    USBClassCode.DEVICE_LEVEL.value: "Device (class defined at interface level)",
    USBClassCode.AUDIO.value: "Audio",
    USBClassCode.CDC_COMM.value: "Communications and CDC Control",
    USBClassCode.HID.value: "Human Interface Device",
    USBClassCode.PHYSICAL.value: "Physical",
    USBClassCode.IMAGE.value: "Image / Camera",
    USBClassCode.PRINTER.value: "Printer",
    USBClassCode.MASS_STORAGE.value: "Mass Storage",
    USBClassCode.HUB.value: "Hub",
    USBClassCode.CDC_DATA.value: "CDC Data",
    USBClassCode.SMART_CARD.value: "Smart Card",
    USBClassCode.CONTENT_SECURITY.value: "Content Security",
    USBClassCode.VIDEO.value: "Video",
    USBClassCode.PERSONAL_HEALTHCARE.value: "Personal Healthcare",
    USBClassCode.AUDIO_VIDEO.value: "Audio/Video Devices",
    USBClassCode.BILLBOARD.value: "Billboard Device",
    USBClassCode.TYPE_C_BRIDGE.value: "USB Type-C Bridge",
    USBClassCode.DIAGNOSTIC.value: "Diagnostic Device",
    USBClassCode.WIRELESS_CONTROLLER.value: "Wireless Controller",
    USBClassCode.MISCELLANEOUS.value: "Miscellaneous",
    USBClassCode.APPLICATION_SPECIFIC.value: "Application Specific",
    USBClassCode.VENDOR_SPECIFIC.value: "Vendor Specific",
}


class DeviceCategory(str, Enum):
    """High level, user-facing device categories (spec section 6)."""

    STORAGE = "Storage"
    INPUT_DEVICE = "Input Device"
    GAME_CONTROLLER = "Game Controller"
    CAMERA = "Camera"
    AUDIO = "Audio"
    PRINTER = "Printer"
    NETWORK_ADAPTER = "Network Adapter"
    SERIAL_DEVICE = "Serial Device"
    DEVELOPMENT_BOARD = "Development Board"
    USB_HUB = "USB Hub"
    MOBILE_DEVICE = "Mobile Device"
    SECURITY_DEVICE = "Security Device"
    UNKNOWN = "Unknown"
    OTHER = "Other"


# Windows PNPClass values (from Win32_PnPEntity.PNPClass / ClassGuid) that map
# fairly reliably to a category. Used as a *secondary* signal alongside the
# USB interface class when available.
PNP_CLASS_TO_CATEGORY: dict[str, DeviceCategory] = {
    "DiskDrive": DeviceCategory.STORAGE,
    "CDROM": DeviceCategory.STORAGE,
    "USB": DeviceCategory.USB_HUB,  # refined further by hardware ID
    "HIDClass": DeviceCategory.INPUT_DEVICE,
    "Keyboard": DeviceCategory.INPUT_DEVICE,
    "Mouse": DeviceCategory.INPUT_DEVICE,
    "Image": DeviceCategory.CAMERA,
    "Camera": DeviceCategory.CAMERA,
    "MEDIA": DeviceCategory.AUDIO,
    "AudioEndpoint": DeviceCategory.AUDIO,
    "Printer": DeviceCategory.PRINTER,
    "Net": DeviceCategory.NETWORK_ADAPTER,
    "Ports": DeviceCategory.SERIAL_DEVICE,
    "Modem": DeviceCategory.SERIAL_DEVICE,
    "WPD": DeviceCategory.MOBILE_DEVICE,  # Windows Portable Devices (phones)
    "SmartCardReader": DeviceCategory.SECURITY_DEVICE,
    "Biometric": DeviceCategory.SECURITY_DEVICE,
}

# Known development-board vendor IDs (VID), used only as a heuristic signal —
# never presented as a certainty beyond "Detected".
KNOWN_DEV_BOARD_VIDS: set[str] = {
    "2341",  # Arduino LLC
    "2A03",  # Arduino SA
    "1A86",  # QinHeng (CH340 - common on Arduino clones/ESP boards)
    "10C4",  # Silicon Labs (CP210x - ESP32/ESP8266 dev boards)
    "0483",  # STMicroelectronics (STM32 boards)
    "239A",  # Adafruit
    "303A",  # Espressif (ESP32-S/C native USB)
    "1B4F",  # SparkFun
}

# Xbox/PlayStation/Nintendo controller VIDs, used as a categorization hint.
KNOWN_CONTROLLER_VIDS: set[str] = {
    "045E",  # Microsoft (Xbox controllers)
    "054C",  # Sony (DualShock/DualSense)
    "057E",  # Nintendo (Switch Pro Controller)
    "046D",  # Logitech (also makes mice - refined by PID/interface class)
}