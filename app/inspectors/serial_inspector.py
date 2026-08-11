"""Inspector for USB serial / COM port devices (spec section 9 - Serial Devices).

This covers:
* CDC ACM devices (Arduino, ESP boards, modems)
* CH340/CH341 USB-UART adapters
* CP210x USB-UART bridges (Silicon Labs)
* FTDI FT232/FT2232 adapters
* Other USB-to-serial adapters that Windows enumerates under PNPClass "Ports"

The COM port number is the most user-relevant piece of information. We obtain
it by querying Win32_SerialPort (by PNPDeviceID) and as a fallback by scanning
the parent-device's children via Win32_PnPEntity filtering by Name containing
"COM".
"""
from __future__ import annotations

from app.inspectors.base_inspector import BaseInspector
from app.models.device_details import DeviceDetails
from app.models.usb_device import USBDevice
from app.usb.usb_constants import DeviceCategory
from app.utils.logger import get_logger

logger = get_logger(__name__)


class SerialInspector(BaseInspector):
    categories = (DeviceCategory.SERIAL_DEVICE.value,)

    def inspect(self, device: USBDevice) -> DeviceDetails:
        details = DeviceDetails(device=device)
        self.build_general_section(details, device)
        self.build_usb_information_section(details, device)
        self.build_hardware_section(details, device)
        self.build_driver_section(details, device)

        section = details.get_or_create_section("Serial Port")
        section.add("Device Description", device.description, source="Directly Reported" if device.description else "Unknown")
        section.add("Manufacturer", device.manufacturer, source="Directly Reported" if device.manufacturer else "Unknown")
        section.add("Vendor ID", device.vendor_id, source="Directly Reported" if device.vendor_id else "Unknown")
        section.add("Product ID", device.product_id, source="Directly Reported" if device.product_id else "Unknown")
        section.add("Serial Number", device.serial_number, source=device.serial_source.value)

        com_port = self._find_com_port(device)
        section.add("COM Port", com_port, source="Directly Reported" if com_port else "Unknown")

        baud_rates = self._get_supported_baud_rates(device)
        section.add("Supported Baud Rates", baud_rates, source="Detected" if baud_rates else "Unknown")

        chip_type = self._identify_chip(device)
        section.add("Controller Chip", chip_type, source="Derived" if chip_type else "Unknown")

        if com_port is None:
            details.warnings.append(
                "COM port assignment could not be determined. "
                "The device may not be fully enumerated or the driver may not be installed."
            )

        # Capabilities
        self.add_capability_if(
            details, True, "USB Serial / COM Port",
            evidence="Category classified as Serial Device",
        )
        self.add_capability_if(
            details, com_port is not None,
            f"Assigned {com_port}" if com_port else "COM Port Assigned",
            evidence=f"Win32_SerialPort or PnP name contains '{com_port}'",
        )

        return details

    @staticmethod
    def _find_com_port(device: USBDevice) -> str | None:
        """Query Win32_SerialPort for a matching COM port assignment."""
        try:
            import wmi  # type: ignore
            conn = wmi.WMI()
        except Exception as exc:  # noqa: BLE001
            logger.debug("WMI unavailable for COM port lookup: %s", exc)
            return None

        # Strategy 1: Win32_SerialPort.PNPDeviceID direct match
        try:
            ports = conn.Win32_SerialPort()
            for port in ports:
                pnp_id = getattr(port, "PNPDeviceID", "") or ""
                if pnp_id.upper() == device.device_id.upper():
                    device_id = getattr(port, "DeviceID", None)
                    if device_id:
                        return str(device_id)
        except Exception as exc:  # noqa: BLE001
            logger.debug("Win32_SerialPort query failed: %s", exc)

        # Strategy 2: Scan all PnP entities whose Name contains "(COMx)".
        # Many USB-UART chips create a child device with the COM port in its name.
        try:
            vid_pid_fragment = ""
            if device.vendor_id and device.product_id:
                vid_pid_fragment = f"VID_{device.vendor_id}&PID_{device.product_id}".upper()

            entities = conn.Win32_PnPEntity()
            for entity in entities:
                name = getattr(entity, "Name", "") or ""
                # Match "USB Serial Device (COM3)" style names
                if "(COM" in name.upper():
                    pnp_id = (getattr(entity, "PNPDeviceID", "") or "").upper()
                    if vid_pid_fragment and vid_pid_fragment in pnp_id:
                        import re
                        m = re.search(r"\(COM\d+\)", name, re.IGNORECASE)
                        if m:
                            return m.group(0).strip("()")
        except Exception as exc:  # noqa: BLE001
            logger.debug("PnP COM port scan failed: %s", exc)

        return None

    @staticmethod
    def _get_supported_baud_rates(device: USBDevice) -> str | None:
        """Return a description of common supported baud rates (derived from chip type)."""
        chip = SerialInspector._identify_chip(device)
        if chip:
            return "300 – 3,000,000 bps (chip dependent)"
        return None

    @staticmethod
    def _identify_chip(device: USBDevice) -> str | None:
        """Heuristically identify the USB-UART controller chip from VID/PID."""
        vid = (device.vendor_id or "").upper()
        pid = (device.product_id or "").upper()
        name_lower = (device.name or "").lower()
        desc_lower = (device.description or "").lower()

        chip_map = {
            ("1A86", "7523"): "CH340",
            ("1A86", "7522"): "CH340",
            ("1A86", "5523"): "CH341",
            ("10C4", "EA60"): "CP2102",
            ("10C4", "EA61"): "CP2103",
            ("10C4", "EA70"): "CP2105",
            ("10C4", "EA71"): "CP2108",
            ("0403", "6001"): "FT232R",
            ("0403", "6010"): "FT2232",
            ("0403", "6011"): "FT4232",
            ("0403", "6014"): "FT232H",
            ("067B", "2303"): "PL2303",
            ("067B", "23A3"): "PL2303",
        }
        if vid and pid:
            result = chip_map.get((vid, pid))
            if result:
                return result

        # Text-based fallback
        for keyword, chip_name in (
            ("ch340", "CH340"), ("ch341", "CH341"),
            ("cp210", "CP210x"), ("ft232", "FT232"),
            ("pl2303", "PL2303"), ("ftdi", "FTDI"),
        ):
            if keyword in name_lower or keyword in desc_lower:
                return chip_name

        return None
