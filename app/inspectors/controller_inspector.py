"""Inspector for USB game controllers / joysticks (spec section 9).

This inspector produces six sections for a game controller:

  General            — name, manufacturer, category, status
  USB Information    — VID, PID, class, subclass, protocol
  Hardware           — hardware IDs, instance ID, serial, parent
  Driver             — driver name/provider/version/date
  Controller         — type, firmware rev, XInput/DirectInput, connection type,
                       all enumerated HID interfaces
  HID Analysis       — button count, axis list, hat switches, force-feedback,
                       report counts — sourced from the hidapi report descriptor
                       where the device allows it, or from hid.enumerate() alone

HID report descriptor parsing strategy
---------------------------------------
Windows HID minidriver stacks the device's report descriptor in the kernel.
The only way to read it from user-space Python without writing a kernel driver
is via the ``hid`` package (hidapi bindings).  hidapi exposes it through
``hid.Device.get_report_descriptor()`` which returns the raw bytes.

We parse those bytes ourselves using a minimal HID report-descriptor parser.
The HID spec (USB-IF, HID 1.11) defines items with a 1-byte prefix:
  bits[7:4]  tag
  bits[3:2]  type  (0=Main, 1=Global, 2=Local)
  bits[1:0]  size  (0→0 bytes, 1→1 byte, 2→2 bytes, 3→4 bytes)

We extract:
  * REPORT_COUNT + REPORT_SIZE → total bits per collection → button/axis counts
  * USAGE (Generic Desktop 0x01): X/Y/Z/Rx/Ry/Rz/Slider/Dial/Wheel/Hat
  * USAGE_PAGE 0x0F (PID — Physical Interface Device) → force-feedback present
  * USAGE_PAGE 0x01 USAGE 0x04 (Joystick) / 0x05 (Gamepad)

When the device refuses to open (exclusive-access driver, e.g. Xbox GIP) or
``hid`` is not installed, we degrade gracefully: all HID Analysis fields show
"Not Available" and a clear warning is added.
"""
from __future__ import annotations

import struct
from dataclasses import dataclass, field
from typing import Any

from app.inspectors.base_inspector import BaseInspector
from app.models.device_details import DeviceDetails
from app.models.usb_device import FieldSource, USBDevice
from app.usb.usb_constants import (
    USB_CLASS_NAMES,
    DeviceCategory,
    KNOWN_CONTROLLER_VIDS,
)
from app.usb.usb_utils import extract_firmware_revision, extract_interface_number
from app.utils.logger import get_logger

logger = get_logger(__name__)

# ---------------------------------------------------------------------------
# HID Generic Desktop usages we care about
# ---------------------------------------------------------------------------
_USAGE_PAGE_GENERIC_DESKTOP = 0x01
_USAGE_PAGE_BUTTON          = 0x09
_USAGE_PAGE_PID             = 0x0F   # Physical Interface Device (force feedback)

_USAGE_JOYSTICK    = 0x04
_USAGE_GAMEPAD     = 0x05
_USAGE_MULTI_AXIS  = 0x08

# Axis usages (Generic Desktop page 0x01)
_AXIS_USAGES: dict[int, str] = {
    0x30: "X Axis",
    0x31: "Y Axis",
    0x32: "Z Axis",
    0x33: "Rx (X Rotation)",
    0x34: "Ry (Y Rotation)",
    0x35: "Rz (Z Rotation)",
    0x36: "Slider",
    0x37: "Dial",
    0x38: "Wheel",
    0x39: "Hat Switch",
}


# ---------------------------------------------------------------------------
# Minimal HID report-descriptor parser
# ---------------------------------------------------------------------------

@dataclass
class ReportField:
    """One logical field extracted from the HID report descriptor.

    Preserves the exact bit offset within the input report so the live
    monitor can decode every field without guessing byte boundaries.

    kind values:
      "axis"    — an analog axis (X, Y, Rz, Slider, …)
      "hat"     — hat switch (4-bit, 0-7 = direction, 8 = centred)
      "buttons" — a block of N button bits starting at bit_offset
      "padding" — constant / padding bits; must be skipped during decode
    """
    kind: str           # "axis" | "hat" | "buttons" | "padding"
    name: str           # human label ("X Axis", "Button Block 1", …)
    bit_offset: int     # position of the first bit in the input report
    bit_size: int       # bits per field (per axis, or total for button block)
    count: int = 1      # number of items (1 for axis/hat, N for buttons)
    max_value: int = 0  # logical max (0 = derive from bit_size)


@dataclass
class _HIDAnalysis:
    """Aggregated results from parsing a HID report descriptor."""
    top_level_usage_page: int | None = None
    top_level_usage: int | None = None
    # Ordered list of all input fields in descriptor order — used by the
    # live monitor for bit-exact decoding.
    ordered_fields: list[ReportField] = field(default_factory=list)
    # Convenience summaries (derived from ordered_fields after parsing)
    axes: list[str] = field(default_factory=list)
    axis_bit_sizes: list[int] = field(default_factory=list)
    hat_count: int = 0
    button_count: int = 0
    has_force_feedback: bool = False
    has_rumble: bool = False
    interface_count: int = 0
    all_interfaces: list[dict[str, Any]] = field(default_factory=list)


def _parse_report_descriptor(raw: bytes) -> _HIDAnalysis:
    """Parse raw HID report descriptor bytes into an _HIDAnalysis.

    Tracks a running ``bit_cursor`` so every field's exact bit offset within
    the input report is recorded in ``analysis.ordered_fields``.  This lets
    the live monitor decode axes, hat, buttons, and padding in the correct
    order without any byte-boundary guesswork.

    Key correctness rules:
    * Input items with Data flag (bit 0 of the Input tag data = 0) are real
      fields.  Input items with Constant flag (bit 0 = 1) are padding and
      must advance the bit cursor but never produce a button or axis.
    * Button count counts only Data buttons, not Constant padding bits.
    * Fields are appended in the order they appear in the descriptor, which
      is the order they appear in the input report.
    """
    analysis = _HIDAnalysis()

    # Global state
    usage_page: int = 0
    report_count: int = 0
    report_size: int = 0
    logical_max: int = 0

    # Local state (reset after each Main item)
    usages: list[int] = []
    usage_minimum: int | None = None
    usage_maximum: int | None = None

    # Running bit cursor — tracks position within the input report
    bit_cursor: int = 0
    in_top_collection = False
    i = 0

    try:
        while i < len(raw):
            prefix = raw[i]
            i += 1

            if prefix == 0xFE:          # long item — skip
                if i < len(raw):
                    skip = raw[i]; i += 1 + skip
                continue

            size_code = prefix & 0x03
            item_type = (prefix >> 2) & 0x03
            tag        = (prefix >> 4) & 0x0F

            size = (0, 1, 2, 4)[size_code]
            if i + size > len(raw):
                break
            raw_value = raw[i: i + size]
            i += size
            value = int.from_bytes(raw_value, "little", signed=False) if size > 0 else 0

            # ── Global items ──────────────────────────────────────────
            if item_type == 1:
                if tag == 0x0:   usage_page   = value
                elif tag == 0x7: report_size  = value
                elif tag == 0x9: report_count = value
                elif tag == 0x4: logical_max  = value  # Logical Maximum
                if value == _USAGE_PAGE_PID:
                    analysis.has_force_feedback = True

            # ── Local items ───────────────────────────────────────────
            elif item_type == 2:
                if tag == 0x0:   usages.append(value)
                elif tag == 0x1: usage_minimum = value
                elif tag == 0x2: usage_maximum = value

            # ── Main items ────────────────────────────────────────────
            elif item_type == 0:

                if tag == 0xA:  # Collection
                    if not in_top_collection and usages:
                        analysis.top_level_usage_page = usage_page
                        analysis.top_level_usage = usages[-1]
                        in_top_collection = True

                elif tag == 0x8:  # Input
                    total_bits = report_size * report_count
                    # Input data flag: bit 0 of the Input tag data byte.
                    # 0 = Data (real field), 1 = Constant (padding).
                    is_constant = bool(value & 0x01)

                    if is_constant:
                        # Padding — advance cursor, record as padding field
                        analysis.ordered_fields.append(ReportField(
                            kind="padding", name=f"Padding@{bit_cursor}",
                            bit_offset=bit_cursor,
                            bit_size=report_size, count=report_count,
                        ))
                        bit_cursor += total_bits

                    elif usage_page == _USAGE_PAGE_BUTTON:
                        # Real button block
                        if usage_minimum is not None and usage_maximum is not None:
                            n_buttons = usage_maximum - usage_minimum + 1
                        elif usages:
                            n_buttons = len(usages)
                        else:
                            n_buttons = report_count
                        analysis.ordered_fields.append(ReportField(
                            kind="buttons",
                            name=f"Buttons {analysis.button_count + 1}–"
                                 f"{analysis.button_count + n_buttons}",
                            bit_offset=bit_cursor,
                            bit_size=report_size,
                            count=n_buttons,
                            max_value=1,
                        ))
                        analysis.button_count += n_buttons
                        bit_cursor += total_bits   # includes any padding bits in this Input

                    elif usage_page == _USAGE_PAGE_GENERIC_DESKTOP:
                        # Resolve usage list from explicit usages or min/max range
                        resolved: list[int] = list(usages)
                        if not resolved and usage_minimum is not None and usage_maximum is not None:
                            resolved = list(range(usage_minimum, usage_maximum + 1))

                        if not resolved:
                            # Unknown generic desktop field — treat as padding
                            analysis.ordered_fields.append(ReportField(
                                kind="padding", name=f"GD-unknown@{bit_cursor}",
                                bit_offset=bit_cursor,
                                bit_size=report_size, count=report_count,
                            ))
                            bit_cursor += total_bits
                        else:
                            for u in resolved:
                                if u not in _AXIS_USAGES:
                                    # Unknown usage within GD — advance 1 field
                                    bit_cursor += report_size
                                    continue
                                label = _AXIS_USAGES[u]
                                if label == "Hat Switch":
                                    analysis.ordered_fields.append(ReportField(
                                        kind="hat", name="Hat Switch",
                                        bit_offset=bit_cursor,
                                        bit_size=report_size, count=1,
                                        max_value=logical_max,
                                    ))
                                    analysis.hat_count += 1
                                    bit_cursor += report_size
                                else:
                                    if label not in analysis.axes:
                                        analysis.axes.append(label)
                                        analysis.axis_bit_sizes.append(report_size)
                                    analysis.ordered_fields.append(ReportField(
                                        kind="axis", name=label,
                                        bit_offset=bit_cursor,
                                        bit_size=report_size, count=1,
                                        max_value=logical_max,
                                    ))
                                    bit_cursor += report_size

                    elif usage_page == _USAGE_PAGE_PID:
                        analysis.has_force_feedback = True
                        bit_cursor += total_bits

                    else:
                        # Unknown usage page — skip
                        bit_cursor += total_bits

                elif tag == 0x9:  # Output
                    if usage_page == _USAGE_PAGE_PID:
                        analysis.has_force_feedback = True
                        analysis.has_rumble = True

                # Reset local state after any Main item
                usages = []
                usage_minimum = None
                usage_maximum = None

    except Exception as exc:  # noqa: BLE001
        logger.debug("HID report descriptor parse error at bit %d: %s", bit_cursor, exc)

    return analysis


# ---------------------------------------------------------------------------
# hidapi interaction helpers
# ---------------------------------------------------------------------------

def _enumerate_interfaces(vid: int, pid: int) -> list[dict[str, Any]]:
    """Return all HID interfaces for this VID/PID using hid.enumerate()."""
    try:
        import hid  # type: ignore
        return list(hid.enumerate(vid, pid))
    except ImportError:
        return []
    except Exception as exc:  # noqa: BLE001
        logger.debug("hid.enumerate(%04X,%04X) failed: %s", vid, pid, exc)
        return []


def _read_report_descriptor(vid: int, pid: int, usage_page: int = 0, usage: int = 0) -> bytes | None:
    """Open the HID device and read its report descriptor.

    Tries every enumerated interface; returns bytes from the first one that
    succeeds.  Returns None if the device refuses all opens (exclusive-access
    drivers like xboxgip) or if hidapi is not installed.

    Uses the hidapi (Cython) API:  hid.device() / .open_path() / .get_report_descriptor()
    """
    try:
        import hid  # type: ignore
    except ImportError:
        logger.debug("hidapi not installed; cannot read report descriptor")
        return None

    interfaces = _enumerate_interfaces(vid, pid)
    for info in interfaces:
        path = info.get("path")
        if not path:
            continue
        try:
            dev = hid.device()
            dev.open_path(path)
            try:
                descriptor = dev.get_report_descriptor()
                return bytes(descriptor) if descriptor else None
            finally:
                dev.close()
        except Exception as exc:  # noqa: BLE001
            logger.debug("Could not open HID path %s: %s", path, exc)
            continue
    return None


# ---------------------------------------------------------------------------
# XInput detection helpers
# ---------------------------------------------------------------------------

def _is_xinput(device: USBDevice) -> bool:
    """Return True when the device uses the XInput protocol."""
    haystack = " ".join(device.compatible_ids + device.hardware_ids).upper()
    return "XUSB" in haystack or "XINPUT" in haystack or "XBOXGIP" in haystack


# ---------------------------------------------------------------------------
# ControllerInspector
# ---------------------------------------------------------------------------

class ControllerInspector(BaseInspector):
    categories = (DeviceCategory.GAME_CONTROLLER.value,)

    def inspect(self, device: USBDevice) -> DeviceDetails:
        details = DeviceDetails(device=device)
        self.build_general_section(details, device)
        self.build_usb_information_section(details, device)
        self.build_hardware_section(details, device)
        self.build_driver_section(details, device)
        self._build_controller_section(details, device)
        self._build_hid_analysis_section(details, device)
        return details

    # ------------------------------------------------------------------
    # Section: Controller
    # ------------------------------------------------------------------

    def _build_controller_section(self, details: DeviceDetails, device: USBDevice) -> None:
        section = details.get_or_create_section("Controller")

        section.add("Controller Name", device.name, source="Directly Reported" if device.name else "Unknown")
        section.add("Manufacturer", device.manufacturer, source="Directly Reported" if device.manufacturer else "Unknown")
        section.add("Vendor ID", device.vendor_id, source="Directly Reported" if device.vendor_id else "Unknown")
        section.add("Product ID", device.product_id, source="Directly Reported" if device.product_id else "Unknown")

        # Firmware revision from REV_XXXX in hardware ID
        firmware_rev = extract_firmware_revision(device.hardware_ids)
        if firmware_rev:
            # bcdDevice: high byte = major, low byte = minor
            try:
                major = int(firmware_rev[:2], 16)
                minor = int(firmware_rev[2:], 16)
                rev_display = f"{major}.{minor:02d}  (bcdDevice: {firmware_rev})"
            except ValueError:
                rev_display = firmware_rev
        else:
            rev_display = None
        section.add("Firmware Revision", rev_display,
                    source="Directly Reported" if firmware_rev else "Unknown")

        # Interface protocol: XInput vs DirectInput
        xinput = _is_xinput(device)
        section.add(
            "Input Protocol",
            "XInput (Xbox-compatible)" if xinput else "DirectInput / HID",
            source="Detected",
        )

        # Connection type: USB wired vs Bluetooth
        dev_id_upper = device.device_id.upper()
        if dev_id_upper.startswith("USB\\"):
            conn_type = "USB (Wired)"
            conn_source = "Directly Reported"
        elif any(dev_id_upper.startswith(p) for p in ("BTH\\", "BTHENUM\\", "BLUETOOTH\\")):
            conn_type = "Bluetooth (Wireless)"
            conn_source = "Directly Reported"
        else:
            conn_type = None
            conn_source = "Unknown"
        section.add("Connection Type", conn_type, source=conn_source)

        # Controller type (derived from name/usage)
        controller_type = self._determine_type(device)
        section.add("Controller Type", controller_type, source="Detected")

        # Number of HID interfaces — tells us if it's a composite device
        if device.vendor_id and device.product_id:
            try:
                vid_int = int(device.vendor_id, 16)
                pid_int = int(device.product_id, 16)
                interfaces = _enumerate_interfaces(vid_int, pid_int)
            except (ValueError, Exception):  # noqa: BLE001
                interfaces = []
        else:
            interfaces = []

        iface_count = len(interfaces)
        section.add(
            "HID Interfaces",
            str(iface_count) if iface_count else None,
            source="Detected" if iface_count else "Unknown",
        )

        if iface_count > 1:
            iface_labels = []
            for iface in interfaces:
                usage_page = iface.get("usage_page")
                usage = iface.get("usage")
                iface_num = iface.get("interface_number", "?")
                up_name = _usage_page_name(usage_page)
                u_name  = _usage_name(usage_page, usage)
                iface_labels.append(f"Interface {iface_num}: {up_name} / {u_name}")
            section.add("Interface Details", "\n".join(iface_labels), source="Detected")

        # Capabilities
        is_known_vid = (device.vendor_id or "").upper() in KNOWN_CONTROLLER_VIDS
        self.add_capability_if(details, True, "Game Controller",
                               evidence="Category = Game Controller")
        self.add_capability_if(details, is_known_vid, "Known Controller Vendor",
                               evidence=f"VID {device.vendor_id} in known controller VID list",
                               source=FieldSource.DERIVED)
        self.add_capability_if(details, xinput, "XInput (Xbox-compatible)",
                               evidence="CompatibleID contains XUSB/XINPUT marker")
        self.add_capability_if(details, not xinput, "DirectInput / HID",
                               evidence="No XInput marker found; standard HID path")

    # ------------------------------------------------------------------
    # Section: HID Analysis
    # ------------------------------------------------------------------

    def _build_hid_analysis_section(self, details: DeviceDetails, device: USBDevice) -> None:
        section = details.get_or_create_section("HID Analysis")

        if not device.vendor_id or not device.product_id:
            section.add("Note", "VID/PID not available; cannot query HID interfaces")
            return

        try:
            vid_int = int(device.vendor_id, 16)
            pid_int = int(device.product_id, 16)
        except ValueError:
            section.add("Note", "Invalid VID/PID; cannot query HID interfaces")
            return

        # ── Step 1: enumerate interfaces (does not open the device) ──
        interfaces = _enumerate_interfaces(vid_int, pid_int)
        if not interfaces:
            section.add(
                "HID Enumeration",
                None,
                source="Unknown",
            )
            details.warnings.append(
                "No HID interfaces were returned by hidapi for this device. "
                "The device may use a non-HID driver (e.g. XInput/xboxgip) "
                "or the hid package may not be installed."
            )
            self._add_hid_capabilities(details, None)
            return

        # Surface per-interface usage page and usage from enumeration
        # (available without opening the device).
        primary = interfaces[0]
        enum_usage_page = primary.get("usage_page")
        enum_usage = primary.get("usage")

        section.add(
            "HID Usage Page",
            f"{enum_usage_page:#06x}  ({_usage_page_name(enum_usage_page)})"
            if enum_usage_page is not None else None,
            source="Directly Reported",
        )
        section.add(
            "HID Usage",
            f"{enum_usage:#06x}  ({_usage_name(enum_usage_page, enum_usage)})"
            if enum_usage is not None else None,
            source="Directly Reported",
        )

        # ── Step 2: attempt to read the report descriptor ─────────────
        raw_descriptor = _read_report_descriptor(vid_int, pid_int)

        if raw_descriptor is None:
            # Cannot open device (exclusive-access driver, e.g. xboxgip).
            # Provide as much info as we can from enumeration alone.
            section.add("Report Descriptor", "Not accessible (device held by exclusive driver)",
                        source="Unknown")
            self._fill_from_enumeration_only(section, details, interfaces)
            details.warnings.append(
                "The HID report descriptor could not be read. "
                "The device driver holds exclusive access — this is normal for "
                "XInput/Xbox controllers and some proprietary HID devices. "
                "Button/axis counts are not available."
            )
            self._add_hid_capabilities(details, None)
            return

        # ── Step 3: parse the report descriptor ───────────────────────
        section.add(
            "Report Descriptor",
            f"{len(raw_descriptor)} bytes  (raw available)",
            source="Directly Reported",
        )

        analysis = _parse_report_descriptor(raw_descriptor)

        # Override top-level usage with enumeration values when parser
        # found nothing (some descriptors start with a PHYSICAL collection).
        if analysis.top_level_usage is None:
            analysis.top_level_usage_page = enum_usage_page
            analysis.top_level_usage = enum_usage

        # Button count
        section.add(
            "Button Count",
            str(analysis.button_count) if analysis.button_count else None,
            source="Directly Reported" if analysis.button_count else "Unknown",
        )

        # Axes
        if analysis.axes:
            section.add("Axes", ", ".join(analysis.axes), source="Directly Reported")
            section.add("Axis Count", str(len(analysis.axes)), source="Detected")
            # Store bit sizes for the live monitor (format: "10,10,8,8")
            if analysis.axis_bit_sizes:
                section.add(
                    "Axis Bit Sizes",
                    ",".join(str(b) for b in analysis.axis_bit_sizes),
                    source="Directly Reported",
                )
            # Store the full ordered field map for bit-exact live decoding
            # Format: "kind:name:bit_offset:bit_size:count" per field, "|" separated
            if analysis.ordered_fields:
                field_map = "|".join(
                    f"{f.kind}:{f.name}:{f.bit_offset}:{f.bit_size}:{f.count}"
                    for f in analysis.ordered_fields
                )
                section.add("Report Field Map", field_map, source="Directly Reported")
        else:
            section.add("Axes", None, source="Unknown")
            section.add("Axis Count", None, source="Unknown")

        # Hat switches
        section.add(
            "Hat Switch(es)",
            str(analysis.hat_count) if analysis.hat_count else None,
            source="Directly Reported" if analysis.hat_count else "Unknown",
        )

        # Force feedback
        ff_value = "Yes — Force Feedback supported" if analysis.has_force_feedback else "No"
        section.add("Force Feedback", ff_value,
                    source="Directly Reported" if analysis.has_force_feedback else "Detected")

        rumble_value = "Yes — Rumble / vibration output present" if analysis.has_rumble else "No"
        section.add("Rumble Motor", rumble_value,
                    source="Directly Reported" if analysis.has_rumble else "Detected")

        # Capabilities from descriptor analysis
        self._add_hid_capabilities(details, analysis)

    def _fill_from_enumeration_only(
        self,
        section,
        details: DeviceDetails,
        interfaces: list[dict],
    ) -> None:
        """Fill what we can without the report descriptor."""
        # Try to infer type from usage alone.
        usage = interfaces[0].get("usage") if interfaces else None
        usage_page = interfaces[0].get("usage_page") if interfaces else None

        if usage_page == _USAGE_PAGE_GENERIC_DESKTOP and usage in (_USAGE_JOYSTICK, _USAGE_GAMEPAD, _USAGE_MULTI_AXIS):
            section.add("Controller Classification",
                        _usage_name(usage_page, usage),
                        source="Directly Reported")
        else:
            section.add("Controller Classification", None, source="Unknown")

        section.add("Button Count", None, source="Unknown")
        section.add("Axes", None, source="Unknown")
        section.add("Hat Switch(es)", None, source="Unknown")
        section.add("Force Feedback", None, source="Unknown")
        section.add("Rumble Motor", None, source="Unknown")

    def _add_hid_capabilities(
        self, details: DeviceDetails, analysis: _HIDAnalysis | None
    ) -> None:
        if analysis is None:
            return
        self.add_capability_if(
            details, analysis.button_count > 0,
            f"{analysis.button_count} Buttons",
            evidence=f"HID report descriptor: REPORT_COUNT={analysis.button_count} on button usage page",
        )
        if analysis.axes:
            self.add_capability_if(
                details, True,
                f"{len(analysis.axes)} Analog Axes ({', '.join(analysis.axes)})",
                evidence="HID report descriptor: Generic Desktop axis usages",
            )
        self.add_capability_if(
            details, analysis.hat_count > 0,
            f"{analysis.hat_count} Hat Switch{'es' if analysis.hat_count > 1 else ''}",
            evidence="HID report descriptor: Hat Switch usage (0x39)",
        )
        self.add_capability_if(
            details, analysis.has_force_feedback,
            "Force Feedback (PID)",
            evidence="HID report descriptor contains PID usage page (0x0F)",
        )
        self.add_capability_if(
            details, analysis.has_rumble,
            "Rumble / Vibration Output",
            evidence="HID report descriptor: Output reports on PID usage page",
        )

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    @staticmethod
    def _determine_type(device: USBDevice) -> str:
        name_lower = (device.name or "").lower()
        if any(k in name_lower for k in ("xbox", "xinput")):
            return "Xbox Controller"
        if any(k in name_lower for k in ("dualshock", "dualsense", "playstation")):
            return "PlayStation Controller"
        if any(k in name_lower for k in ("wingman", "extreme", "attack", "force")):
            return "Logitech Joystick"
        if "thrustmaster" in name_lower:
            return "Thrustmaster Controller"
        if "sidewinder" in name_lower:
            return "Microsoft SideWinder"
        if any(k in name_lower for k in ("joystick", "flight", "flightstick", "hotas")):
            return "Flight Joystick"
        if any(k in name_lower for k in ("wheel", "racing")):
            return "Racing Wheel"
        if any(k in name_lower for k in ("gamepad", "joypad")):
            return "Gamepad"
        if "controller" in name_lower:
            return "Game Controller"
        return "Game Controller"


# ---------------------------------------------------------------------------
# Formatting helpers
# ---------------------------------------------------------------------------

def _usage_page_name(usage_page: int | None) -> str:
    if usage_page is None:
        return "Unknown"
    names = {
        0x01: "Generic Desktop",
        0x02: "Simulation Controls",
        0x03: "VR Controls",
        0x04: "Sport Controls",
        0x05: "Game Controls",
        0x09: "Button",
        0x0C: "Consumer",
        0x0F: "Physical Interface Device (Force Feedback)",
        0x84: "Power Device",
    }
    return names.get(usage_page, f"Page 0x{usage_page:02X}")


def _usage_name(usage_page: int | None, usage: int | None) -> str:
    if usage_page is None or usage is None:
        return "Unknown"
    if usage_page == _USAGE_PAGE_GENERIC_DESKTOP:
        names = {
            0x01: "Pointer",
            0x02: "Mouse",
            0x04: "Joystick",
            0x05: "Gamepad",
            0x06: "Keyboard",
            0x07: "Keypad",
            0x08: "Multi-axis Controller",
            0x09: "Tablet PC System Controls",
        }
        return names.get(usage, f"Usage 0x{usage:04X}")
    return f"Usage 0x{usage:04X}"
