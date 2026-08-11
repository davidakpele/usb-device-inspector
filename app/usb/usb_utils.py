r"""USB utility helpers: VID/PID extraction, serial-number parsing, etc.

All functions accept possibly-None / possibly-malformed strings and degrade
gracefully — callers must never crash due to unexpected hardware data.

Hardware-ID string anatomy (from Windows PnP layer)::

  Primary HW ID:   USB\VID_046D&PID_C207&REV_0101
  Interface HW ID: USB\VID_046D&PID_C207&REV_0101&MI_00
  Compatible ID:   USB\Class_03&SubClass_00&Prot_00
  Compatible ID:   USB\Class_03&SubClass_00
  Compatible ID:   USB\Class_03
"""
from __future__ import annotations

import re

# Matches "VID_XXXX&PID_XXXX" anywhere inside a hardware/device ID string.
_VID_PID_RE = re.compile(r"VID_([0-9A-Fa-f]{4})&PID_([0-9A-Fa-f]{4})", re.IGNORECASE)

# Firmware revision embedded as "REV_XXXX" in the primary hardware ID.
_REV_RE = re.compile(r"REV_([0-9A-Fa-f]{4})", re.IGNORECASE)

# USB class/subclass/protocol in compatible ID strings:
#   "USB\Class_03&SubClass_01&Prot_01"  →  class=03, sub=01, prot=01
_CLASS_FULL_RE  = re.compile(r"Class_([0-9A-Fa-f]{2})&SubClass_([0-9A-Fa-f]{2})&Prot_([0-9A-Fa-f]{2})", re.IGNORECASE)
_CLASS_SUB_RE   = re.compile(r"Class_([0-9A-Fa-f]{2})&SubClass_([0-9A-Fa-f]{2})",                       re.IGNORECASE)
_CLASS_ONLY_RE  = re.compile(r"Class_([0-9A-Fa-f]{2})",                                                  re.IGNORECASE)

# The third backslash-separated segment of a PNPDeviceID for a USB device is
# the "instance qualifier" — it is the serial number IF it contains only
# alphanumeric + '&' characters and its length is > 1.  A value like
# "0000000000000000" or a single digit or all zeros is not a real serial.
_SERIAL_RE = re.compile(r"^[0-9A-Fa-f]{1,2}$")  # single/double hex → not serial


def extract_vid_pid(device_id: str | None) -> tuple[str | None, str | None]:
    """Extract (VID, PID) strings (upper-case hex, no prefix) from a device-ID.

    Returns (None, None) when the ID is missing or lacks a recognizable
    VID/PID pair rather than raising.
    """
    if not device_id:
        return None, None
    match = _VID_PID_RE.search(device_id)
    if not match:
        return None, None
    return match.group(1).upper(), match.group(2).upper()


def extract_serial_from_instance_id(instance_id: str | None) -> str | None:
    """Extract the serial number component from a PNPDeviceID string.

    USB PNPDeviceIDs have the form:
        USB\\VID_XXXX&PID_XXXX\\<serial_or_location>

    The third segment is the serial number when it is longer than two characters
    and not composed solely of a USB port/location token like "6&1A2B3C4D&0&1".
    Returns None rather than fabricating a value.
    """
    if not instance_id:
        return None
    parts = instance_id.split("\\")
    if len(parts) < 3:
        return None
    candidate = parts[2].strip()
    if not candidate:
        return None
    # Location strings look like "6&XXXXXXXX&0&N" — contain '&' and are short
    # hex tokens between ampersands. A serial number is a single uninterrupted
    # alphanumeric token, typically 12+ characters.
    if "&" in candidate:
        return None  # location-based identifier, not a serial number
    if len(candidate) <= 2:
        return None
    # All-zero strings are not real serials.
    if all(c == "0" for c in candidate):
        return None
    return candidate


def extract_firmware_revision(hardware_ids: list[str]) -> str | None:
    """Extract the firmware/device revision from the REV_XXXX token.

    The primary hardware ID for a USB device looks like:
        USB\\VID_046D&PID_C207&REV_0101
    Returns the 4-digit hex string (e.g. "0101") or None if absent.
    Source: directly reported in the USB device descriptor (bcdDevice field).
    """
    for hid in hardware_ids:
        m = _REV_RE.search(hid)
        if m:
            return m.group(1).upper()
    return None


def extract_usb_class_subclass_protocol(
    compatible_ids: list[str],
) -> tuple[str | None, str | None, str | None]:
    """Parse USB device class, subclass, and protocol from compatible-ID strings.

    Windows populates CompatibleID with entries like:
        USB\\Class_03&SubClass_01&Prot_01   (full)
        USB\\Class_03&SubClass_01            (partial)
        USB\\Class_03                         (class only)

    Returns (class_hex, subclass_hex, protocol_hex) — any may be None.
    All values are upper-case 2-digit hex strings (e.g. "03", "01", "00").
    Source: Directly Reported (these values come from the USB interface descriptor).
    """
    best_class: str | None = None
    best_sub: str | None = None
    best_prot: str | None = None

    for cid in compatible_ids:
        # Try the most-specific pattern first (class + subclass + protocol).
        m = _CLASS_FULL_RE.search(cid)
        if m:
            return m.group(1).upper(), m.group(2).upper(), m.group(3).upper()

        m = _CLASS_SUB_RE.search(cid)
        if m:
            best_class = m.group(1).upper()
            best_sub = m.group(2).upper()
            continue

        m = _CLASS_ONLY_RE.search(cid)
        if m and best_class is None:
            best_class = m.group(1).upper()

    return best_class, best_sub, best_prot


def extract_interface_number(hardware_id: str | None) -> int | None:
    """Extract the MI (multiple-interface) index from a hardware ID.

    E.g. "USB\\VID_046D&PID_C21D&REV_0220&MI_01" → 1
    Returns None if no MI token is present (single-interface device).
    """
    if not hardware_id:
        return None
    m = re.search(r"&MI_([0-9A-Fa-f]{2})", hardware_id, re.IGNORECASE)
    if m:
        try:
            return int(m.group(1), 16)
        except ValueError:
            return None
    return None


def first_non_empty(*values: str | None) -> str | None:
    """Return the first truthy string value, or None if all are empty/None."""
    for v in values:
        if v and v.strip():
            return v.strip()
    return None


def format_vid_pid(vid: str | None, pid: str | None) -> str:
    """Format VID and PID for display, e.g. 'VID: 046D  PID: C077'."""
    parts = []
    if vid:
        parts.append(f"VID: {vid.upper()}")
    if pid:
        parts.append(f"PID: {pid.upper()}")
    return "  ".join(parts) if parts else "Not Available"


def safe_hex(value: str | None, prefix: str = "") -> str | None:
    """Normalize a hex string: strip '0x'/'0X' prefix, upper-case.

    Returns None for None/empty input.
    """
    if not value:
        return None
    cleaned = value.strip().upper().lstrip("0X")
    return f"{prefix}{cleaned}" if cleaned else None
