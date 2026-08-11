"""Normalization + classification: RawPnPDescriptor -> USBDevice.

Kept separate from usb_enumerator.py (which only talks to Windows) so
classification rules can be unit-tested with plain Python objects and no
WMI/mocking required.
"""
from __future__ import annotations

from app.models.usb_device import ConnectionStatus, FieldSource, USBDevice
from app.usb.usb_constants import (
    KNOWN_CONTROLLER_VIDS,
    KNOWN_DEV_BOARD_VIDS,
    PNP_CLASS_TO_CATEGORY,
    DeviceCategory,
)
from app.usb.usb_descriptor import RawPnPDescriptor
from app.usb.usb_utils import (
    extract_firmware_revision,
    extract_serial_from_instance_id,
    extract_usb_class_subclass_protocol,
    extract_vid_pid,
    first_non_empty,
)
from app.utils.logger import get_logger

logger = get_logger(__name__)


def normalize(descriptor: RawPnPDescriptor) -> USBDevice:
    """Build a USBDevice from a raw descriptor. Never raises; degrades gracefully."""
    vid, pid = extract_vid_pid(descriptor.primary_hardware_id or descriptor.device_id)

    serial = extract_serial_from_instance_id(descriptor.device_id)
    serial_source = FieldSource.DETECTED if serial else FieldSource.UNKNOWN

    status = ConnectionStatus.CONNECTED if descriptor.present else ConnectionStatus.DISCONNECTED

    # Parse USB class/subclass/protocol from compatible-ID strings.
    # These values come from the USB interface descriptor and are directly
    # reported by Windows PnP — source = DIRECTLY_REPORTED.
    device_class, device_subclass, device_protocol = extract_usb_class_subclass_protocol(
        descriptor.compatible_ids
    )

    # Parse firmware revision (bcdDevice) from the primary hardware ID.
    # E.g. "USB\VID_046D&PID_C207&REV_0101" → "0101"
    firmware_rev = extract_firmware_revision(descriptor.hardware_ids)

    device = USBDevice(
        device_id=descriptor.device_id,
        name=first_non_empty(descriptor.name, descriptor.description),
        manufacturer=descriptor.manufacturer,
        description=descriptor.description,
        vendor_id=vid,
        product_id=pid,
        serial_number=serial,
        serial_source=serial_source,
        device_class=device_class,
        device_subclass=device_subclass,
        device_protocol=device_protocol,
        # usb_version: not exposed by Win32_PnPEntity — left None
        hardware_id=descriptor.primary_hardware_id,
        hardware_ids=list(descriptor.hardware_ids),
        compatible_ids=list(descriptor.compatible_ids),
        instance_id=descriptor.device_id,
        status=status,
        connected=descriptor.present,
    )

    # Stash the firmware revision in driver_version as a temporary carrier so
    # the inspector can surface it.  It will be overwritten by the real driver
    # version when device_scanner calls get_driver_info() later; we save it on
    # the hardware_ids list so the inspector can re-extract it cleanly.
    # (The USBDevice model does not have a dedicated firmware_rev field yet,
    # so we expose it through the inspector's own extraction call.)

    category, source = classify(descriptor, vid)
    device.category = category.value
    device.category_source = source
    return device


def classify(descriptor: RawPnPDescriptor, vid: str | None) -> tuple[DeviceCategory, FieldSource]:
    """Classify a device into a DeviceCategory with honest provenance.

    Precedence:
      1. PNPClass mapping (Windows-reported) → DETECTED, but HIDClass is
         refined: joysticks/gamepads inside HIDClass get promoted to
         GAME_CONTROLLER before returning INPUT_DEVICE.
      2. Hardware-ID / compatible-ID keyword heuristics → DETECTED
      3. Known-VID heuristics (dev boards, controllers) → DERIVED (weaker)
      4. Fall back to UNKNOWN, never guess without a real signal.
    """
    pnp_class = descriptor.pnp_class or ""

    if pnp_class in PNP_CLASS_TO_CATEGORY:
        category = PNP_CLASS_TO_CATEGORY[pnp_class]

        # ── Refine HIDClass before accepting INPUT_DEVICE ──────────────
        # Windows puts joysticks/gamepads in HIDClass alongside mice and
        # keyboards. We promote them to GAME_CONTROLLER when:
        #   a) The device name/description contains joystick/gamepad keywords, OR
        #   b) The compatible-IDs contain "HID_DEVICE_SYSTEM_GAME_CONTROLLER"
        #      or XInput-specific tokens, OR
        #   c) The known-VID list matches AND the name looks like a controller.
        if category == DeviceCategory.INPUT_DEVICE:
            if _is_game_controller(descriptor, vid):
                return DeviceCategory.GAME_CONTROLLER, FieldSource.DETECTED

        # "USB" PNPClass is ambiguous (covers hubs and generic USB devices) -
        # refine using hardware ID keywords before trusting it.
        if category == DeviceCategory.USB_HUB and not _looks_like_hub(descriptor):
            refined = _classify_by_hardware_id(descriptor)
            if refined is not None:
                return refined, FieldSource.DETECTED

        return category, FieldSource.DETECTED

    refined = _classify_by_hardware_id(descriptor)
    if refined is not None:
        return refined, FieldSource.DETECTED

    if vid:
        if vid in KNOWN_DEV_BOARD_VIDS:
            return DeviceCategory.DEVELOPMENT_BOARD, FieldSource.DERIVED
        if vid in KNOWN_CONTROLLER_VIDS and _looks_like_controller(descriptor):
            return DeviceCategory.GAME_CONTROLLER, FieldSource.DERIVED

    return DeviceCategory.UNKNOWN, FieldSource.UNKNOWN


# ---------------------------------------------------------------------------
# Private helpers
# ---------------------------------------------------------------------------

def _is_game_controller(descriptor: RawPnPDescriptor, vid: str | None) -> bool:
    """Return True when a HIDClass device is a joystick / gamepad.

    We check four independent signals; any one is sufficient:
      1. Compatible-ID contains the Windows-reported game-controller string.
      2. Name or description contains explicit joystick/gamepad keywords.
      3. XInput / XUSB marker in compatible-IDs (Xbox controller).
      4. Known-controller VID AND name-based controller keyword.
    """
    haystack_ids = " ".join(
        descriptor.hardware_ids + descriptor.compatible_ids
    ).upper()
    name_desc = ((descriptor.name or "") + " " + (descriptor.description or "")).lower()

    # 1. Windows-assigned game-controller compatible ID
    if "HID_DEVICE_SYSTEM_GAME_CONTROLLER" in haystack_ids:
        return True

    # 2. XInput / Xbox
    if "XUSB" in haystack_ids or "XINPUT" in haystack_ids:
        return True
    if "XBOXGIP" in haystack_ids or "XBOXGIP" in name_desc:
        return True

    # 3. Name/description keywords that unambiguously indicate a controller
    controller_keywords = (
        "joystick", "gamepad", "game controller", "game pad",
        "joypad", "wingman", "sidewinder", "thrustmaster",
        "flightstick", "hotas", "wheel", "racing",
        "dualshock", "dualsense", "xbox controller",
    )
    if any(k in name_desc for k in controller_keywords):
        return True

    # 4. Known VID + looser name signal
    if vid and vid.upper() in KNOWN_CONTROLLER_VIDS:
        if _looks_like_controller(descriptor):
            return True

    return False


def _looks_like_hub(descriptor: RawPnPDescriptor) -> bool:
    haystack = " ".join(descriptor.hardware_ids + [descriptor.name or "", descriptor.description or ""])
    return "HUB" in haystack.upper()


def _classify_by_hardware_id(descriptor: RawPnPDescriptor) -> DeviceCategory | None:
    haystack = " ".join(
        descriptor.hardware_ids + descriptor.compatible_ids + [descriptor.device_id]
    ).upper()

    # Check for controller markers before the generic HID/CLASS_03 check so
    # that a joystick with a non-HIDClass PNP class still gets promoted.
    if "HID_DEVICE_SYSTEM_GAME_CONTROLLER" in haystack or "XUSB" in haystack or "XINPUT" in haystack:
        return DeviceCategory.GAME_CONTROLLER

    if "USBSTOR" in haystack or "SCSI\\DISK" in haystack:
        return DeviceCategory.STORAGE
    if "HID_DEVICE_SYSTEM_MOUSE" in haystack:
        return DeviceCategory.INPUT_DEVICE
    if "HID_DEVICE_SYSTEM_KEYBOARD" in haystack or "KEYBOARD" in (descriptor.name or "").upper():
        return DeviceCategory.INPUT_DEVICE
    if "CLASS_03" in haystack:  # USB interface class 0x03 = HID (generic, after controller check)
        return DeviceCategory.INPUT_DEVICE
    if "CLASS_08" in haystack:  # Mass Storage
        return DeviceCategory.STORAGE
    if "CLASS_07" in haystack:  # Printer
        return DeviceCategory.PRINTER
    if "CLASS_0E" in haystack or "CLASS_06" in haystack:  # Video / Image
        return DeviceCategory.CAMERA
    if "CLASS_01" in haystack:  # Audio
        return DeviceCategory.AUDIO
    if "CLASS_02" in haystack or "CLASS_0A" in haystack:  # CDC comm/data
        return DeviceCategory.SERIAL_DEVICE
    if "CLASS_0B" in haystack:  # Smart card
        return DeviceCategory.SECURITY_DEVICE
    if "WPD" in haystack or "MTP" in haystack:
        return DeviceCategory.MOBILE_DEVICE
    if "USB\\VID" in haystack and "CLASS_09" in haystack:  # Hub
        return DeviceCategory.USB_HUB
    return None


def _looks_like_controller(descriptor: RawPnPDescriptor) -> bool:
    haystack = (descriptor.name or "") + " " + (descriptor.description or "")
    keywords = (
        "controller", "gamepad", "joystick", "xbox", "dualshock",
        "dualsense", "wingman", "sidewinder", "thrustmaster", "joypad",
        "flightstick", "hotas", "wheel",
    )
    return any(k in haystack.lower() for k in keywords)
