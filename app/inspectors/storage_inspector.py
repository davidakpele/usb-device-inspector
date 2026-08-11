"""Inspector for USB mass-storage devices (spec section 9 - USB Storage)."""
from __future__ import annotations

from app.inspectors.base_inspector import BaseInspector
from app.models.device_details import DeviceDetails
from app.models.usb_device import FieldSource, USBDevice
from app.usb.usb_constants import DeviceCategory
from app.usb.usb_enumerator import get_storage_volume_info
from app.utils.logger import get_logger

logger = get_logger(__name__)

_BYTES_PER_GB = 1024 ** 3


def _fmt_bytes(value) -> str | None:
    if value is None:
        return None
    try:
        gb = int(value) / _BYTES_PER_GB
        return f"{gb:.2f} GB"
    except (TypeError, ValueError):
        return None


class StorageInspector(BaseInspector):
    categories = (DeviceCategory.STORAGE.value,)

    def inspect(self, device: USBDevice) -> DeviceDetails:
        details = DeviceDetails(device=device)
        self.build_general_section(details, device)
        self.build_usb_information_section(details, device)
        self.build_hardware_section(details, device)
        self.build_driver_section(details, device)

        try:
            volumes = list(get_storage_volume_info(device.device_id))
        except Exception as exc:  # noqa: BLE001
            logger.warning("Storage volume lookup failed for %s: %s", device.device_id, exc)
            volumes = []
            details.warnings.append(
                "Some device information could not be retrieved. "
                "The device may not expose this information to Windows."
            )

        section = details.get_or_create_section("Storage")
        if not volumes:
            section.add("Drive Letter", None)
            section.add("Volume Name", None)
            section.add("File System", None)
            section.add("Capacity", None)
            section.add("Free Space", None)
            section.add("Used Space", None)
        else:
            # A device may expose multiple partitions; show the first as the
            # primary and note additional ones rather than silently dropping.
            primary = volumes[0]
            capacity = primary.get("capacity_bytes")
            free = primary.get("free_bytes")
            used = None
            if capacity is not None and free is not None:
                try:
                    used = int(capacity) - int(free)
                except (TypeError, ValueError):
                    used = None

            section.add("Drive Letter", primary.get("drive_letter"), source="Directly Reported")
            section.add("Volume Name", primary.get("volume_name"), source="Directly Reported")
            section.add("File System", primary.get("file_system"), source="Directly Reported")
            section.add("Capacity", _fmt_bytes(capacity), source="Detected")
            section.add("Free Space", _fmt_bytes(free), source="Detected")
            section.add("Used Space", _fmt_bytes(used), source="Detected")
            section.add(
                "Removable",
                "Yes" if primary.get("removable") else "No" if primary.get("removable") is not None else None,
            )
            section.add("Disk Model", primary.get("disk_model"), source="Directly Reported")
            section.add("Disk Serial Number", primary.get("disk_serial"), source="Directly Reported")

            if len(volumes) > 1:
                extra = ", ".join(v.get("drive_letter") or "?" for v in volumes[1:])
                details.warnings.append(f"Device also exposes additional partitions: {extra}")

        self.add_capability_if(
            details, bool(volumes), "USB Storage", evidence="Resolved logical disk volume(s)"
        )
        self.add_capability_if(
            details,
            any(v.get("removable") for v in volumes),
            "Removable Device",
            evidence="Win32_DiskDrive.MediaType == 'Removable Media'",
        )
        return details