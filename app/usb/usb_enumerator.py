"""Windows USB enumeration via WMI (Win32_PnPEntity).

Why WMI over raw SetupAPI / pyusb for the primary data source:

* ``Win32_PnPEntity`` already aggregates the same information SetupAPI
  exposes (hardware IDs, compatible IDs, driver service, PNPClass, status)
  through a stable, well-documented COM interface with a mature Python
  binding (``pywin32``'s ``win32com`` / the ``wmi`` package), instead of us
  hand-rolling ctypes bindings for ``SetupDiGetDeviceProperty`` and friends.
* It works for every PnP-managed USB device class out of the box (storage,
  HID, printers, cameras, serial, hubs) without per-class native code.
* It does not require rebinding a device to WinUSB the way ``pyusb``
  (libusb backend) would to read descriptors directly — critical given the
  spec's read-only, non-invasive requirement (section 19): rebinding a
  mouse or storage device to libusb would break its normal Windows driver.
* Driver metadata (provider/version/date) is most reliably obtained by
  joining ``Win32_PnPEntity`` to ``Win32_PnPSignedDriver`` on DeviceID,
  which only WMI conveniently exposes.

This module is the **single seam** between the OS and the rest of the
application (see utils/platform.py) — nothing outside app/usb/ should
import ``wmi``/``win32com`` directly.
"""
from __future__ import annotations

from typing import Iterable

from app.usb.usb_descriptor import RawPnPDescriptor
from app.utils.logger import get_logger

logger = get_logger(__name__)

# USB PNPDeviceIDs always start with "USB\\"; USBSTOR entries (mass storage
# volumes) start with "USBSTOR\\" and are joined back to their parent USB
# device separately by the storage inspector.
_USB_ID_PREFIXES = ("USB\\", "USBSTOR\\")


class WMIUnavailableError(RuntimeError):
    """Raised when WMI cannot be reached (service down, permissions, etc.)."""


def _connect():
    """Create a WMI connection. Isolated for testability (mockable)."""
    try:
        import wmi  # type: ignore  # pywin32 + WMI, Windows-only
    except ImportError as exc:  # pragma: no cover - exercised only off-Windows
        raise WMIUnavailableError(
            "The 'wmi' package (and pywin32) is required on Windows. "
            "Install with: pip install wmi pywin32"
        ) from exc

    try:
        return wmi.WMI()
    except Exception as exc:  # noqa: BLE001 - WMI raises generic COM errors
        raise WMIUnavailableError(f"Could not connect to WMI: {exc}") from exc


def enumerate_usb_descriptors() -> list[RawPnPDescriptor]:
    """Enumerate all currently-present USB PnP entities.

    Returns an empty list (never raises) if WMI is unavailable, logging the
    failure — a transient WMI outage must not crash device listing entirely
    (spec section 15: "Never terminate the application because one device
    cannot be inspected").
    """
    try:
        conn = _connect()
    except WMIUnavailableError as exc:
        logger.error("USB enumeration unavailable: %s", exc)
        return []

    descriptors: list[RawPnPDescriptor] = []
    try:
        entities = conn.Win32_PnPEntity()
    except Exception as exc:  # noqa: BLE001
        logger.error("WMI query for Win32_PnPEntity failed: %s", exc)
        return []

    for entity in entities:
        device_id = getattr(entity, "PNPDeviceID", None) or getattr(entity, "DeviceID", None)
        if not device_id or not device_id.upper().startswith(_USB_ID_PREFIXES):
            continue
        try:
            descriptors.append(_entity_to_descriptor(entity, device_id))
        except Exception as exc:  # noqa: BLE001 - one bad device must not abort the rest
            logger.warning("Failed to read PnP entity %s: %s", device_id, exc)
            continue

    logger.info("Enumerated %d USB PnP entities", len(descriptors))
    return descriptors


def _entity_to_descriptor(entity, device_id: str) -> RawPnPDescriptor:
    hardware_ids = list(getattr(entity, "HardwareID", None) or [])
    compatible_ids = list(getattr(entity, "CompatibleID", None) or [])
    config_error = getattr(entity, "ConfigManagerErrorCode", None)
    status = "OK" if config_error == 0 else f"Error code {config_error}" if config_error is not None else None

    return RawPnPDescriptor(
        device_id=device_id,
        name=getattr(entity, "Name", None),
        description=getattr(entity, "Description", None),
        manufacturer=getattr(entity, "Manufacturer", None),
        pnp_class=getattr(entity, "PNPClass", None),
        class_guid=getattr(entity, "ClassGuid", None),
        hardware_ids=hardware_ids,
        compatible_ids=compatible_ids,
        service=getattr(entity, "Service", None),
        status=status,
        present=bool(getattr(entity, "Present", True)),
    )


def get_driver_info(device_id: str) -> dict[str, str | None]:
    """Look up driver provider/version/date for a device via Win32_PnPSignedDriver.

    Returns a dict with keys driver_name, driver_provider, driver_version,
    driver_date — any of which may be None if not reported.
    """
    result: dict[str, str | None] = {
        "driver_name": None,
        "driver_provider": None,
        "driver_version": None,
        "driver_date": None,
    }
    try:
        conn = _connect()
        drivers = conn.Win32_PnPSignedDriver(DeviceID=device_id)
    except Exception as exc:  # noqa: BLE001
        logger.debug("Driver lookup failed for %s: %s", device_id, exc)
        return result

    if not drivers:
        return result

    drv = drivers[0]
    result["driver_name"] = getattr(drv, "DeviceName", None)
    result["driver_provider"] = getattr(drv, "DriverProviderName", None)
    result["driver_version"] = getattr(drv, "DriverVersion", None)
    result["driver_date"] = getattr(drv, "DriverDate", None)
    return result


def get_storage_volume_info(pnp_device_id: str) -> Iterable[dict]:
    """Join Win32_DiskDrive -> Win32_DiskPartition -> Win32_LogicalDisk.

    Used by StorageInspector to resolve drive letter/filesystem/capacity for
    a USB mass-storage device's PNPDeviceID. Returns one dict per logical
    volume found (a device may expose multiple partitions).
    """
    try:
        conn = _connect()
        disks = conn.Win32_DiskDrive(PNPDeviceID=pnp_device_id)
    except Exception as exc:  # noqa: BLE001
        logger.debug("Disk lookup failed for %s: %s", pnp_device_id, exc)
        return []

    volumes: list[dict] = []
    for disk in disks:
        try:
            partitions = disk.associators(wmi_result_class="Win32_DiskPartition")
        except Exception as exc:  # noqa: BLE001
            logger.debug("Partition lookup failed for %s: %s", pnp_device_id, exc)
            continue
        for partition in partitions:
            try:
                logical_disks = partition.associators(wmi_result_class="Win32_LogicalDisk")
            except Exception as exc:  # noqa: BLE001
                logger.debug("Logical disk lookup failed: %s", exc)
                continue
            for ld in logical_disks:
                volumes.append(
                    {
                        "drive_letter": getattr(ld, "DeviceID", None),
                        "volume_name": getattr(ld, "VolumeName", None),
                        "file_system": getattr(ld, "FileSystem", None),
                        "capacity_bytes": getattr(ld, "Size", None),
                        "free_bytes": getattr(ld, "FreeSpace", None),
                        "disk_model": getattr(disk, "Model", None),
                        "disk_serial": getattr(disk, "SerialNumber", None),
                        "removable": getattr(disk, "MediaType", "") == "Removable Media",
                    }
                )
    return volumes