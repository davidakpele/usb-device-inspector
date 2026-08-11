"""Device scanning orchestration (spec sections 9, 10, 14).

``DeviceScanner`` is the single entry point for producing a ``DeviceDetails``
object from a ``USBDevice``. It:

1. Enriches the device with live driver info (via usb_enumerator) before
   handing it to an inspector, so inspectors always receive the most
   up-to-date data.
2. Dispatches to the correct ``BaseInspector`` subclass via an internal
   registry keyed on ``DeviceCategory``.
3. Falls back to a ``GenericInspector`` for categories with no specialist,
   so every device always produces a complete set of standard sections.

Adding a new device-type inspector only requires:
  * Creating the class (subclass of BaseInspector with ``categories`` set)
  * Importing it here and adding to ``_INSPECTOR_REGISTRY``

Nothing else in the application needs to change.
"""
from __future__ import annotations

from app.inspectors.base_inspector import BaseInspector
from app.inspectors.camera_inspector import CameraInspector
from app.inspectors.controller_inspector import ControllerInspector
from app.inspectors.hid_inspector import HIDInspector
from app.inspectors.serial_inspector import SerialInspector
from app.inspectors.storage_inspector import StorageInspector
from app.models.device_details import DeviceDetails
from app.models.usb_device import USBDevice
from app.usb.usb_enumerator import get_driver_info
from app.utils.logger import get_logger

logger = get_logger(__name__)

# Registry: DeviceCategory.value -> inspector instance (singletons are fine,
# inspectors hold no per-device state).
_INSPECTOR_REGISTRY: dict[str, BaseInspector] = {}


def _build_registry() -> dict[str, BaseInspector]:
    instances: list[BaseInspector] = [
        StorageInspector(),
        HIDInspector(),
        ControllerInspector(),
        CameraInspector(),
        SerialInspector(),
    ]
    registry: dict[str, BaseInspector] = {}
    for inspector in instances:
        for category in inspector.categories:
            registry[category] = inspector
    return registry


_INSPECTOR_REGISTRY = _build_registry()


class _GenericInspector(BaseInspector):
    """Fallback inspector: builds standard sections only (spec section 9)."""

    categories = ()  # registered dynamically — not a fixed category

    def inspect(self, device: USBDevice) -> DeviceDetails:
        details = DeviceDetails(device=device)
        self.build_general_section(details, device)
        self.build_usb_information_section(details, device)
        self.build_hardware_section(details, device)
        self.build_driver_section(details, device)
        return details


_GENERIC_INSPECTOR = _GenericInspector()


class DeviceScanner:
    """Synchronous scan execution — run inside a worker thread (ScanningService).

    ``scan`` is the *only* public method. It is intentionally synchronous
    because threading is managed at the service layer (QRunnable), not here.
    """

    def scan(self, device: USBDevice) -> DeviceDetails:
        """Perform a full inspection of *device* and return a DeviceDetails.

        Never raises for device-specific failures — any unexpected exception
        is caught, logged, and surfaced as a warning in the returned details
        so the UI can display a user-friendly message (spec section 15).
        """
        logger.info("Scan started: %s", device.device_id)

        # Enrich with fresh driver info before inspecting.
        self._enrich_driver_info(device)

        inspector = _INSPECTOR_REGISTRY.get(device.category, _GENERIC_INSPECTOR)
        logger.debug("Using %s for category '%s'", type(inspector).__name__, device.category)

        try:
            details = inspector.inspect(device)
        except Exception as exc:  # noqa: BLE001
            logger.exception("Inspector raised unexpectedly for %s: %s", device.device_id, exc)
            details = _GENERIC_INSPECTOR.inspect(device)
            details.warnings.append(
                f"Some device information could not be retrieved: {exc}. "
                "The device may have been disconnected during scanning."
            )

        logger.info("Scan completed: %s (%d sections)", device.device_id, len(details.sections))
        return details

    # ------------------------------------------------------------------
    @staticmethod
    def _enrich_driver_info(device: USBDevice) -> None:
        """Fetch and attach driver metadata if not already present."""
        if device.driver_version:
            return  # already populated (e.g. from a previous scan)
        try:
            info = get_driver_info(device.device_id)
            device.driver_name = device.driver_name or info.get("driver_name")
            device.driver_provider = device.driver_provider or info.get("driver_provider")
            device.driver_version = device.driver_version or info.get("driver_version")
            device.driver_date = device.driver_date or info.get("driver_date")
        except Exception as exc:  # noqa: BLE001
            logger.debug("Driver enrichment failed for %s: %s", device.device_id, exc)
