"""Persistent device history service (spec section 11).

History is stored as a JSON file in the user's local app-data directory.
Only non-sensitive metadata is persisted: device_id, name, category,
vendor_id, product_id, first_seen, last_seen. Serial numbers are NOT
written to disk (spec section 19 - security/privacy).

Thread safety: all public methods acquire a lock, so the service is safe
to call from the monitor thread and the UI thread simultaneously.
"""
from __future__ import annotations

import json
import threading
from datetime import datetime
from pathlib import Path
from typing import Any

from app.models.usb_device import USBDevice
from app.utils.logger import get_logger

logger = get_logger(__name__)

_HISTORY_DIR = Path.home() / "AppData" / "Local" / "USBDeviceInspector"
_HISTORY_FILE = _HISTORY_DIR / "device_history.json"
_DT_FORMAT = "%Y-%m-%dT%H:%M:%S"


def _dt_str(dt: datetime | None) -> str | None:
    return dt.strftime(_DT_FORMAT) if dt else None


def _dt_parse(s: str | None) -> datetime | None:
    if not s:
        return None
    try:
        return datetime.strptime(s, _DT_FORMAT)
    except (ValueError, TypeError):
        return None


class HistoryService:
    """Read/write device history to a local JSON file.

    Each entry is keyed by ``device_id`` and stores:
      device_id, name, category, vendor_id, product_id,
      first_seen (ISO-8601), last_seen (ISO-8601)
    """

    def __init__(self) -> None:
        self._lock = threading.RLock()
        # In-memory cache: device_id -> record dict
        self._records: dict[str, dict[str, Any]] = {}
        self._load()

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def record(self, device: USBDevice) -> None:
        """Update (or create) the history entry for *device*."""
        with self._lock:
            existing = self._records.get(device.device_id)
            if existing:
                existing["name"] = device.name or existing.get("name")
                existing["category"] = device.category or existing.get("category")
                existing["last_seen"] = _dt_str(device.last_seen or datetime.now())
            else:
                self._records[device.device_id] = {
                    "device_id": device.device_id,
                    "name": device.name,
                    "category": device.category,
                    "vendor_id": device.vendor_id,
                    "product_id": device.product_id,
                    "first_seen": _dt_str(device.first_seen or datetime.now()),
                    "last_seen": _dt_str(device.last_seen or datetime.now()),
                }
            self._save()

    def all_entries(self) -> list[dict[str, Any]]:
        """Return all history entries, sorted newest-first by last_seen."""
        with self._lock:
            entries = list(self._records.values())
        entries.sort(key=lambda e: e.get("last_seen") or "", reverse=True)
        return entries

    def clear(self) -> None:
        """Delete all history records and remove the persisted file."""
        with self._lock:
            self._records.clear()
            try:
                _HISTORY_FILE.unlink(missing_ok=True)
            except OSError as exc:
                logger.warning("Could not delete history file: %s", exc)
        logger.info("Device history cleared")

    def count(self) -> int:
        with self._lock:
            return len(self._records)

    # ------------------------------------------------------------------
    # Persistence helpers
    # ------------------------------------------------------------------

    def _load(self) -> None:
        if not _HISTORY_FILE.exists():
            return
        try:
            with _HISTORY_FILE.open(encoding="utf-8") as fh:
                data = json.load(fh)
            if isinstance(data, list):
                for entry in data:
                    did = entry.get("device_id")
                    if did:
                        self._records[did] = entry
            logger.info("Loaded %d history entries from %s", len(self._records), _HISTORY_FILE)
        except (json.JSONDecodeError, OSError) as exc:
            logger.warning("Could not load device history: %s", exc)

    def _save(self) -> None:
        try:
            _HISTORY_DIR.mkdir(parents=True, exist_ok=True)
            with _HISTORY_FILE.open("w", encoding="utf-8") as fh:
                json.dump(list(self._records.values()), fh, indent=2, ensure_ascii=False)
        except OSError as exc:
            logger.warning("Could not save device history: %s", exc)
