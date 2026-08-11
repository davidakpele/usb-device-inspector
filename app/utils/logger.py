"""Structured, rotating application logging (spec section 18).

Deliberately avoids logging raw serial numbers or full hardware IDs at INFO
level (they can be device-identifying); those are only emitted at DEBUG.
"""
from __future__ import annotations

import logging
import logging.handlers
from pathlib import Path

_LOG_DIR = Path.home() / "AppData" / "Local" / "USBDeviceInspector" / "logs"
_LOG_FORMAT = "%(asctime)s | %(levelname)-8s | %(name)s | %(message)s"

_configured = False


def configure_logging(level: int = logging.INFO) -> None:
    """Idempotently configure root application logging.

    Safe to call multiple times (e.g. from tests) — only configures once.
    """
    global _configured
    if _configured:
        return

    try:
        _LOG_DIR.mkdir(parents=True, exist_ok=True)
        log_file = _LOG_DIR / "usb_inspector.log"
        file_handler: logging.Handler = logging.handlers.RotatingFileHandler(
            log_file, maxBytes=5 * 1024 * 1024, backupCount=3, encoding="utf-8"
        )
    except OSError:
        # Fall back to console-only logging if the log directory is
        # unavailable (e.g. restricted environment) - never crash the app
        # over logging setup.
        file_handler = logging.NullHandler()

    formatter = logging.Formatter(_LOG_FORMAT)
    file_handler.setFormatter(formatter)

    console_handler = logging.StreamHandler()
    console_handler.setFormatter(formatter)

    root = logging.getLogger("usb_inspector")
    root.setLevel(level)
    root.addHandler(file_handler)
    root.addHandler(console_handler)
    root.propagate = False

    _configured = True


def get_logger(name: str) -> logging.Logger:
    """Return a namespaced child logger, e.g. get_logger(__name__)."""
    configure_logging()
    return logging.getLogger("usb_inspector").getChild(name)