"""Platform detection so the future Linux/macOS backends can plug in cleanly.

The spec targets Windows 10/11 only, but core/ and ui/ must never import
Windows-only modules directly — they go through usb/usb_enumerator.py,
which is the single seam a future LinuxUSBEnumerator would replace.
"""
from __future__ import annotations

import platform
import sys


def is_windows() -> bool:
    return sys.platform == "win32"


def is_windows_10_or_11() -> bool:
    if not is_windows():
        return False
    release = platform.release()
    return release in {"10", "11"}


def require_windows() -> None:
    """Raise a clear error if run on an unsupported OS.

    Called once at startup (main.py) rather than scattered through the
    codebase, so the failure mode is a single, understandable message
    instead of a stack trace from deep inside a WMI call.
    """
    if not is_windows():
        raise RuntimeError(
            "USB Device Inspector currently supports Windows 10/11 only. "
            f"Detected platform: {sys.platform}. "
            "The architecture separates the Windows-specific enumeration "
            "layer (app/usb/) from the rest of the app so Linux/macOS "
            "support can be added later without touching the UI or "
            "inspection logic."
        )