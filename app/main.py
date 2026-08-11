"""Application entry point (spec sections 3, 18, 25 - Phase 1).

Start with:
    python -m app.main
or:
    python app/main.py

Windows 10/11 only — ``require_windows()`` aborts with a clear message on
other platforms before any Windows-specific import occurs.
"""
from __future__ import annotations

import sys

from app.utils.logger import configure_logging, get_logger
from app.utils.platform import require_windows


def main() -> int:
    # Configure logging before any other module is imported, so all early
    # log calls are captured.
    configure_logging()
    logger = get_logger(__name__)
    logger.info("USB Device Inspector starting")

    # Abort cleanly on non-Windows platforms.
    try:
        require_windows()
    except RuntimeError as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1

    # PySide6 import deferred until after platform check so tests running on
    # Linux/macOS can still exercise non-UI modules.
    from PySide6.QtCore import Qt
    from PySide6.QtWidgets import QApplication

    from app.ui.main_window import MainWindow

    # Ensure high-DPI scaling behaves consistently on Windows 10/11.
    QApplication.setHighDpiScaleFactorRoundingPolicy(
        Qt.HighDpiScaleFactorRoundingPolicy.PassThrough
    )

    app = QApplication(sys.argv)
    app.setApplicationName("USB Device Inspector")
    app.setOrganizationName("USBDeviceInspector")

    window = MainWindow()
    window.show()

    logger.info("Application event loop started")
    exit_code = app.exec()
    logger.info("Application exited with code %d", exit_code)
    return exit_code


if __name__ == "__main__":
    sys.exit(main())
