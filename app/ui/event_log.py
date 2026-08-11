"""Real-time event log panel (spec section 13).

Displays timestamped USB connection/disconnection events and application
messages in a scrolling, read-only text area at the bottom of the main
window. Events are colour-coded by type for quick visual scanning.
"""
from __future__ import annotations

from PySide6.QtCore import Qt, Slot
from PySide6.QtGui import QColor, QTextCharFormat, QTextCursor
from PySide6.QtWidgets import (
    QHBoxLayout,
    QLabel,
    QPushButton,
    QSizePolicy,
    QTextEdit,
    QVBoxLayout,
    QWidget,
)

from app.models.device_event import DeviceEvent, DeviceEventType

# Colour palette (dark-friendly muted tones)
_COLOURS: dict[DeviceEventType, str] = {
    DeviceEventType.CONNECTED:      "#4CAF50",  # green
    DeviceEventType.REMOVED:        "#EF5350",  # red
    DeviceEventType.SCAN_STARTED:   "#42A5F5",  # blue
    DeviceEventType.SCAN_COMPLETED: "#66BB6A",  # light green
    DeviceEventType.SCAN_FAILED:    "#FFA726",  # orange
    DeviceEventType.ERROR:          "#EF5350",  # red
    DeviceEventType.INFO:           "#B0BEC5",  # grey-blue
}
_DEFAULT_COLOUR = "#B0BEC5"
_MAX_EVENTS = 500  # cap to avoid memory growth


class EventLogWidget(QWidget):
    """Scrolling event log with timestamp, type icon, and message."""

    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self._event_count = 0
        self._setup_ui()

    # ------------------------------------------------------------------
    # Construction
    # ------------------------------------------------------------------

    def _setup_ui(self) -> None:
        root = QVBoxLayout(self)
        root.setContentsMargins(0, 4, 0, 0)
        root.setSpacing(4)

        # Header row
        header = QHBoxLayout()
        title = QLabel("Device Events")
        title.setObjectName("sectionLabel")
        header.addWidget(title)
        header.addStretch()

        self._clear_btn = QPushButton("Clear Log")
        self._clear_btn.setObjectName("smallButton")
        self._clear_btn.setFixedHeight(24)
        self._clear_btn.clicked.connect(self.clear)
        header.addWidget(self._clear_btn)

        root.addLayout(header)

        # Log area
        self._log = QTextEdit()
        self._log.setReadOnly(True)
        self._log.setObjectName("eventLog")
        self._log.setLineWrapMode(QTextEdit.LineWrapMode.NoWrap)
        self._log.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        self._log.setMinimumHeight(100)
        root.addWidget(self._log)

    # ------------------------------------------------------------------
    # Public slots
    # ------------------------------------------------------------------

    @Slot(object)
    def append_event(self, event: DeviceEvent) -> None:
        """Append a DeviceEvent to the log. Safe to call from any thread
        via a connected Qt signal.
        """
        if self._event_count >= _MAX_EVENTS:
            # Drop oldest quarter of events to keep memory bounded.
            self._trim()

        colour = _COLOURS.get(event.event_type, _DEFAULT_COLOUR)
        timestamp = event.timestamp.strftime("%H:%M:%S")
        icon = _event_icon(event.event_type)
        line = f"{timestamp}  {icon}  {event.message}"

        cursor = self._log.textCursor()
        cursor.movePosition(QTextCursor.MoveOperation.End)

        fmt = QTextCharFormat()
        fmt.setForeground(QColor(colour))
        cursor.insertText(line + "\n", fmt)

        self._log.setTextCursor(cursor)
        self._log.ensureCursorVisible()
        self._event_count += 1

    @Slot()
    def clear(self) -> None:
        self._log.clear()
        self._event_count = 0

    # ------------------------------------------------------------------
    # Internals
    # ------------------------------------------------------------------

    def _trim(self) -> None:
        """Remove the oldest ~25 % of lines to stay under _MAX_EVENTS."""
        doc = self._log.document()
        target_remove = _MAX_EVENTS // 4
        cursor = QTextCursor(doc)
        cursor.movePosition(QTextCursor.MoveOperation.Start)
        for _ in range(target_remove):
            cursor.select(QTextCursor.SelectionType.LineUnderCursor)
            cursor.removeSelectedText()
            cursor.deleteChar()  # remove the trailing newline
        self._event_count -= target_remove


def _event_icon(event_type: DeviceEventType) -> str:
    icons = {
        DeviceEventType.CONNECTED:      "↑",
        DeviceEventType.REMOVED:        "↓",
        DeviceEventType.SCAN_STARTED:   "⟳",
        DeviceEventType.SCAN_COMPLETED: "✓",
        DeviceEventType.SCAN_FAILED:    "✗",
        DeviceEventType.ERROR:          "!",
        DeviceEventType.INFO:           "·",
    }
    return icons.get(event_type, "·")
