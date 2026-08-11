"""Device list widget — the main dashboard table (spec sections 7, 12, 26).

Displays currently connected USB devices in a table with columns:
  Status indicator | Name | Category | VID | PID | Status text

Emits ``device_selected(USBDevice)`` when the user clicks a row.
"""
from __future__ import annotations

from PySide6.QtCore import Qt, Signal, Slot
from PySide6.QtGui import QColor, QFont
from PySide6.QtWidgets import (
    QAbstractItemView,
    QHBoxLayout,
    QLabel,
    QSizePolicy,
    QTableWidget,
    QTableWidgetItem,
    QVBoxLayout,
    QWidget,
)

from app.models.usb_device import ConnectionStatus, USBDevice

_COLUMNS = ["", "Device Name", "Category", "VID", "PID", "Status"]
_COL_DOT = 0
_COL_NAME = 1
_COL_CAT = 2
_COL_VID = 3
_COL_PID = 4
_COL_STATUS = 5

_STATUS_COLOURS: dict[ConnectionStatus, str] = {
    ConnectionStatus.CONNECTED:    "#4CAF50",
    ConnectionStatus.DISCONNECTED: "#EF5350",
    ConnectionStatus.ERROR:        "#FFA726",
    ConnectionStatus.UNKNOWN:      "#90A4AE",
}


class DeviceListWidget(QWidget):
    """Tabular list of USB devices with connection-state colour indicators."""

    device_selected = Signal(object)   # USBDevice

    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self._devices: list[USBDevice] = []
        self._setup_ui()

    # ------------------------------------------------------------------
    # Construction
    # ------------------------------------------------------------------

    def _setup_ui(self) -> None:
        root = QVBoxLayout(self)
        root.setContentsMargins(0, 0, 0, 0)
        root.setSpacing(6)

        # Summary label
        self._count_label = QLabel("Connected Devices: 0")
        self._count_label.setObjectName("sectionLabel")
        root.addWidget(self._count_label)

        # Table
        self._table = QTableWidget()
        self._table.setColumnCount(len(_COLUMNS))
        self._table.setHorizontalHeaderLabels(_COLUMNS)
        self._table.setSelectionBehavior(QAbstractItemView.SelectionBehavior.SelectRows)
        self._table.setSelectionMode(QAbstractItemView.SelectionMode.SingleSelection)
        self._table.setEditTriggers(QAbstractItemView.EditTrigger.NoEditTriggers)
        self._table.setAlternatingRowColors(True)
        self._table.verticalHeader().setVisible(False)
        self._table.setShowGrid(False)
        self._table.setSortingEnabled(True)
        self._table.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)

        # Column sizing
        hdr = self._table.horizontalHeader()
        hdr.setDefaultAlignment(Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter)
        self._table.setColumnWidth(_COL_DOT, 20)
        self._table.setColumnWidth(_COL_NAME, 240)
        self._table.setColumnWidth(_COL_CAT, 130)
        self._table.setColumnWidth(_COL_VID, 70)
        self._table.setColumnWidth(_COL_PID, 70)
        hdr.setStretchLastSection(True)

        self._table.itemSelectionChanged.connect(self._on_selection_changed)
        root.addWidget(self._table)

    # ------------------------------------------------------------------
    # Public
    # ------------------------------------------------------------------

    @Slot(list)
    def refresh(self, devices: list[USBDevice]) -> None:
        """Repopulate the table from a fresh device list."""
        self._devices = list(devices)

        # Suppress signals while rebuilding to avoid spurious selections.
        self._table.blockSignals(True)
        self._table.setSortingEnabled(False)
        self._table.setRowCount(0)

        for row, device in enumerate(self._devices):
            self._table.insertRow(row)
            self._populate_row(row, device)

        self._table.setSortingEnabled(True)
        self._table.blockSignals(False)

        connected = sum(1 for d in self._devices if d.connected)
        self._count_label.setText(f"Connected Devices: {connected}")

    def selected_device(self) -> USBDevice | None:
        """Return the currently selected USBDevice or None."""
        rows = self._table.selectedItems()
        if not rows:
            return None
        row = self._table.currentRow()
        if row < 0 or row >= len(self._devices):
            return None
        return self._devices[row]

    def select_device_by_id(self, device_id: str) -> None:
        """Restore selection to *device_id* after a refresh."""
        for row, device in enumerate(self._devices):
            if device.device_id == device_id:
                self._table.selectRow(row)
                return

    # ------------------------------------------------------------------
    # Internal
    # ------------------------------------------------------------------

    def _populate_row(self, row: int, device: USBDevice) -> None:
        colour_hex = _STATUS_COLOURS.get(device.status, "#90A4AE")
        colour = QColor(colour_hex)

        # Dot indicator
        dot = QTableWidgetItem("●")
        dot.setForeground(colour)
        dot.setTextAlignment(Qt.AlignmentFlag.AlignCenter)
        dot.setFlags(Qt.ItemFlag.ItemIsEnabled | Qt.ItemFlag.ItemIsSelectable)
        self._table.setItem(row, _COL_DOT, dot)

        name_text = device.name or device.device_id
        name_item = QTableWidgetItem(name_text)
        name_font = QFont()
        name_font.setBold(True)
        name_item.setFont(name_font)
        name_item.setFlags(Qt.ItemFlag.ItemIsEnabled | Qt.ItemFlag.ItemIsSelectable)
        self._table.setItem(row, _COL_NAME, name_item)

        for col, text in (
            (_COL_CAT,    device.category or "Unknown"),
            (_COL_VID,    device.vendor_id or "—"),
            (_COL_PID,    device.product_id or "—"),
            (_COL_STATUS, device.status.value),
        ):
            item = QTableWidgetItem(text)
            item.setFlags(Qt.ItemFlag.ItemIsEnabled | Qt.ItemFlag.ItemIsSelectable)
            if col == _COL_STATUS:
                item.setForeground(colour)
            self._table.setItem(row, col, item)

        self._table.setRowHeight(row, 30)

    def _on_selection_changed(self) -> None:
        device = self.selected_device()
        if device is not None:
            self.device_selected.emit(device)
