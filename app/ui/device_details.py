"""Device details panel (spec sections 8, 9, 26).

Displays the full DeviceDetails result produced by an inspector:
  * Each DetailSection becomes a tab in a QTabWidget
  * Each DetailField becomes a two-column row (label | value + source badge)
  * DeviceCapability list shown on a dedicated "Capabilities" tab
  * Warnings shown in a collapsible area at the bottom

The widget is read-only and never calls any Windows API directly.
"""
from __future__ import annotations

from PySide6.QtCore import Qt
from PySide6.QtGui import QColor, QFont
from PySide6.QtWidgets import (
    QFrame,
    QHBoxLayout,
    QLabel,
    QScrollArea,
    QSizePolicy,
    QTabWidget,
    QTextEdit,
    QVBoxLayout,
    QWidget,
)

from app.models.device_capability import DeviceCapability
from app.models.device_details import DetailSection, DeviceDetails
from app.models.usb_device import FieldSource

_SOURCE_COLOURS: dict[str, str] = {
    FieldSource.DIRECTLY_REPORTED.value: "#4CAF50",
    FieldSource.DETECTED.value:          "#42A5F5",
    FieldSource.DERIVED.value:           "#FFA726",
    FieldSource.UNKNOWN.value:           "#78909C",
    "Unknown":                           "#78909C",
}


class DeviceDetailsWidget(QWidget):
    """Shows a DeviceDetails in a tabbed layout. Call ``show_details()`` to populate."""

    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self._setup_ui()

    # ------------------------------------------------------------------
    # Construction
    # ------------------------------------------------------------------

    def _setup_ui(self) -> None:
        root = QVBoxLayout(self)
        root.setContentsMargins(0, 0, 0, 0)
        root.setSpacing(6)

        self._tabs = QTabWidget()
        self._tabs.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        root.addWidget(self._tabs)

        # Warnings area (hidden when empty)
        self._warnings_label = QLabel()
        self._warnings_label.setObjectName("warningLabel")
        self._warnings_label.setWordWrap(True)
        self._warnings_label.hide()
        root.addWidget(self._warnings_label)

    # ------------------------------------------------------------------
    # Public
    # ------------------------------------------------------------------

    def show_details(self, details: DeviceDetails) -> None:
        """Populate the widget from a DeviceDetails result."""
        self._tabs.clear()

        for section in details.sections:
            tab = self._build_section_tab(section)
            self._tabs.addTab(tab, section.title)

        if details.capabilities:
            cap_tab = self._build_capabilities_tab(details.capabilities)
            self._tabs.addTab(cap_tab, "Capabilities")

        if details.warnings:
            self._warnings_label.setText(
                "⚠  " + "\n⚠  ".join(details.warnings)
            )
            self._warnings_label.show()
        else:
            self._warnings_label.hide()

    def clear(self) -> None:
        self._tabs.clear()
        self._warnings_label.hide()

    # ------------------------------------------------------------------
    # Section tab
    # ------------------------------------------------------------------

    def _build_section_tab(self, section: DetailSection) -> QWidget:
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QFrame.Shape.NoFrame)

        container = QWidget()
        layout = QVBoxLayout(container)
        layout.setSpacing(2)
        layout.setContentsMargins(12, 8, 12, 8)

        for field in section.fields:
            row = self._build_field_row(field.label, field.display_value(), field.source)
            layout.addWidget(row)

        layout.addStretch()
        scroll.setWidget(container)
        return scroll

    def _build_field_row(self, label: str, value: str, source: str) -> QWidget:
        row = QWidget()
        h = QHBoxLayout(row)
        h.setContentsMargins(0, 2, 0, 2)
        h.setSpacing(8)

        lbl = QLabel(f"{label}:")
        lbl.setFixedWidth(180)
        lbl.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
        lbl_font = QFont()
        lbl_font.setBold(True)
        lbl.setFont(lbl_font)
        lbl.setObjectName("fieldLabel")
        h.addWidget(lbl)

        val_text = value if value else "Not Available"
        val = QLabel(val_text)
        val.setObjectName("fieldValue")
        val.setTextInteractionFlags(Qt.TextInteractionFlag.TextSelectableByMouse)
        val.setWordWrap(True)
        val.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Preferred)
        h.addWidget(val)

        # Source badge
        source_colour = _SOURCE_COLOURS.get(source, _SOURCE_COLOURS["Unknown"])
        badge = QLabel(source)
        badge.setObjectName("sourceBadge")
        badge.setStyleSheet(
            f"color: {source_colour}; font-size: 10px; border: 1px solid {source_colour};"
            "border-radius: 3px; padding: 0px 4px;"
        )
        badge.setFixedHeight(16)
        badge.setAlignment(Qt.AlignmentFlag.AlignCenter)
        h.addWidget(badge)

        return row

    # ------------------------------------------------------------------
    # Capabilities tab
    # ------------------------------------------------------------------

    def _build_capabilities_tab(self, capabilities: list[DeviceCapability]) -> QWidget:
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QFrame.Shape.NoFrame)

        container = QWidget()
        layout = QVBoxLayout(container)
        layout.setSpacing(6)
        layout.setContentsMargins(12, 8, 12, 8)

        for cap in capabilities:
            row = QWidget()
            h = QHBoxLayout(row)
            h.setContentsMargins(0, 0, 0, 0)
            h.setSpacing(10)

            tick = QLabel("✓")
            tick.setFixedWidth(20)
            tick.setStyleSheet("color: #4CAF50; font-weight: bold; font-size: 14px;")
            h.addWidget(tick)

            label = QLabel(cap.label)
            label.setObjectName("capabilityLabel")
            cap_font = QFont()
            cap_font.setBold(True)
            label.setFont(cap_font)
            h.addWidget(label)

            evidence = QLabel(f"  [{cap.evidence}]")
            evidence.setObjectName("evidenceLabel")
            evidence.setStyleSheet("color: #78909C; font-size: 11px;")
            evidence.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Preferred)
            h.addWidget(evidence)

            layout.addWidget(row)

        layout.addStretch()
        scroll.setWidget(container)
        return scroll
