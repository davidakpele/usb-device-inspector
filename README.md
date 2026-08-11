# USB Device Inspector

A professional Windows desktop application built with Python and PySide6 that detects, identifies, monitors, and provides deep inspection of every USB device connected to your computer — including live real-time controller input monitoring with directional interpretation.

---

## Table of Contents

1. [Overview](#overview)
2. [Features](#features)
3. [Requirements](#requirements)
4. [Installation](#installation)
5. [Running the Application](#running-the-application)
6. [Architecture](#architecture)
7. [Module Reference](#module-reference)
8. [Device Categories](#device-categories)
9. [Device Inspection — Sections Produced](#device-inspection--sections-produced)
10. [Controller Live Monitor](#controller-live-monitor)
11. [Device History](#device-history)
12. [Logging](#logging)
13. [Data Flow](#data-flow)
14. [Security and Safety](#security-and-safety)
15. [Known Limitations](#known-limitations)
16. [Future Extensions](#future-extensions)

---

## Overview

USB Device Inspector is a read-only inspection and monitoring tool. It never modifies connected devices, installs drivers, or executes any content from USB media. It operates entirely through the Windows PnP layer (WMI / hidapi) and presents the information in a clean, dark-themed PySide6 desktop UI.

### Workflow

```
USB Device Connected
        ↓
Detect Device (WMI Win32_PnPEntity)
        ↓
Normalize & Classify (device_detector.py)
        ↓
Display in Device List
        ↓
User selects device → Scan
        ↓
Inspector dispatched by category
        ↓
DeviceDetails displayed (tabbed sections)
        ↓
[Game Controller only] → Test Controller Live
        ↓
Real-time HID input monitoring (120 Hz)
```

---

## Features

### Core Detection
- Enumerates all connected USB devices on startup via `Win32_PnPEntity`
- Real-time connect/disconnect monitoring via WMI event notifications (no polling)
- Handles multiple devices simultaneously
- Gracefully survives device removal during scanning

### Device Information
For every detected device:
- Device name, manufacturer, description
- Vendor ID (VID) and Product ID (PID)
- Serial number (extracted from PNPDeviceID instance segment; not reported if unavailable)
- USB device class, subclass, and protocol (parsed from compatible-ID strings)
- Firmware revision / bcdDevice (parsed from `REV_XXXX` in hardware ID)
- Hardware IDs and compatible IDs
- Device instance ID and parent device
- Driver name, provider, version, and date (via `Win32_PnPSignedDriver`)
- Connection status

### Device Categorization
Automatic classification into 14 categories with honest provenance tracking (`Directly Reported`, `Detected`, `Derived`, `Unknown`).

### Device-Specific Inspection
| Category | Extra information |
|---|---|
| Storage | Drive letter, volume name, file system, capacity, free/used space, removable flag, disk model, disk serial |
| Input Device (HID) | HID usage page/usage, input type (keyboard/mouse/generic), hidapi integration |
| Game Controller | Firmware revision, XInput vs DirectInput, wired vs Bluetooth, interface count, full HID report descriptor parse (button count, axis list with bit widths, hat switch count, force-feedback, rumble) |
| Camera | Camera type (webcam vs still-image scanner), DirectShow limitation note |
| Serial Device | COM port assignment, controller chip identification (CH340/CP210x/FT232/PL2303), supported baud rates |

### Scan Function
- Dedicated scan operation per device; non-destructive, read-only
- Progress dialog with 10 / 50 / 90 / 100 % milestones
- Scan runs on a `QRunnable` / `QThreadPool` worker — UI stays responsive
- Results displayed in a tabbed `DeviceDetails` panel

### Controller Live Monitor
Available for any device classified as `Game Controller`:
- **Real-time HID polling at ~120 Hz** via `hidapi`
- **Directional interpretation**: Forward / Back / Left / Right / Forward-Left / Forward-Right / Back-Left / Back-Right / Center
- **Motion status**: Moving / Stopped (with configurable 15 % dead-zone)
- **Joystick canvas**: animated 2D dot with green trail and dead-zone ring
- **Measurements**: X/Y coordinates (±1.0), angle from forward (0–360°), magnitude (0.0–1.0)
- **Twist / rudder**: Rz axis in degrees and percent
- **Throttle / slider**: slider axis in percent
- **Hat switch compass**: animated 8-point compass rose
- **Button grid**: all buttons shown; lights green on press
- **Motion Log**: timestamped direction-change events
- **Input Event Log**: timestamped button press / release events
- **Statistics**: total event count, events/second rate

### Device History
- Persists to `%LOCALAPPDATA%\USBDeviceInspector\device_history.json`
- Records device name, category, VID, first-seen and last-seen timestamps
- Serial numbers are never persisted
- History viewer dialog; one-click clear

### Real-Time Event Log
- Colour-coded by event type (connected = green, removed = red, error = orange, info = grey)
- Capped at 500 entries with automatic trimming
- Per-session; not persisted to disk

---

## Requirements

| Package | Version | Purpose |
|---|---|---|
| PySide6 | 6.11.1 | Desktop UI framework |
| pywin32 | 311 | Windows COM / WMI bindings |
| WMI | 1.5.1 | WMI Python wrapper |
| hidapi | 0.15.0 | HID device access for controller scan and live input |
| psutil | 7.0.0 | System utilities |

**Platform**: Windows 10 or Windows 11 only. The application refuses to start on other platforms with a clear error message.

**Python**: 3.13+

---

## Installation

```powershell
# Clone or download the project
cd usb-device-inspector

# Create a virtual environment
python -m venv .venv
.venv\Scripts\activate

# Install dependencies
pip install -r requirements.txt
```

---

## Running the Application

```powershell
# From the project root with the venv active:
python -m app.main
```

Or directly:

```powershell
python app/main.py
```

---

## Architecture

The application is layered so that Windows-specific code never reaches the UI:

```
┌─────────────────────────────────────────┐
│          PySide6 UI (app/ui/)           │
├─────────────────────────────────────────┤
│       Application Services (app/services/)│
│  DeviceService · ScanningService        │
│  HistoryService                         │
├─────────────────────────────────────────┤
│       Core Logic (app/core/)            │
│  DeviceManager · DeviceScanner          │
│  DeviceMonitorThread · DeviceDetector   │
│  ControllerMonitorThread                │
├─────────────────────────────────────────┤
│  Device Inspectors (app/inspectors/)    │
│  Storage · HID · Controller · Camera   │
│  Serial                                 │
├─────────────────────────────────────────┤
│       USB Layer (app/usb/)              │
│  usb_enumerator · usb_descriptor        │
│  usb_constants · usb_utils              │
├─────────────────────────────────────────┤
│   Windows OS (WMI / hidapi / Win32 API) │
└─────────────────────────────────────────┘
```

---

## Module Reference

### `app/main.py`
Application entry point. Calls `require_windows()`, configures logging, creates `QApplication` and `MainWindow`, defers initial USB enumeration 100 ms via `QTimer.singleShot` so the window appears before the first WMI call.

---

### `app/models/`

| File | Contents |
|---|---|
| `usb_device.py` | `USBDevice` dataclass — the normalized in-memory representation of one USB device. Fields: `device_id`, `name`, `manufacturer`, `vendor_id`, `product_id`, `serial_number`, `device_class/subclass/protocol`, `usb_version`, `hardware_id/ids`, `compatible_ids`, `driver_*`, `status`, `category`, `connected`, `first_seen`, `last_seen`. Also `FieldSource` enum (`DIRECTLY_REPORTED`, `DETECTED`, `DERIVED`, `UNKNOWN`) and `ConnectionStatus` enum. |
| `device_details.py` | `DeviceDetails` (device + sections + capabilities + warnings), `DetailSection` (title + list of `DetailField`), `DetailField` (label + value + source). |
| `device_capability.py` | `DeviceCapability` (label + source + evidence string). Every capability claim requires a concrete evidence string. |
| `device_event.py` | `DeviceEvent` (event_type, message, timestamp, device_id) and `DeviceEventType` enum. |

---

### `app/usb/`

| File | Contents |
|---|---|
| `usb_descriptor.py` | `RawPnPDescriptor` — verbatim snapshot of `Win32_PnPEntity` fields. Kept separate from `USBDevice` so classification can be unit-tested without WMI. |
| `usb_enumerator.py` | **The single Windows seam.** `enumerate_usb_descriptors()` queries `Win32_PnPEntity` and returns `RawPnPDescriptor` list. `get_driver_info()` queries `Win32_PnPSignedDriver`. `get_storage_volume_info()` chains `Win32_DiskDrive → Win32_DiskPartition → Win32_LogicalDisk`. Nothing outside `app/usb/` imports `wmi` or `win32com`. |
| `usb_constants.py` | `USBClassCode` enum, `USB_CLASS_NAMES` dict, `DeviceCategory` enum, `PNP_CLASS_TO_CATEGORY` mapping, `KNOWN_DEV_BOARD_VIDS`, `KNOWN_CONTROLLER_VIDS`. |
| `usb_utils.py` | Pure utility functions: `extract_vid_pid()`, `extract_serial_from_instance_id()`, `extract_firmware_revision()` (`REV_XXXX` from hardware ID), `extract_usb_class_subclass_protocol()` (`Class_XX&SubClass_XX&Prot_XX` from compatible IDs), `extract_interface_number()` (`MI_XX`), `first_non_empty()`, `format_vid_pid()`, `safe_hex()`. |

---

### `app/core/`

| File | Contents |
|---|---|
| `device_detector.py` | `normalize(descriptor)` — builds a `USBDevice` from a `RawPnPDescriptor`, parsing class/subclass/protocol and firmware revision. `classify(descriptor, vid)` — 4-level classification: PNPClass mapping → hardware-ID heuristics (with controller promotion before generic HID) → known-VID heuristics → UNKNOWN. `_is_game_controller()` detects joysticks/gamepads inside HIDClass via `HID_DEVICE_SYSTEM_GAME_CONTROLLER`, `XUSB`/`XINPUT` markers, and name keywords (wingman, sidewinder, thrustmaster, hotas, wheel, etc.). |
| `device_manager.py` | Thread-safe in-memory device registry. `apply_snapshot()` diffs a new device list against current state and returns (newly_connected, newly_removed). `mark_connected()`, `mark_removed()`, `get()`, `all_devices()`. |
| `device_monitor.py` | `DeviceMonitorThread` (QThread) — WMI creation/deletion event watcher using `watch_for()`. Emits `device_arrived(str)` and `device_departed(str)` signals carrying the PNPDeviceID. Timeout-based polling at 2-second `WITHIN` interval. |
| `device_scanner.py` | `DeviceScanner.scan(device)` — enriches driver info, dispatches to the correct `BaseInspector` via `_INSPECTOR_REGISTRY` (keyed by `DeviceCategory.value`), falls back to `_GenericInspector`. Adding a new inspector only requires importing it here and adding to the registry dict. |
| `controller_monitor.py` | Full real-time HID input pipeline. See [Controller Live Monitor](#controller-live-monitor) below. |

---

### `app/inspectors/`

All inspectors extend `BaseInspector`. The four standard sections (General, USB Information, Hardware Identification, Driver) are built by shared helpers in `BaseInspector`. Each inspector adds a device-specific section.

| Inspector | Category | Extra section |
|---|---|---|
| `StorageInspector` | Storage | Drive letter, volume name, file system, capacity, free/used space, removable, disk model, disk serial |
| `HIDInspector` | Input Device | HID usage page/usage (via `hid.enumerate`), input type |
| `ControllerInspector` | Game Controller | Controller section (firmware rev, XInput/DirectInput, connection type, interface count) + HID Analysis section (report descriptor parse: buttons, axes + bit widths, hat, force-feedback, rumble) |
| `CameraInspector` | Camera | Camera name, type (webcam vs scanner), note about DirectShow |
| `SerialInspector` | Serial Device | COM port (via `Win32_SerialPort` + PnP name scan), chip identification, baud rates |

`add_capability_if(condition, label, evidence)` is the single choke point for capability claims — callers must pass a concrete boolean and a justification string.

---

### `app/services/`

| File | Contents |
|---|---|
| `device_service.py` | `DeviceService` (QObject) — orchestrates all backend objects. Signals: `devices_changed`, `device_connected`, `device_removed`, `event_logged`, `monitor_error`. `initialize()` runs `refresh()` then starts the monitor thread. `refresh()` calls `enumerate_usb_descriptors()`, normalizes, diffs, records history, emits events. |
| `scanning_service.py` | `ScanSignals` (QObject with `scan_progress`, `scan_complete`, `scan_failed`). `ScanWorker` (QRunnable) wraps `DeviceScanner.scan()`. `ScanningService` submits workers to `QThreadPool.globalInstance()` and tracks active scans per `device_id` to prevent double-submission. |
| `history_service.py` | Thread-safe JSON persistence. `record(device)` upserts an entry. `all_entries()` returns newest-first. `clear()` removes the file. Stores: `device_id`, `name`, `category`, `vendor_id`, `product_id`, `first_seen`, `last_seen`. Serial numbers are never written. |

---

### `app/ui/`

| File | Contents |
|---|---|
| `main_window.py` | `MainWindow` (QMainWindow). Dark Catppuccin theme. Three-panel vertical splitter: device list / details / event log. Toolbar buttons: Inspect Device, Scan Device, 🎮 Test Controller Live (visible only for Game Controller category), Refresh, Device History. Owns `DeviceService`. |
| `device_list.py` | `DeviceListWidget` — `QTableWidget` with columns: status dot, device name, category, VID, PID, status. Colour-coded by `ConnectionStatus`. Emits `device_selected(USBDevice)`. |
| `device_details.py` | `DeviceDetailsWidget` — `QTabWidget` with one tab per `DetailSection`. Each field row shows label, value, and a coloured source badge (green=Directly Reported, blue=Detected, orange=Derived, grey=Unknown). Capabilities tab shows ✓ items with evidence strings. Warnings displayed below tabs. |
| `scan_dialog.py` | `ScanDialog` — modal progress dialog. Progress bar updates at milestones. Close button enabled only after scan completes or fails. |
| `event_log.py` | `EventLogWidget` — colour-coded scrolling log, 500-entry cap with automatic trimming. |
| `controller_test_widget.py` | `ControllerTestWidget` — full live monitor window. See below. |

---

### `app/utils/`

| File | Contents |
|---|---|
| `logger.py` | Rotating file handler (`5 MB × 3`) to `%LOCALAPPDATA%\USBDeviceInspector\logs\usb_inspector.log` + console handler. `get_logger(name)` returns a namespaced child of `usb_inspector`. |
| `platform.py` | `is_windows()`, `is_windows_10_or_11()`, `require_windows()` — single platform gate called once at startup. |

---

## Device Categories

| Category | Detection basis |
|---|---|
| Storage | `PNPClass=DiskDrive/CDROM`, `USBSTOR\\` prefix, `CLASS_08` compatible ID |
| Input Device | `PNPClass=HIDClass/Keyboard/Mouse` (after controller check), `HID_DEVICE_SYSTEM_MOUSE/KEYBOARD` |
| Game Controller | `HID_DEVICE_SYSTEM_GAME_CONTROLLER`, `XUSB`/`XINPUT` marker, name keywords (wingman, joystick, gamepad, sidewinder, thrustmaster, wheel, hotas, etc.), known VID list |
| Camera | `PNPClass=Image/Camera`, `CLASS_0E/06` |
| Audio | `PNPClass=MEDIA/AudioEndpoint`, `CLASS_01` |
| Printer | `PNPClass=Printer`, `CLASS_07` |
| Network Adapter | `PNPClass=Net` |
| Serial Device | `PNPClass=Ports/Modem`, `CLASS_02/0A` |
| Development Board | Known VID list (Arduino, Silicon Labs, Espressif, STM32, Adafruit, SparkFun) |
| USB Hub | `PNPClass=USB` (refined), `CLASS_09` |
| Mobile Device | `PNPClass=WPD`, `WPD`/`MTP` in hardware IDs |
| Security Device | `PNPClass=SmartCardReader/Biometric`, `CLASS_0B` |
| Unknown | No reliable signal found |
| Other | Explicit fallback |

---

## Device Inspection — Sections Produced

Every device scan produces these standard sections plus any device-specific section:

### General
Name, Manufacturer, Description, Category (with source badge), Status, Connection State.

### USB Information
Vendor ID, Product ID, USB Version, Device Class (resolved to human name), Subclass, Protocol.
Class/Subclass/Protocol are parsed from `Class_XX&SubClass_XX&Prot_XX` tokens in the compatible-ID strings — directly reported by the Windows PnP layer.

### Hardware Identification
Hardware ID, Device Instance ID, Compatible IDs, Serial Number (with provenance), Parent Device.

### Driver
Driver Name, Driver Provider, Driver Version, Driver Date, Driver Status.
Populated via `Win32_PnPSignedDriver` joined on `DeviceID`.

### Capabilities
One entry per `DeviceCapability` — each shows a ✓ label, source badge, and the concrete evidence string that justified the claim.

---

## Controller Live Monitor

Opened via the **🎮 Test Controller Live** button, which appears only when a Game Controller is selected.

### How it works

1. **Scan first** — `ControllerInspector` parses the HID report descriptor (via `hidapi`) to extract exact axis names, per-axis bit widths, button count, and hat presence.
2. **Monitor opens** — `ControllerMonitorThread` (QThread) opens the HID device using `hidapi` and polls at ~120 Hz.
3. **ReportDecoder** uses the pre-computed bit layout (from scan) to extract axis values, buttons, and hat with bit-exact precision.
4. **MotionInterpreter** converts raw axis values into `MotionState` with dead-zone applied.

### ReportDecoder — bit-exact parsing

Per-axis bit widths are stored in the scan result (`Axis Bit Sizes` field). For the Logitech WingMan Extreme Digital 3D:

```
Bit layout (6-byte report):
  bits  0-9:  X Axis (10-bit, 0-1023)
  bits 10-19: Y Axis (10-bit, 0-1023)
  bits 20-23: Hat Switch (4-bit, 0-8)
  bits 24-31: Rz / Twist (8-bit, 0-255)
  bits 32-38: 7 Buttons (1-bit each)
  bit  39:    padding
  bits 40-47: Slider / Throttle (8-bit, 0-255)
```

### MotionInterpreter — dead-zone and direction

Dead-zone: 15 % from centre (configurable). Axes within the dead-zone collapse to 0.0.

Axis role detection by name fragments:
- X axis: `"x axis"`, `"x-axis"`, `"lx"`, `"left x"`
- Y axis: `"y axis"`, `"y-axis"`, `"ly"`, `"left y"`
- Twist: `"rz"`, `"z rotation"`, `"rudder"`, `"twist"`, `"yaw"`
- Throttle: `"slider"`, `"throttle"`, `"z axis"`, `"wheel"`, `"dial"`

Direction mapping (8-way + center):

| Y position | X position | Direction | Angle |
|---|---|---|---|
| Forward (Y < dead) | Center | Forward | 0° |
| Back (Y > dead) | Center | Back | 180° |
| Center | Right (X > dead) | Right | 90° |
| Center | Left (X < dead) | Left | 270° |
| Forward | Right | Forward-Right | 45° |
| Forward | Left | Forward-Left | 315° |
| Back | Right | Back-Right | 135° |
| Back | Left | Back-Left | 225° |
| Center | Center | Center | 0° |

### Live monitor UI panels

**Left — Axes**
One row per axis: progress bar (0–100 %), percentage, angle in degrees (0–359.9°), raw sensor value.

**Left — Hat Switch**
Animated 8-point compass rose. Active direction lights green. Angle label below.

**Middle — Direction Command**
- Large directional arrow glyph (↑ ↓ ← → ↖ ↗ ↙ ↘ ⊙)
- Direction text label
- `▶ Moving` (green) / `■ Stopped` (grey)

**Middle — Stick Position**
200×200 painted canvas:
- Blue dot = current stick position
- Green trail (40 points) = recent movement history
- Dashed ring = dead-zone boundary
- Labels: FWD / BCK / L / R

**Middle — Measurements**

| Field | Description |
|---|---|
| X Coordinate | `+0.742  (87.1%)` — right is positive, range −1.0 … +1.0 |
| Y Coordinate | `+1.000  (100.0%)` — forward is positive |
| Angle (from fwd) | 0° = forward, 90° = right, 180° = back, 270° = left |
| Magnitude | 0.000 (centre) → 1.000 (full throw) |
| Twist / Rudder | Rz axis in degrees and percent |
| Throttle / Slider | Slider axis in percent |

**Middle — Motion Log**
Timestamped direction-change events:
```
17:42:11.304  →  Direction: Center ▶ Forward  |  angle 2.3°  mag 0.98
17:42:11.912  ▶ Moving  (X=+0.00, Y=+0.98)
17:42:12.580  →  Direction: Forward ▶ Forward-Right  |  angle 44.1°  mag 0.95
```

**Right — Buttons**
Grid of numbered squares. Turns green on press, grey on release.

**Right — Statistics**
Total events, events/second rate, last direction.

**Right — Input Event Log**
```
17:42:08.112  ▼  Button 1 pressed
17:42:08.344  ▲  Button 1 released
17:42:09.001  ▼  Button 3 pressed
```

---

## Device History

Stored at: `%LOCALAPPDATA%\USBDeviceInspector\device_history.json`

Each entry:
```json
{
  "device_id": "USB\\VID_046D&PID_C207\\7&2350D3DC&0&1",
  "name": "WingMan Extreme Digital 3D",
  "category": "Game Controller",
  "vendor_id": "046D",
  "product_id": "C207",
  "first_seen": "2026-08-11T17:11:45",
  "last_seen": "2026-08-11T17:39:22"
}
```

Serial numbers are never written to disk.

Open via **Device History** button in the main toolbar. Click **Clear History** to delete all records.

---

## Logging

Log file: `%LOCALAPPDATA%\USBDeviceInspector\logs\usb_inspector.log`
Rotation: 5 MB per file, 3 backups kept.

Logged events:
- Application startup and shutdown
- Initial USB enumeration (device count)
- Every device connection and removal
- Scan start and completion (with section count)
- Scan failures and inspector exceptions
- Controller monitor start and stop
- HID read errors
- WMI query failures
- Driver lookup failures

Serial numbers and full hardware IDs are logged only at DEBUG level.

---

## Data Flow

```
Win32_PnPEntity (WMI)
        │
        ▼
RawPnPDescriptor          ← verbatim OS data, no interpretation
        │
        ▼
normalize() + classify()  ← USBDevice + DeviceCategory
        │
        ▼
DeviceManager             ← thread-safe in-memory registry
        │
        ├── DeviceListWidget (UI)
        │
        └── DeviceScanner.scan()
                │
                ▼
         Inspector (by category)
                │
                ▼
          DeviceDetails     ← sections + capabilities + warnings
                │
                ▼
        DeviceDetailsWidget (UI)
                │
                └── [Game Controller] ControllerMonitorThread
                            │
                            ▼
                     ReportDecoder → MotionInterpreter
                            │
                            ▼
                    ControllerTestWidget (UI)
```

---

## Security and Safety

- **Read-only by design.** The application never writes to a connected USB device, never modifies drivers, and never executes files from USB media.
- **No serial numbers on disk.** Device history stores only non-identifying metadata.
- **No internet connection required.** All data comes from the local Windows PnP layer.
- **Graceful failure.** A single bad device never crashes enumeration. All inspection errors are caught, logged, and surfaced as user-friendly warnings.
- **Provenance on every fact.** Every displayed field carries a `FieldSource` badge: `Directly Reported`, `Detected`, `Derived`, or `Unknown`. No data is fabricated.
- **Capabilities require evidence.** The `add_capability_if()` method requires a concrete boolean and a justification string — capability claims cannot be made by category assumption alone.

---

## Known Limitations

- **Windows only.** The USB enumeration layer uses WMI/Win32 APIs. The architecture is designed so a future Linux/macOS backend could be added by replacing `app/usb/usb_enumerator.py` without touching the UI or inspection logic.
- **XInput controllers (Xbox)** — Windows intercepts HID reports before they reach user-space for XInput devices. The live monitor cannot read input from Xbox controllers via hidapi. The scan still produces all static information; only the live monitor is unavailable.
- **Exclusive-access HID devices** — Some keyboards and mice hold the HID interface exclusively. The HID Analysis section will show "Not accessible" for the report descriptor; all other scan information is still displayed.
- **USB Version** — `Win32_PnPEntity` does not expose the USB version (1.1 / 2.0 / 3.x) for most devices. This field shows "Not Available" unless the device descriptor exposes it through another mechanism.
- **Parent device** — `Win32_PnPEntity` does not directly expose the parent instance ID. This field shows "Not Available".
- **Physical characteristics** — Country of manufacture, exact production date, and "original vs counterfeit" status are not exposed by USB or Windows. These are never inferred.

---

## Future Extensions

The architecture supports adding these without restructuring:

- Product database lookup (VID/PID → brand/model)
- Export device information to JSON / CSV / PDF
- Device inventory management
- Device trust / allowlist / blocklist
- System tray monitoring with notifications
- Dark / light theme toggle
- Linux and macOS support (replace `app/usb/usb_enumerator.py`)
- Hardware diagnostics
- Network-based device inventory
- USB device fingerprinting and comparison
