# USB Device Inspector

A professional Windows desktop application that detects, inspects, and monitors
every USB device connected to your computer — including a full real-time controller
input monitor and a military-grade 3D fixed-wing UAV flight simulator controlled by
your joystick.

---

## Table of Contents

1. [Overview](#overview)
2. [Features](#features)
3. [Requirements](#requirements)
4. [Installation](#installation)
5. [Running](#running)
6. [Architecture](#architecture)
7. [Module Reference](#module-reference)
8. [Device Categories](#device-categories)
9. [Device Inspection](#device-inspection)
10. [Controller Live Monitor](#controller-live-monitor)
11. [Drone Simulator](#drone-simulator)
    - [How to fly](#how-to-fly)
    - [Controller mapping](#controller-mapping)
    - [Throttle / altitude-hold model](#throttle--altitude-hold-model)
    - [Physics model](#physics-model)
    - [Flight modes and speeds](#flight-modes-and-speeds)
    - [3D View](#3d-view--fixed-wing-uav-renderer-pure-qpainter-no-opengl)
    - [Architecture — data flow](#architecture--data-flow-through-simulator)
12. [Device History](#device-history)
13. [Flight Logging](#flight-logging)
14. [Application Logging](#application-logging)
15. [Data Flow](#data-flow)
16. [Security and Safety](#security-and-safety)
17. [Known Limitations](#known-limitations)

---

## Overview

USB Device Inspector is a read-only Windows desktop application built with
Python 3.13 and PySide6. It never modifies connected devices, installs drivers,
or executes any content from USB media. All data comes from the Windows PnP
layer (WMI / hidapi).

### Application workflow

```
USB Device Connected
        │
        ▼
WMI Win32_PnPEntity enumeration
        │
        ▼
Normalize + Classify  →  USBDevice + DeviceCategory
        │
        ▼
Device List (main window)
        │
        ├── Scan Device → Inspector → DeviceDetails tabs
        │
        ├── [Game Controller] → 🎮 Test Controller Live
        │       │
        │       └── ControllerMonitorThread (120 Hz HID)
        │               │
        │               └── ControllerTestWidget (axes/buttons/direction)
        │
        └── [Game Controller] → ✈ Drone Simulator
                │
                └── ControllerMonitorThread (120 Hz HID)
                        │
                        ▼
                DronePhysics (60 Hz, altitude-hold)
                        │
                        ▼
                Drone3DWidget (QPainter, fixed-wing UAV)
```

---

## Features

### USB Detection and Inspection
- Enumerates all USB devices on startup via `Win32_PnPEntity`
- Real-time plug/unplug monitoring via WMI event notifications
- Graceful handling of device removal during scanning
- Per-device scan with progress dialog (runs on background thread)
- Tabbed detail panel with source-provenance badge on every field

### Device Information Extracted
- Name, manufacturer, description
- VID / PID (parsed from hardware ID)
- Serial number (from PNPDeviceID instance segment — never fabricated)
- USB device class, subclass, protocol (from `Class_XX&SubClass_XX&Prot_XX`)
- Firmware revision / bcdDevice (from `REV_XXXX` in hardware ID)
- Hardware IDs, compatible IDs, device instance ID
- Driver name, provider, version, date (via `Win32_PnPSignedDriver`)
- Connection status with colour-coded indicator

### Game Controller — Deep HID Analysis
- Full HID report descriptor parse (button count, axis names + bit widths,
  hat switch count, force-feedback, rumble)
- XInput vs DirectInput detection
- Wired vs Bluetooth detection
- Per-interface enumeration
- Stored as `Report Field Map` for bit-exact live decoding

### Controller Live Monitor (🎮 Test Controller Live)
- 120 Hz HID polling via `hidapi`
- Bit-exact report decoding using parsed field map
- **Auto-calibration**: first 60 frames measure resting axis centres and
  subtract hardware offset (fixes WingMan X-axis drift at 58.4% rest)
- **Circular dead-zone**: applied to vector magnitude, not per-axis —
  preserves exact direction even near the dead-zone edge
- **Continuous 360° movement**: 16-sector `atan2`-based direction label,
  no snap to 8 compass points
- Joystick canvas with trail, direction arrow, velocity vector
- Hat compass rose, button grid, motion log, event log

### Drone Simulator (✈ Drone Simulator)
Full 3D fixed-wing UAV flight simulator controlled by the physical joystick.
See [Drone Simulator](#drone-simulator) section below.

### Device History
- Persists to `%LOCALAPPDATA%\USBDeviceInspector\device_history.json`
- Records VID, name, category, first-seen, last-seen
- Serial numbers never persisted

### Flight Log (per drone session)
- Markdown file per session: `%LOCALAPPDATA%\USBDeviceInspector\logs\flight_YYYYMMDD_HHMMSS.md`
- Records axis calibration, every mode change, every button press,
  combined telemetry + input table sampled at 1 Hz

---

## Requirements

| Package | Version | Purpose |
|---|---|---|
| PySide6 | 6.11.1 | Desktop UI (Qt6 bindings) |
| pywin32 | 311 | WMI / Win32 COM bindings |
| WMI | 1.5.1 | WMI Python wrapper |
| hidapi | 0.15.0 | HID device access (`hid.device()` API) |
| psutil | 7.0.0 | System utilities |

**Platform**: Windows 10 / 11 only.  
**Python**: 3.13+

---

## Installation

```powershell
cd usb-device-inspector
python -m venv .venv
.venv\Scripts\activate
pip install -r requirements.txt
```

---

## Running

```powershell
python -m app.main
```

---

## Architecture

```
┌──────────────────────────────────────────────────────────┐
│                   PySide6 UI  (app/ui/)                  │
│  MainWindow · DeviceList · DeviceDetails · EventLog      │
│  ControllerTestWidget · DroneSimulator · Drone3DView     │
├──────────────────────────────────────────────────────────┤
│               Application Services  (app/services/)      │
│  DeviceService · ScanningService · HistoryService        │
├──────────────────────────────────────────────────────────┤
│                Core Logic  (app/core/)                   │
│  DeviceManager · DeviceScanner · DeviceMonitorThread     │
│  ControllerMonitorThread · DronePhysics                  │
├──────────────────────────────────────────────────────────┤
│              Device Inspectors  (app/inspectors/)        │
│  Storage · HID · Controller · Camera · Serial            │
├──────────────────────────────────────────────────────────┤
│                 USB Layer  (app/usb/)                    │
│  usb_enumerator · usb_descriptor · usb_constants        │
│  usb_utils                                               │
├──────────────────────────────────────────────────────────┤
│          Windows OS  (WMI / hidapi / Win32 API)          │
└──────────────────────────────────────────────────────────┘
```

The Windows seam is confined entirely to `app/usb/usb_enumerator.py`.
Nothing outside that file imports `wmi` or `win32com`.

---

## Module Reference

### `app/main.py`
Entry point. Calls `require_windows()`, configures rotating log, creates
`QApplication` + `MainWindow`, defers initial WMI enumeration 100 ms so the
window appears before the first WMI query.

---

### `app/models/`

| File | Key contents |
|---|---|
| `usb_device.py` | `USBDevice` dataclass, `FieldSource` enum, `ConnectionStatus` enum |
| `device_details.py` | `DeviceDetails`, `DetailSection`, `DetailField` |
| `device_capability.py` | `DeviceCapability` — every claim needs an evidence string |
| `device_event.py` | `DeviceEvent`, `DeviceEventType` enum |

---

### `app/usb/`

| File | Key contents |
|---|---|
| `usb_descriptor.py` | `RawPnPDescriptor` — verbatim WMI snapshot |
| `usb_enumerator.py` | `enumerate_usb_descriptors()`, `get_driver_info()`, `get_storage_volume_info()` |
| `usb_constants.py` | `USBClassCode`, `USB_CLASS_NAMES`, `DeviceCategory`, `PNP_CLASS_TO_CATEGORY`, `KNOWN_CONTROLLER_VIDS` |
| `usb_utils.py` | `extract_vid_pid()`, `extract_serial_from_instance_id()`, `extract_firmware_revision()`, `extract_usb_class_subclass_protocol()`, `extract_interface_number()` |

---

### `app/core/`

| File | Key contents |
|---|---|
| `device_detector.py` | `normalize()`, `classify()`, `_is_game_controller()` — detects HIDClass joysticks by `HID_DEVICE_SYSTEM_GAME_CONTROLLER`, XUSB/XINPUT markers, name keywords |
| `device_manager.py` | Thread-safe registry; `apply_snapshot()` diffs new/removed devices |
| `device_monitor.py` | `DeviceMonitorThread` — WMI creation/deletion watcher |
| `device_scanner.py` | `DeviceScanner.scan()` — dispatcher to inspector registry |
| `controller_monitor.py` | Full HID pipeline: `ReportDecoder`, `MotionInterpreter`, `ControllerMonitorThread` |
| `drone_physics.py` | `DronePhysics`, `DroneState`, `DroneInput`, `FlightMode` |

#### `controller_monitor.py` — detailed

**`ReportDecoder`** — three modes (priority order):
1. Field-map-driven: uses `Report Field Map` from scan for bit-exact decode
2. Bit-size-aware: uses per-axis bit widths, axes-first assumption
3. Auto-detect: heuristic uniform-width fallback

**`MotionInterpreter`** — continuous 360° polar system:
- Auto-calibrates axis centres over first 60 frames (fixes hardware drift)
- Circular dead-zone on vector magnitude — direction never distorted
- Radial rescale: `(mag − dz) / (1 − dz)` → smooth [0, 1] response
- `atan2(x, y)` angle CW from forward (0°)
- 16-sector direction labels × 22.5° each — covers full circle, no gaps
- Throttle NOT calibrated (absolute axis, needs full range)

#### `drone_physics.py` — flight model

**Flight modes**: DISARMED → ARMED → HOVER / SPORT / PRECISION → TAKEOFF → LANDING

**Altitude-hold**: slider 0.5 = hold, >0.5 = climb, <0.5 = descend.
Dead-zone ±0.08 around 0.5. Max descent capped at 30% so 0% throttle
gives 1.1 m/s descent not instant crash.

**Continuous 360° movement**:
- `inp.roll` + `inp.pitch` form a 2D polar input vector
- Magnitude extracted: `sqrt(roll² + pitch²)`
- Unit direction vector rotated by drone yaw into world frame
- World velocity = `unit_dir × magnitude × max_speed`
- No quadrant logic, no snap, smooth transition through every angle

**Control mapping**:

| Input | Action |
|---|---|
| X axis (roll) | Strafe left / right |
| Y axis (pitch) | Forward / backward |
| Rz twist | Yaw (rotate in place) |
| Slider | Altitude (50% = hold) |
| Button 1 | ARM / DISARM |
| Button 2 | Emergency LAND |
| Button 3 | HOVER / STABLE toggle |
| Button 4 | Reset drone |
| Button 5 | SPORT mode (18 m/s) |
| Button 6 | PRECISION mode (3 m/s) |
| Button 7 | Auto TAKE-OFF to 3 m |
| Hat N/S | Altitude trim ± |

---

### `app/inspectors/`

All extend `BaseInspector`. Shared sections: General, USB Information,
Hardware Identification, Driver.

| Inspector | Category | Extra section |
|---|---|---|
| `StorageInspector` | Storage | Drive letter, FS, capacity, removable, disk model |
| `HIDInspector` | Input Device | HID usage page/usage |
| `ControllerInspector` | Game Controller | Firmware rev, XInput/DirectInput, Report Field Map, button/axis/hat/FF |
| `CameraInspector` | Camera | Camera type, DirectShow note |
| `SerialInspector` | Serial Device | COM port, chip ID (CH340/CP210x/FT232/PL2303) |

---

### `app/services/`

| File | Key contents |
|---|---|
| `device_service.py` | `DeviceService` (QObject) — master orchestrator; signals: `devices_changed`, `device_connected`, `device_removed`, `event_logged`, `monitor_error` |
| `scanning_service.py` | `ScanWorker` (QRunnable), `ScanSignals`, `ScanningService` |
| `history_service.py` | Thread-safe JSON persistence; serial numbers never written |

---

### `app/ui/`

| File | Key contents |
|---|---|
| `main_window.py` | `MainWindow` — dark Catppuccin theme; buttons: Inspect, Scan, 🎮 Test Controller Live, ✈ Drone Simulator (controller only), Refresh, Device History |
| `device_list.py` | `DeviceListWidget` — connection-state dot, category, VID/PID |
| `device_details.py` | `DeviceDetailsWidget` — tabbed sections, source badges, capabilities |
| `scan_dialog.py` | `ScanDialog` — modal progress at 10/50/90/100% |
| `event_log.py` | `EventLogWidget` — colour-coded, 500-entry cap |
| `controller_test_widget.py` | `ControllerTestWidget` — 60 Hz render timer, 3-column layout |
| `drone_3d_view.py` | `Drone3DWidget` — pure QPainter oblique-projection renderer; `_FixedWingUAV` geometry class |
| `drone_simulator.py` | `DroneSimulatorWindow` — full GCS-style simulator window |

---

### `app/utils/`

| File | Key contents |
|---|---|
| `logger.py` | Rotating handler 5 MB×3 to `logs/usb_inspector.log` |
| `platform.py` | `require_windows()` — platform gate at startup |
| `flight_logger.py` | `FlightLogger` — Markdown session log, thread-safe, 1 Hz sampling |

---

## Device Categories

| Category | Detection signals |
|---|---|
| Storage | `PNPClass=DiskDrive/CDROM`, `USBSTOR`, `CLASS_08` |
| Input Device | `PNPClass=HIDClass/Keyboard/Mouse` (after controller check) |
| **Game Controller** | `HID_DEVICE_SYSTEM_GAME_CONTROLLER`, `XUSB/XINPUT`, name keywords (wingman, joystick, gamepad, sidewinder, thrustmaster, hotas, wheel, flightstick) |
| Camera | `PNPClass=Image/Camera`, `CLASS_0E/06` |
| Audio | `PNPClass=MEDIA/AudioEndpoint`, `CLASS_01` |
| Serial Device | `PNPClass=Ports/Modem`, `CLASS_02/0A` |
| Printer | `PNPClass=Printer`, `CLASS_07` |
| Network Adapter | `PNPClass=Net` |
| Development Board | VID list: Arduino, Silicon Labs, Espressif, STM32, Adafruit, SparkFun |
| USB Hub | `PNPClass=USB` + hub keyword, `CLASS_09` |
| Mobile Device | `PNPClass=WPD`, `WPD/MTP` in hardware IDs |
| Security Device | `PNPClass=SmartCardReader/Biometric`, `CLASS_0B` |
| Unknown | No reliable signal |

Classification uses honest provenance: `DETECTED` for PNPClass match,
`DERIVED` for VID heuristic, `UNKNOWN` when no signal found.

---

## Device Inspection

Every scan produces four standard sections plus a device-specific section:

- **General** — name, manufacturer, description, category (with source badge), status
- **USB Information** — VID, PID, device class/subclass/protocol (from compatible IDs)
- **Hardware Identification** — hardware ID, instance ID, compatible IDs, serial number
- **Driver** — name, provider, version, date (via `Win32_PnPSignedDriver`)
- **Capabilities** — ✓ items with evidence strings; no claim without proof

---

## Controller Live Monitor

Opened via **🎮 Test Controller Live** (Game Controller devices only).

### Signal flow

```
ControllerMonitorThread (120 Hz HID)
        │
        ▼  _on_state() — stores InputState only
        │
        ▼  QTimer 60 Hz — _render_frame()
        │
        ├── AxisRowWidget.update_axis()
        ├── HatCompassWidget.set_direction()
        ├── JoystickCanvas.update_position()
        └── button grid, motion log, event log
```

The 120 Hz signal handler stores only the latest state snapshot.
A separate 60 Hz timer renders from that snapshot — eliminates Qt
stylesheet reparse glitch from direct 120 Hz UI updates.

### Auto-calibration

The first 60 frames measure the resting centre of each axis and subtract
the offset. Example for Logitech WingMan Extreme Digital 3D:

| Axis | Raw rest | Offset | After calibration |
|---|---|---|---|
| X Axis | 58.4% | +8.4% | 50.0% → 0 drift |
| Y Axis | 47.2% | −2.8% | 50.0% → 0 drift |
| Rz | 46.7% | −3.3% | 50.0% → 0 drift |

Throttle/Slider is **not** calibrated — it is an absolute control.

### Continuous 360° movement

The old system had 8 discrete direction states with gaps between them.
The new system:

1. Subtract calibrated centre → normalised raw `(x, y)` in [−1, +1]
2. Compute polar magnitude: `sqrt(x² + y²)`
3. Apply **circular** dead-zone on magnitude (not per-axis)
4. Radial rescale: `(mag − dz) / (1 − dz)` → smooth response
5. Reconstruct `x_coord / y_coord = unit_vector × rescaled_magnitude`
6. Angle: `atan2(x, y)` clockwise from forward (0°)
7. Direction label from 16 sectors × 22.5° each — every degree covered

### WingMan bit layout (6-byte report)

```
bits  0–9:  X Axis      (10-bit, 0–1023)
bits 10–19: Y Axis      (10-bit, 0–1023)
bits 20–23: Hat Switch  (4-bit,  0–8)
bits 24–31: Rz/Twist    (8-bit,  0–255)
bits 32–38: 7 Buttons   (1-bit each)
bit  39:    Padding
bits 40–47: Slider      (8-bit,  0–255)
```

This layout is stored in the scan result as `Report Field Map` and used
for bit-exact decoding during live monitoring.

### UI panels

| Panel | Contents |
|---|---|
| Left — Axes | Bar + % + degrees + raw value per axis |
| Left — Hat | 8-point compass rose, lights on press |
| Middle — Direction | Arrow glyph, direction text, Moving/Stopped |
| Middle — Canvas | 200×200 stick position dot + trail + dead-zone ring |
| Middle — Measurements | X/Y coord ±1, angle 0–360°, magnitude, twist, throttle |
| Middle — Motion Log | Timestamped direction-change events |
| Right — Buttons | Grid lights green on press |
| Right — Statistics | Total events, events/second rate, last direction |
| Right — Event Log | Timestamped button press/release |

---

## Drone Simulator

Opened via **✈ Drone Simulator** (Game Controller devices only).
Requires scanning the device first to get the precise HID field map.

### How to fly

1. Click **ARM / DISARM** or press Button 1
2. Click **🚀 TAKE-OFF** or press Button 7 → auto-climbs to 3 m, enters HOVER
3. Move the **stick** in any direction to fly
4. Move the **slider** above 50% to climb, below 50% to descend
5. **Twist** the handle (Rz) to rotate in place (yaw)
6. Click **⬇ LAND** or press Button 2 → auto-descends and disarms

### Controller mapping

| Input | Effect |
|---|---|
| X axis | Roll — strafe left / right |
| Y axis | Pitch — forward / backward |
| Rz (twist) | Yaw — rotate clockwise / counter-clockwise |
| Slider (50% = hold) | Altitude control |
| Hat N | Altitude trim up |
| Hat S | Altitude trim down |
| Button 1 | ARM / DISARM |
| Button 2 | Emergency auto-land |
| Button 3 | HOVER / STABLE mode |
| Button 4 | Reset drone to origin |
| Button 5 | SPORT mode (18 m/s) |
| Button 6 | PRECISION mode (3 m/s) |
| Button 7 | Auto take-off to 3 m |

Keyboard: `Space`=ARM, `T`=TAKEOFF, `L`=LAND, `H`=HOVER, `←/→`=orbit camera,
`+`=zoom in, `−`=zoom out, `0`=reset zoom.

### Throttle / altitude-hold model

The slider is treated as a climb-rate command, not raw thrust:

| Slider | Effect |
|---|---|
| 50% (centred) | Hold current altitude (dead-zone ±8%) |
| >50% | Climb up to 5 m/s |
| <50% | Descend — gentle 1.1 m/s at 0% (30% of max) |

This is the same model used in DJI Phantom / Mavic drones.
The drone will not crash from a momentarily-released slider.

### Physics model

- **60 Hz** physics tick via `QTimer.PreciseTimer`
- Continuous 360° horizontal movement via polar vector decomposition
- `inp.pitch > 0` = forward, no sign inversion needed
- World velocity = `rotate(unit_dir, yaw) × magnitude × max_speed`
- Drag: 3× stronger when stick released (brakes quickly)
- Visual tilt derived from actual world velocity (not stick input)
- Engine/rotor speed proportional to throttle effort
- Button edge detection: fires once per press regardless of hold duration

### Flight modes and speeds

| Mode | H speed | Yaw rate |
|---|---|---|
| ARMED | 8 m/s | 100°/s |
| HOVER | 6 m/s | 100°/s |
| SPORT | 18 m/s | 200°/s |
| PRECISION | 3 m/s | 50°/s |

### 3D View — fixed-wing UAV renderer (pure QPainter, no OpenGL)

**Camera**: Satellite-station-above perspective (70° top-down tilt).
Drone is always locked at screen centre regardless of position or zoom.
Camera follow is mathematically exact — zero pixel error at all zoom levels.

**Zoom**:
- Mouse scroll wheel (×1.25 per notch, ×1.12 with Ctrl)
- `🔍+` / `🔍−` / `1:1` buttons in header
- Keyboard `+` / `−` / `0`
- Range: 20 px/m (100 m+ view) → 220 px/m (close-up ~4 m view)
- Default: 85 px/m (~10×15 m visible area)

**Projection constants**:

| Setting | Value | Effect |
|---|---|---|
| `DEPTH_X` | 0.20 | Mild horizontal oblique — clean top-down |
| `DEPTH_Y` | 0.58 | Steep 70° downward tilt — satellite feel |
| `cam_yaw` default | 25° | Three-quarter front-side view for best UAV perspective |

**Environment layers** (back to front):
1. Night sky — gradient + 120 pre-cached stars + NVG horizon glow
2. Dark olive terrain with diagonal texture lines
3. Range rings at 25 m / 50 m / 100 m (dotted, fades with altitude)
4. Tactical 5 m grid — major lines every 25 m, sector labels
5. Origin cross-hair (amber runway marker)
6. Ground compass — N/S/E/W badges always centred on drone
7. Flight trail — amber→green colour fade over 300 points
8. Altitude shadow — radial gradient, fades with height
9. Fixed-wing UAV body (see below)
10. Velocity vector — green arrow, glow, speed label

**Fixed-wing UAV body — Reconnaissance UAV**:

The aircraft is built entirely in body frame by the `_FixedWingUAV` class and
projected to screen via the same oblique `_proj()` formula used by the rest of
the scene. No external model file is required.

Body-frame coordinate convention:
- `+X` = right wing tip direction
- `+Y` = up (dorsal)
- `−Z` = forward (nose) — standard aerospace convention

| Part | Detail |
|---|---|
| Fuselage | Tapered hexagonal cross-section, 17 longitudinal stations, ogive nose, tail boom; longitudinal panel lines |
| Main wings | Swept trapezoidal panels, left and right; leading-edge dark strip; slight dihedral at tip |
| Horizontal stabiliser | Smaller tail planes, port and starboard |
| Vertical stabiliser | Dorsal fin with dark leading-edge strip |
| Ailerons | Trailing-edge panels on each wing, deflect ±22° from `state.roll` (differential — left up when right down) |
| Elevator | Trailing edge of horizontal stab, deflects ±20° from `state.pitch` |
| Rudder | Trailing edge of vertical fin, deflects ±18° from roll/yaw |
| Sensor pod | Streamlined EO/IR ball under nose — dark body, blue glass aperture, lens reflection |
| Propeller | 2-blade nose tractor: disc blur fades in with `rotor_speed`; blade lines visible when slow; spinner hub |
| Landing gear | Tricycle (nose + two mains), retracts above 1.5 m AGL; wheels drawn as ovals |
| Nav lights | Green port wingtip, red starboard wingtip, white tail strobe (blinks every 30 frames) |
| Engine exhaust | Twin exhaust rings at rear belly; orange heat-glow shimmer at speed |
| Engine wash | Radial turbulence glow around drone when flying |
| Altitude stem | Dashed vertical line + 5 m tick marks |
| FWD marker | Amber dot + "FWD" label ahead of nose |

**Control surface animation**:

Control surfaces deflect in real time from the physics state — purely visual,
no secondary physics effect:

| DroneState field | Surface | Max deflection |
|---|---|---|
| `roll` | Left / right ailerons (differential) | ±22° |
| `pitch` | Elevator (both sides) | ±20° |
| `roll` (yaw proxy) | Rudder | ±18° |
| `rotor_speed` | Propeller disc opacity + blade fade | 0→1 |

Active surfaces switch to a lighter composite colour so deflection is visible
against the fixed structure.

**HUD panels** (tactical GCS style):

| Panel | Location | Contents |
|---|---|---|
| Mode badge | Top centre | `[ HOVER ]` with corner bracket styling |
| Flight command | Below badge | Large coloured banner — colour varies by direction |
| Telemetry | Top left | ALT, H-SPD, V-SPD, HDG, PITCH, ROLL, X POS, Z POS, DIST, FLT TIME |
| Compass rose | Top right | 36 bearing ticks, needle, digital heading readout |
| ADI | Bottom left | Artificial horizon with pitch ladder, roll, sky/earth split |
| Altitude tape | Bottom right | Blue ladder with pointer arrow + value box |
| Throttle tape | Bottom right | Green, centred on HOLD, pointer arrow |
| Status strip | Right edge | MODE / SPD / ALT / HDG / THR / AIR-BORN colour-coded |
| Radar mini-map | Bottom corner | Range rings, trail, heading arrow, RADAR label |
| Crosshair | Centre | FPV reticle with corner ticks |
| Scanlines | Full screen | Subtle CRT/NVG effect |

### Architecture — data flow through simulator

```
Physical Controller (USB HID)
        │  ~120 Hz raw bytes
        ▼
ControllerMonitorThread
  ReportDecoder  →  MotionInterpreter
        │  Signal: state_updated(InputState)
        ▼
DroneSimulatorWindow._on_controller_state()
  [stores snapshot only]
        │
        ▼  QTimer 60 Hz
DroneSimulatorWindow._physics_tick()
  _build_drone_input()
    inp.roll     = motion.x_coord
    inp.pitch    = motion.y_coord
    inp.yaw      = twist_raw
    inp.throttle = throttle_percent / 100
    inp.btn_*    = rising-edge detection
        │
        ▼
DronePhysics.step(inp, dt)  →  DroneState
        │
        ├── DroneSimulatorWindow._update_telemetry()
        ├── DroneSimulatorWindow._update_mode_label()
        │
        ▼
Drone3DWidget.update_state(DroneState)
        │  _prop_angle += rotor_speed × 18°/frame
        │  smooth camera follow (α=0.18 / 0.08)
        ▼
paintEvent() → _draw_drone()
  b2w(bx,by,bz)  = roll → pitch → yaw → translate
  _proj(wx,wy,wz) = oblique projection → screen pixels
  _FixedWingUAV.draw_fuselage()
  _FixedWingUAV.draw_wings(aileron_defl)
  _FixedWingUAV.draw_hstab(elevator_defl)
  _FixedWingUAV.draw_vstab(rudder_defl)
  _FixedWingUAV.draw_sensor_pod()
  _FixedWingUAV.draw_propeller(prop_angle, rotor_speed)
  _FixedWingUAV.draw_landing_gear(altitude)
  _FixedWingUAV.draw_nav_lights(armed, rotor_speed, frame)
  _FixedWingUAV.draw_engine_detail(rotor_speed)
        │
        ▼
Rendered Fixed-Wing UAV on tactical display
```

---

## Device History

Stored at: `%LOCALAPPDATA%\USBDeviceInspector\device_history.json`

```json
{
  "device_id": "USB\\VID_046D&PID_C207\\7&2350D3DC&0&1",
  "name": "WingMan Extreme Digital 3D",
  "category": "Game Controller",
  "vendor_id": "046D",
  "product_id": "C207",
  "first_seen": "2026-08-11T17:11:45",
  "last_seen":  "2026-08-11T17:39:22"
}
```

Serial numbers are never written to disk. Open via **Device History**
button; click **Clear History** to delete all records.

---

## Flight Logging

Each drone simulator session creates:
`%LOCALAPPDATA%\USBDeviceInspector\logs\flight_YYYYMMDD_HHMMSS.md`

### Log structure

```markdown
# USB Device Inspector — Flight Log
| Session start | 2026-08-11 22:34:23 |
| Controller    | WingMan Extreme Digital 3D |

## Axis Calibration
| Axis   | Raw Rest % | Offset  | Status            |
| x axis | 58.4       | +8.40%  | ⚠ corrected       |
| y axis | 47.2       | -2.80%  | ⚠ corrected       |

## Axis Diagnostic Snapshot
(raw vs calibrated per axis)

🔧 `ARM`     — Armed
🚀 `TAKEOFF` — Auto takeoff to 3 m
🔄 `MODE`    — Mode changed: ARMED → HOVER

## Flight Data (1 Hz sample)
| Time     | Mode  | Command | Alt  | H-Spd | V-Spd | Hdg° | Pitch° | Roll° | X    | Z     | FT  | inp.Roll | inp.Pitch | inp.Yaw | Thr   | Btns |
| 22:34:25 | HOVER | Forward | 3.00 | 3.62  | +0.00 | 0.0  | -16.9  | +0.0  | 0.00 | -3.26 | 3.0 | +0.000   | +0.800    | +0.000  | 0.500 | —    |

## Session Summary
| Duration | 47.3 s |
| Rows     | 47     |
```

All event types (MODE, COMMAND, BUTTON, ARM, LAND, TAKEOFF, ERROR, CALIB)
are logged immediately when they occur. The telemetry table is a single
continuous block — it never reopens mid-session.

---

## Application Logging

Log file: `%LOCALAPPDATA%\USBDeviceInspector\logs\usb_inspector.log`  
Rotation: 5 MB per file × 3 backups.

Logged events: startup/shutdown, enumeration, device connect/remove,
scan start/complete, inspector errors, controller start/stop, HID errors,
WMI failures, driver lookup failures.

---

## Data Flow

```
WMI Win32_PnPEntity
        │  enumerate_usb_descriptors()
        ▼
RawPnPDescriptor           ← raw OS fields, no interpretation
        │  normalize() + classify()
        ▼
USBDevice + DeviceCategory ← FieldSource badge on every fact
        │
        ▼
DeviceManager              ← thread-safe registry
        │
        ├─ DeviceListWidget
        │
        └─ DeviceScanner.scan(device)
                │  per-category inspector
                ▼
          DeviceDetails    ← tabbed sections + capabilities + warnings
                │
                └─ [Game Controller]
                        │
                        ├─ ControllerMonitorThread (120 Hz HID)
                        │        │  _on_state() → stores snapshot
                        │        │  QTimer 60 Hz → _render_frame()
                        │        ▼
                        │   ControllerTestWidget
                        │
                        └─ DroneSimulatorWindow
                                │  ControllerMonitorThread (120 Hz)
                                │  QTimer 60 Hz physics + render
                                ▼
                           DronePhysics.step(inp, dt)
                                │
                                ▼
                           Drone3DWidget.update_state()
                                │  paintEvent (QPainter)
                                ▼
                           Satellite 3D view + tactical HUD
```

---

## Security and Safety

- **Read-only.** Never writes to a USB device, never modifies drivers.
- **No serial numbers on disk.** History stores only non-identifying metadata.
- **No internet.** All data from local Windows PnP layer.
- **Graceful failure.** One bad device never aborts enumeration.
- **Provenance on every fact.** `FieldSource` badge on every field: `Directly Reported`, `Detected`, `Derived`, or `Unknown`.
- **Capabilities require evidence.** `add_capability_if()` needs a concrete boolean + justification string.

---

## Known Limitations

- **Windows only** — WMI/Win32 API dependency. Architecture supports replacing `app/usb/usb_enumerator.py` for Linux/macOS without touching UI or logic.
- **XInput controllers (Xbox)** — Windows intercepts HID reports for XInput devices. Live monitor unavailable; static scan still works.
- **Exclusive-access HID devices** — Report descriptor shows "Not accessible"; all other info still displayed.
- **USB Version** — `Win32_PnPEntity` does not expose USB 1.1/2.0/3.x for most devices.
- **Parent device** — Not directly exposed by `Win32_PnPEntity`.
- **Drone simulator** — Pure 2D oblique projection (no OpenGL). Performance adequate at 60 Hz on any modern CPU. Aircraft geometry is procedurally generated via QPainter — no external 3D model file required.
- **Drone physics** — Simplified altitude-hold model, not aerodynamically accurate. Designed for control responsiveness and crash-resistance. Control surface deflections (ailerons, elevator, rudder) are visual only — they do not feed back into the physics engine.
