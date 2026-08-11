"""Real-time HID controller input monitor (spec section 20).

``ControllerMonitorThread`` runs on a dedicated QThread.  Every ~8 ms it
reads one raw HID input report from the device, decodes it into:

  * Per-axis values  — raw 0-65535 → percentage (0-100) + degrees (0-359)
  * Button states    — list of (button_index, pressed: bool)
  * Hat direction    — 8-way compass + "Centered"
  * Total rotation   — accumulated degrees for full-rotation axes

The decoded state is emitted as an ``InputState`` (a plain dataclass, not a
Qt object) via the ``state_updated`` signal so the UI thread can render it
without touching the hardware layer.

HID report decoding strategy
------------------------------
We do NOT re-parse the report descriptor on every poll — that was done once
during the scan (ControllerInspector) and the results (axis count, button
count, etc.) are passed in as ``AxisInfo`` / ``ButtonCount`` at construction.

Instead we use a pragmatic approach:
  * Read the raw bytes with ``hid.Device.read(64)``
  * Identify byte boundaries by trial and error against the scan-time
    report-descriptor analysis (axis usages → byte offsets)
  * For devices where we have the parsed descriptor we use precise byte
    offsets; for others we use the full-descriptor auto-detection fallback

The auto-detection fallback works well for standard DirectInput joysticks
(HID Usage 0x04) and gamepads (0x05) which follow a well-known layout:
  Bytes 0-1: X axis (16-bit little-endian)
  Bytes 2-3: Y axis
  Bytes 4-5: Z axis / throttle
  Bytes 6-7: Rx / rudder
  ...
  Last 2-4 bytes: buttons as bitmask, hat nibble

For XInput devices Windows intercepts reports before they reach user-space
HID — we cannot read them via hidapi in that case and the thread exits
cleanly with a ``monitor_error`` signal.
"""
from __future__ import annotations

import math
import time
from dataclasses import dataclass, field
from typing import Any

from PySide6.QtCore import QThread, Signal

from app.utils.logger import get_logger

logger = get_logger(__name__)

_POLL_INTERVAL_S = 0.008        # ~120 Hz
_READ_TIMEOUT_MS = 50
_HAT_DIRECTIONS = [
    "North", "North-East", "East", "South-East",
    "South", "South-West", "West", "North-West",
    "Centered",
]

# ---------------------------------------------------------------------------
# Data structures
# ---------------------------------------------------------------------------

@dataclass
class AxisState:
    name: str
    raw: int            # 0 – max_value
    max_value: int      # 255, 1023, 4095, 65535 depending on resolution
    percent: float      # 0.0 – 100.0  (centre = 50.0 for stick axes)
    degrees: float      # 0.0 – 359.9 for full-rotation axes; 0-180 for half
    is_rotation: bool   # True for Rz / rudder / twist axes


@dataclass
class ButtonState:
    index: int          # 1-based (Button 1 … Button N)
    pressed: bool
    label: str          # e.g. "Button 3" or custom label if known


@dataclass
class HatState:
    raw: int            # 0-7 = direction, 8 = centred (HID hat encoding)
    direction: str      # "North", "South-East", etc., or "Centered"
    degrees: float | None   # angle in degrees (0=N, 90=E, …) or None when centred


@dataclass
class MotionState:
    """High-level interpretation of the primary stick axes.

    Directions are mapped from the main X/Y axes (first two axes in the
    report).  Thresholds are configurable — default dead-zone is ±15 %
    from centre (50 %).

    Fields
    ------
    direction       — human label: Forward / Back / Left / Right /
                      Forward-Left / Forward-Right / Back-Left / Back-Right /
                      Center
    motion_status   — "Moving" or "Stopped"
    x_percent       — X axis 0-100 %, centre = 50
    y_percent       — Y axis 0-100 %, centre = 50
    x_coord         — floating-point logical coordinate, centre = 0.0,
                      range −1.0 … +1.0
    y_coord         — same for Y  (positive = forward in stick convention)
    angle_deg       — angle of the stick vector from north, 0-359.9°
                      (0 = straight forward, 90 = right, 180 = back …)
    magnitude       — distance from centre, 0.0 (centre) … 1.0 (full throw)
    twist_percent   — primary rotation axis (Rz/rudder/twist) 0-100 %
    twist_degrees   — twist angle 0-359.9°
    throttle_percent — slider/throttle 0-100 % (0 = full back, 100 = full fwd)
    """
    direction: str = "Center"
    motion_status: str = "Stopped"
    x_percent: float = 50.0
    y_percent: float = 50.0
    x_coord: float = 0.0       # −1.0 … +1.0
    y_coord: float = 0.0       # −1.0 … +1.0  (positive = forward)
    angle_deg: float = 0.0     # 0 = forward, clockwise
    magnitude: float = 0.0     # 0.0 … 1.0
    twist_percent: float = 50.0
    twist_degrees: float = 0.0
    throttle_percent: float = 0.0


@dataclass
class InputState:
    """Complete decoded state snapshot from one HID input report."""
    axes: list[AxisState] = field(default_factory=list)
    buttons: list[ButtonState] = field(default_factory=list)
    hat: HatState | None = None
    motion: MotionState = field(default_factory=MotionState)
    timestamp: float = 0.0          # time.monotonic()
    raw_bytes: bytes = b""


# ---------------------------------------------------------------------------
# Motion interpreter
# ---------------------------------------------------------------------------

# Axis name fragments that identify each logical role.
# We match the first axis whose name contains any of these fragments.
_X_HINTS    = ("x axis", "x-axis", " x ", "lx", "left x")
_Y_HINTS    = ("y axis", "y-axis", " y ", "ly", "left y")
_TWIST_HINTS = ("rz", "z rotation", "rudder", "twist", "yaw", "rx", "ry")
_THROT_HINTS = ("slider", "throttle", "z axis", "z-axis", "wheel", "dial")


def _find_axis(axes: list[AxisState], hints: tuple[str, ...]) -> AxisState | None:
    for axis in axes:
        n = axis.name.lower()
        if any(h in n for h in hints):
            return axis
    return None


class MotionInterpreter:
    """Converts a list of AxisState values into a MotionState.

    Dead-zone: any axis within ``dead_zone`` percent of centre (50.0) is
    treated as zero movement on that axis.  Default 15 % gives comfortable
    centre-release without false triggers.

    Direction vocabulary
    --------------------
    The primary stick (X/Y axes) maps to 8 directions + Center:
      Y < dead  → Forward   (stick pushed away from player)
      Y > dead  → Back      (stick pulled toward player)
      X < dead  → Left
      X > dead  → Right
    Diagonals are "Forward-Left" etc.
    When both axes are inside the dead-zone → Center.

    Motion status
    -------------
    "Moving"  — magnitude > dead_zone as fraction of full throw
    "Stopped" — magnitude ≤ dead_zone (stick effectively at rest)

    Coordinates
    -----------
    x_coord / y_coord are normalised to −1.0 … +1.0 with dead-zone
    applied (values inside dead-zone collapse to 0.0).
    angle_deg is measured clockwise from north (forward = 0°).
    """

    def __init__(self, dead_zone: float = 15.0) -> None:
        self.dead_zone = dead_zone       # percent from centre (0-50)

    def interpret(self, axes: list[AxisState]) -> MotionState:
        ms = MotionState()
        if not axes:
            return ms

        x_ax  = _find_axis(axes, _X_HINTS)
        y_ax  = _find_axis(axes, _Y_HINTS)
        tw_ax = _find_axis(axes, _TWIST_HINTS)
        th_ax = _find_axis(axes, _THROT_HINTS)

        # ── X / Y stick ─────────────────────────────────────────────
        x_pct = x_ax.percent if x_ax else 50.0
        y_pct = y_ax.percent if y_ax else 50.0

        ms.x_percent = round(x_pct, 1)
        ms.y_percent = round(y_pct, 1)

        # Normalise to −1.0 … +1.0 (centre = 0)
        x_norm = (x_pct - 50.0) / 50.0
        y_norm = (y_pct - 50.0) / 50.0    # positive = stick pulled back
        # Convention: positive Y_coord = Forward (invert raw Y)
        y_norm_fwd = -y_norm

        dz = self.dead_zone / 50.0         # dead-zone in normalised units

        # Apply dead-zone
        x_dz = x_norm if abs(x_norm) > dz else 0.0
        y_dz = y_norm_fwd if abs(y_norm_fwd) > dz else 0.0

        ms.x_coord = round(max(-1.0, min(1.0, x_dz)), 3)
        ms.y_coord = round(max(-1.0, min(1.0, y_dz)), 3)

        # Magnitude and angle
        magnitude = math.sqrt(x_dz ** 2 + y_dz ** 2)
        magnitude = min(1.0, magnitude)
        ms.magnitude = round(magnitude, 3)

        # Angle: atan2(x, y_forward) gives clockwise-from-north
        if magnitude > 0.001:
            angle_rad = math.atan2(x_dz, y_dz)
            angle_deg = math.degrees(angle_rad)
            if angle_deg < 0:
                angle_deg += 360.0
            ms.angle_deg = round(angle_deg, 1)
        else:
            ms.angle_deg = 0.0

        # ── Direction label ──────────────────────────────────────────
        h_dir = ""   # horizontal component
        v_dir = ""   # vertical component

        if x_dz > 0:
            h_dir = "Right"
        elif x_dz < 0:
            h_dir = "Left"

        if y_dz > 0:
            v_dir = "Forward"
        elif y_dz < 0:
            v_dir = "Back"

        if v_dir and h_dir:
            ms.direction = f"{v_dir}-{h_dir}"
        elif v_dir:
            ms.direction = v_dir
        elif h_dir:
            ms.direction = h_dir
        else:
            ms.direction = "Center"

        # ── Motion status ────────────────────────────────────────────
        ms.motion_status = "Moving" if magnitude > 0.001 else "Stopped"

        # ── Twist / rudder ───────────────────────────────────────────
        if tw_ax:
            ms.twist_percent = round(tw_ax.percent, 1)
            ms.twist_degrees = round(tw_ax.degrees, 1)
        else:
            ms.twist_percent = 50.0
            ms.twist_degrees = 0.0

        # ── Throttle / slider ────────────────────────────────────────
        if th_ax:
            ms.throttle_percent = round(th_ax.percent, 1)
        else:
            ms.throttle_percent = 0.0

        return ms


# ---------------------------------------------------------------------------
# Descriptor-driven axis map (produced by ControllerInspector)
# ---------------------------------------------------------------------------

@dataclass
class AxisDescriptor:
    """Describes one axis as parsed from the HID report descriptor."""
    name: str
    byte_offset: int    # byte offset in the input report
    bit_size: int       # bits (8, 10, 12, 16 are common)
    logical_min: int
    logical_max: int
    is_rotation: bool


# ---------------------------------------------------------------------------
# Report decoder
# ---------------------------------------------------------------------------

class ReportDecoder:
    """Decodes raw HID report bytes into an InputState.

    Supports two modes:
      1. Descriptor-driven — uses AxisDescriptor list when available
      2. Bit-size-aware    — uses per-axis bit sizes from _HIDAnalysis
      3. Auto-detect       — heuristically decodes standard HID layouts
    """

    def __init__(
        self,
        axis_names: list[str],
        button_count: int,
        has_hat: bool,
        axis_descriptors: list[AxisDescriptor] | None = None,
        axis_bit_sizes: list[int] | None = None,
    ) -> None:
        self._axis_names = axis_names
        self._button_count = button_count
        self._has_hat = has_hat
        self._descriptors = axis_descriptors
        # Per-axis bit sizes from the parsed report descriptor.
        # When present, this gives exact decoding without descriptor-level detail.
        self._axis_bit_sizes: list[int] = axis_bit_sizes or []
        self._interpreter = MotionInterpreter(dead_zone=15.0)

        # Pre-compute axis bit layout so we decode the same way every poll.
        self._axis_layout: list[tuple[str, int, int]] = []  # (name, bit_offset, bit_size)
        self._hat_bit_offset: int = 0
        self._btn_bit_offset: int = 0
        self._compute_layout()

    def _compute_layout(self) -> None:
        """Pre-compute bit offsets for axes, hat, and buttons."""
        bit_cursor = 0
        n_axes = len(self._axis_names)

        # Use provided bit sizes, or auto-detect uniform width
        if len(self._axis_bit_sizes) == n_axes and n_axes > 0:
            sizes = self._axis_bit_sizes
        else:
            # Auto-detect: prefer 10-bit if we have enough data headroom
            # (will be refined in decode() when we know report length)
            sizes = []  # empty = defer to _auto_decode_axes

        for i, name in enumerate(self._axis_names):
            bits = sizes[i] if i < len(sizes) else 10
            self._axis_layout.append((name, bit_cursor, bits))
            bit_cursor += bits

        self._hat_bit_offset = bit_cursor
        if self._has_hat:
            bit_cursor += 4  # hat is always 4 bits in standard HID

        self._btn_bit_offset = bit_cursor

    def decode(self, data: bytes) -> InputState:
        state = InputState(timestamp=time.monotonic(), raw_bytes=data)
        if not data:
            return state

        if self._descriptors:
            state.axes = self._decode_axes_from_descriptors(data)
        elif self._axis_layout and all(bits > 0 for _, _, bits in self._axis_layout):
            state.axes = self._decode_axes_from_layout(data)
        else:
            state.axes = self._auto_decode_axes(data)

        state.buttons = self._decode_buttons(data)
        state.hat     = self._decode_hat(data)
        state.motion  = self._interpreter.interpret(state.axes)
        return state

    # ------------------------------------------------------------------
    # Layout-driven decoding (uses pre-computed bit offsets)
    # ------------------------------------------------------------------

    def _decode_axes_from_layout(self, data: bytes) -> list[AxisState]:
        axes = []
        for name, bit_offset, bit_size in self._axis_layout:
            raw = _extract_bits(data, bit_offset, bit_size)
            if raw is None:
                continue
            max_val = (1 << bit_size) - 1
            pct = (raw / max_val) * 100.0 if max_val else 0.0
            deg = (raw / max_val) * 359.9 if max_val else 0.0
            is_rot = any(k in name.lower() for k in ("rotation", "rz", "rx", "ry", "rudder", "twist"))
            axes.append(AxisState(
                name=name, raw=raw, max_value=max_val,
                percent=round(pct, 1), degrees=round(deg, 1),
                is_rotation=is_rot,
            ))
        return axes

    # ------------------------------------------------------------------
    # Descriptor-driven decoding
    # ------------------------------------------------------------------

    def _decode_axes_from_descriptors(self, data: bytes) -> list[AxisState]:
        axes = []
        for desc in self._descriptors:
            raw = _extract_bits(data, desc.byte_offset * 8, desc.bit_size)
            if raw is None:
                continue
            max_val = (1 << desc.bit_size) - 1
            pct = (raw / max_val) * 100.0 if max_val else 0.0
            deg = (raw / max_val) * 359.9 if max_val else 0.0
            axes.append(AxisState(
                name=desc.name, raw=raw, max_value=max_val,
                percent=round(pct, 1), degrees=round(deg, 1),
                is_rotation=desc.is_rotation,
            ))
        return axes

    # ------------------------------------------------------------------
    # Auto-detect decoding (standard HID joystick/gamepad layout)
    # ------------------------------------------------------------------

    def _auto_decode_axes(self, data: bytes) -> list[AxisState]:
        """Decode axes from a raw HID input report.

        Strategy:
        1. If we have exactly the axis names from the parsed report descriptor,
           try to match the known bit-packed layout produced by _parse_report_descriptor.
           Most DirectInput joysticks pack axes LSB-first at the start of the report.
        2. For 10-bit axes (common on Logitech/Thrustmaster sticks) use bitfield extraction.
        3. For 8-bit axes fall back to byte-aligned reads.
        4. Never crash — return whatever we can decode.
        """
        axes = []
        n_axes = len(self._axis_names)
        if n_axes == 0 or not data:
            return axes

        # Estimate axis bit-width from report length.
        # Leave room for hat (4 bits) + buttons (ceil(N/8) bytes) at end.
        tail_bytes = math.ceil(self._button_count / 8) + (1 if self._has_hat else 0)
        available_bits = (len(data) - tail_bytes) * 8
        if available_bits <= 0:
            available_bits = len(data) * 8

        # Determine bits-per-axis that fits all axes in the available space.
        # Prefer 10-bit (common for quality sticks), fall back to 8-bit.
        if available_bits >= n_axes * 10:
            bits_per_axis = 10
            max_val = 1023
        else:
            bits_per_axis = 8
            max_val = 255

        bit_cursor = 0
        for name in self._axis_names[:n_axes]:
            raw = _extract_bits(data, bit_cursor, bits_per_axis)
            if raw is None:
                break
            bit_cursor += bits_per_axis
            pct = (raw / max_val) * 100.0
            is_rot = any(k in name.lower() for k in ("rotation", "rz", "rx", "ry", "rudder", "twist"))
            deg = (raw / max_val) * 359.9
            axes.append(AxisState(
                name=name, raw=raw, max_value=max_val,
                percent=round(pct, 1), degrees=round(deg, 1),
                is_rotation=is_rot,
            ))
        return axes

    # ------------------------------------------------------------------
    # Button decoding
    # ------------------------------------------------------------------

    def _decode_buttons(self, data: bytes) -> list[ButtonState]:
        """Extract button states from the pre-computed bit offset."""
        buttons = []
        if self._button_count == 0:
            return buttons

        # Use pre-computed offset when available
        if self._btn_bit_offset > 0:
            btn_bit_start = self._btn_bit_offset
        else:
            # Fallback: estimate from report tail
            n_axes = len(self._axis_names)
            tail_bytes = math.ceil(self._button_count / 8) + (1 if self._has_hat else 0)
            available_bits = (len(data) - tail_bytes) * 8
            bits_per_axis = 10 if (available_bits >= n_axes * 10) else 8
            btn_bit_start = n_axes * bits_per_axis + (4 if self._has_hat else 0)

        bitmask = 0
        btn_bytes = math.ceil(self._button_count / 8)
        for i in range(btn_bytes):
            raw = _extract_bits(data, btn_bit_start + i * 8, 8)
            if raw is not None:
                bitmask |= raw << (i * 8)

        bitmask &= (1 << self._button_count) - 1

        for b in range(self._button_count):
            pressed = bool(bitmask & (1 << b))
            buttons.append(ButtonState(
                index=b + 1, pressed=pressed, label=f"Button {b + 1}",
            ))
        return buttons

    # ------------------------------------------------------------------
    # Hat decoding
    # ------------------------------------------------------------------

    def _decode_hat(self, data: bytes) -> HatState | None:
        """Extract hat switch value from the pre-computed bit offset."""
        if not self._has_hat or not data:
            return None

        if self._hat_bit_offset > 0:
            hat_bit_offset = self._hat_bit_offset
        else:
            n_axes = len(self._axis_names)
            tail_bytes = math.ceil(self._button_count / 8) + 1
            available_bits = (len(data) - tail_bytes) * 8
            bits_per_axis = 10 if (available_bits >= n_axes * 10) else 8
            hat_bit_offset = n_axes * bits_per_axis

        hat_raw = _extract_bits(data, hat_bit_offset, 4)
        if hat_raw is None:
            return None

        if hat_raw > 8:
            hat_raw = 8
        direction = _HAT_DIRECTIONS[hat_raw] if hat_raw < len(_HAT_DIRECTIONS) else "Centered"
        hat_degrees_map = [0.0, 45.0, 90.0, 135.0, 180.0, 225.0, 270.0, 315.0]
        deg = hat_degrees_map[hat_raw] if hat_raw < 8 else None
        return HatState(raw=hat_raw, direction=direction, degrees=deg)


# ---------------------------------------------------------------------------
# Bit-extraction helper
# ---------------------------------------------------------------------------

def _extract_bits(data: bytes, bit_offset: int, bit_count: int) -> int | None:
    """Extract an unsigned integer of ``bit_count`` bits starting at ``bit_offset``."""
    byte_start = bit_offset // 8
    byte_end = (bit_offset + bit_count + 7) // 8
    if byte_end > len(data):
        return None
    chunk = data[byte_start:byte_end]
    value = int.from_bytes(chunk, "little")
    # Shift to align the field to bit 0
    value >>= bit_offset % 8
    # Mask to bit_count bits
    value &= (1 << bit_count) - 1
    return value


# ---------------------------------------------------------------------------
# Monitor thread
# ---------------------------------------------------------------------------

class ControllerMonitorThread(QThread):
    """Polls the HID device at ~120 Hz and emits decoded InputState snapshots.

    Signals
    -------
    state_updated(InputState)   — emitted on every successful read
    monitor_error(str)          — emitted when the device cannot be opened
                                  or is disconnected during monitoring
    """

    state_updated = Signal(object)   # InputState
    monitor_error = Signal(str)

    def __init__(
        self,
        vid: int,
        pid: int,
        axis_names: list[str],
        button_count: int,
        has_hat: bool,
        hid_path: bytes | None = None,
        axis_bit_sizes: list[int] | None = None,
        parent=None,
    ) -> None:
        super().__init__(parent)
        self._vid = vid
        self._pid = pid
        self._axis_names = axis_names
        self._button_count = button_count
        self._has_hat = has_hat
        self._hid_path = hid_path
        self._running = False
        self._decoder = ReportDecoder(
            axis_names, button_count, has_hat,
            axis_bit_sizes=axis_bit_sizes,
        )

    def run(self) -> None:
        self._running = True
        try:
            import hid  # type: ignore
        except ImportError:
            self.monitor_error.emit(
                "The 'hidapi' package is not installed. "
                "Install with: pip install hidapi"
            )
            return

        dev = None
        try:
            dev = self._open_device(hid)
        except Exception as exc:  # noqa: BLE001
            self.monitor_error.emit(
                f"Could not open controller for live monitoring: {exc}\n"
                "The device may be held exclusively by another driver (e.g. XInput)."
            )
            return

        logger.info(
            "Controller monitor started: VID=%04X PID=%04X  axes=%d  buttons=%d  hat=%s",
            self._vid, self._pid, len(self._axis_names), self._button_count, self._has_hat,
        )

        try:
            dev.set_nonblocking(False)
            while self._running:
                try:
                    data = dev.read(_READ_TIMEOUT_MS)
                except Exception as exc:  # noqa: BLE001
                    if self._running:
                        logger.debug("Controller read error: %s", exc)
                        self.monitor_error.emit(f"Controller disconnected: {exc}")
                    break

                if data:
                    state = self._decoder.decode(bytes(data))
                    self.state_updated.emit(state)
                else:
                    # Non-blocking: nothing to read yet; small sleep to avoid spin.
                    time.sleep(_POLL_INTERVAL_S)
        finally:
            try:
                dev.close()
            except Exception:  # noqa: BLE001
                pass
            logger.info("Controller monitor stopped")

    def stop(self) -> None:
        self._running = False
        self.wait(500)

    def _open_device(self, hid):
        """Open via preferred path first, fall back to VID/PID.

        Uses the hidapi (Cython) API:
          hid.device() — create device object
          .open_path(path) — open by path bytes
          .open(vid, pid)  — open by VID/PID
        """
        # Try preferred path (joystick/gamepad interface) first.
        if self._hid_path:
            try:
                dev = hid.device()
                dev.open_path(self._hid_path)
                return dev
            except Exception:  # noqa: BLE001
                pass

        # Enumerate all interfaces; prefer Generic Desktop joystick/gamepad usage.
        for info in hid.enumerate(self._vid, self._pid):
            up = info.get("usage_page", 0)
            u  = info.get("usage", 0)
            # Generic Desktop (0x01): joystick (4), gamepad (5), multi-axis (8)
            if up == 0x01 and u in (0x04, 0x05, 0x08):
                path = info.get("path")
                if path:
                    try:
                        dev = hid.device()
                        dev.open_path(path)
                        return dev
                    except Exception:  # noqa: BLE001
                        continue

        # Last resort: open first available interface by VID/PID.
        dev = hid.device()
        dev.open(self._vid, self._pid)
        return dev
