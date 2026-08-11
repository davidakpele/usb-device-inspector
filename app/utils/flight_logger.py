"""Flight diagnostic logger — flight_logger.py

Writes a complete Markdown trace file for every drone simulator session.
Records:
  - Session header (controller, start time, log path)
  - Axis calibration offsets
  - Axis diagnostic snapshot
  - Every event (mode change, command, button press, error, keyboard action)
  - Raw controller input table  (sampled 1 Hz)
  - Flight telemetry table      (sampled 1 Hz)
  - Session summary on close

Output: %LOCALAPPDATA%/USBDeviceInspector/logs/flight_YYYYMMDD_HHMMSS.md

The tick counter is advanced by tick() and shared across log_input /
log_state so both tables always stay in sync.
"""
from __future__ import annotations

import threading
from datetime import datetime
from pathlib import Path

_LOG_DIR = (Path.home() / "AppData" / "Local"
            / "USBDeviceInspector" / "logs")

_TELEM_INTERVAL  = 60    # write one row per second (60 Hz physics)
_MAX_INPUT_ROWS  = 3000


class FlightLogger:
    """Thread-safe Markdown flight logger."""

    def __init__(self, device_name: str = "Unknown Controller") -> None:
        self._lock   = threading.Lock()
        self._closed = False
        self._tick   = 0           # advanced once per physics step
        self._input_rows  = 0
        self._telem_rows  = 0
        self._input_table = False
        self._telem_table = False
        self._start       = datetime.now()

        _LOG_DIR.mkdir(parents=True, exist_ok=True)
        ts = self._start.strftime("%Y%m%d_%H%M%S")
        self._path = _LOG_DIR / f"flight_{ts}.md"
        self._fh   = self._path.open("w", encoding="utf-8")
        self._write_header(device_name)

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    @property
    def path(self) -> Path:
        return self._path

    def tick(self) -> None:
        """Advance the internal clock by one physics step (call every frame)."""
        self._tick += 1

    def log_calibration(self, offsets: dict[str, float],
                        frames_used: int = 60) -> None:
        with self._lock:
            self._close_all()
            self._w("## Axis Calibration\n\n")
            self._w(f"*Measured over first {frames_used} frames.*\n\n")
            self._w("| Axis | Raw Rest % | Offset | Status |\n|---|---|---|---|\n")
            for ax, off in sorted(offsets.items()):
                raw  = round(50.0 + off, 2)
                qual = "✓ centred" if abs(off) < 2.0 else f"⚠ corrected {off:+.2f}%"
                self._w(f"| {ax} | {raw} | {off:+.2f}% | {qual} |\n")
            self._w("\n")
            self._flush()

    def log_axis_diagnostic(self, axes_raw: dict[str, float],
                            axes_cal: dict[str, float]) -> None:
        with self._lock:
            self._close_all()
            self._w("## Axis Diagnostic Snapshot\n\n")
            self._w("| Axis | Raw % | Calibrated % | Offset | Status |\n|---|---|---|---|---|\n")
            for name in axes_raw:
                raw = axes_raw[name]
                cal = axes_cal.get(name, raw)
                off = raw - cal
                st  = ("✓ centred"      if abs(cal-50)<3
                       else "⚠ drifting" if abs(cal-50)<10
                       else "❌ large offset")
                self._w(f"| {name} | {raw:.1f} | {cal:.1f} | {off:+.1f} | {st} |\n")
            self._w("\n")
            self._flush()

    def log_event(self, event_type: str, detail: str) -> None:
        icons = {
            "INFO": "ℹ", "ARM": "🔧", "DISARM": "🔒",
            "TAKEOFF": "🚀", "LAND": "⬇", "MODE": "🔄",
            "COMMAND": "✈", "ERROR": "❌", "WARN": "⚠",
            "CALIB": "📐", "BUTTON": "🔘",
        }
        icon = icons.get(event_type, "·")
        ts   = self._ts()
        with self._lock:
            # Events can appear inline after a table section; close any open
            # table only if writing to keep Markdown valid.
            self._close_all()
            self._w(f"**{ts}** {icon} `{event_type}` — {detail}\n\n")
            self._flush()

    def log_input(self, roll: float, pitch: float, yaw: float,
                  throttle: float, x_coord: float, y_coord: float,
                  twist_pct: float, thr_pct: float,
                  btns_pressed: list[int]) -> None:
        """Stored for the combined row — call log_state after to flush."""
        # Just cache the values; they are written in log_state
        self._pending_input = (roll, pitch, yaw, throttle,
                               x_coord, y_coord, twist_pct, thr_pct,
                               btns_pressed)

    def log_state(self, mode: str, command: str,
                  altitude: float, speed_h: float, speed_v: float,
                  heading: float, pitch: float, roll: float,
                  x: float, z: float, flight_time: float) -> None:
        """Write one combined row to the single periodic data table."""
        if self._tick % _TELEM_INTERVAL != 0:
            return
        if self._telem_rows >= _MAX_INPUT_ROWS:
            return
        pi = getattr(self, "_pending_input", None)
        with self._lock:
            if not self._telem_table:
                self._w("## Flight Data (1 Hz sample)\n\n")
                self._w("| Time | Mode | Command | Alt | H-Spd | V-Spd |"
                        " Hdg° | Pitch° | Roll° | X | Z | FT |"
                        " inp.Roll | inp.Pitch | inp.Yaw | Thr | Btns |\n")
                self._w("|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|\n")
                self._telem_table = True

            btns = "—"
            i_roll = i_pitch = i_yaw = i_thr = 0.0
            if pi:
                i_roll, i_pitch, i_yaw, i_thr, _, _, _, _, bp = pi
                btns = ", ".join(str(b) for b in sorted(bp)) or "—"

            self._w(
                f"| {self._ts()} | {mode} | {command} "
                f"| {altitude:.2f} | {speed_h:.2f} | {speed_v:+.2f} "
                f"| {heading:.1f} | {pitch:+.1f} | {roll:+.1f} "
                f"| {x:.2f} | {z:.2f} | {flight_time:.1f} "
                f"| {i_roll:+.3f} | {i_pitch:+.3f} | {i_yaw:+.3f}"
                f"| {i_thr:.3f} | {btns} |\n")
            self._telem_rows += 1
            self._flush()

    def log_error(self, source: str, message: str) -> None:
        self.log_event("ERROR", f"**{source}**: {message}")

    def close(self) -> None:
        if self._closed:
            return
        with self._lock:
            self._close_all()
            elapsed = (datetime.now() - self._start).total_seconds()
            self._w(f"\n---\n\n## Session Summary\n\n"
                    f"| Field | Value |\n|---|---|\n"
                    f"| End time | {datetime.now():%Y-%m-%d %H:%M:%S} |\n"
                    f"| Duration | {elapsed:.1f} s |\n"
                    f"| Input rows | {self._input_rows} |\n"
                    f"| Telemetry rows | {self._telem_rows} |\n"
                    f"| Total ticks | {self._tick} |\n\n"
                    f"*Log: `{self._path.name}`*\n")
            self._fh.flush()
            self._fh.close()
            self._closed = True

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _write_header(self, device_name: str) -> None:
        now = self._start
        self._w(f"# USB Device Inspector — Flight Log\n\n"
                f"| Field | Value |\n|---|---|\n"
                f"| Session start | {now:%Y-%m-%d %H:%M:%S} |\n"
                f"| Controller    | {device_name} |\n"
                f"| Log file      | `{self._path.name}` |\n\n"
                f"---\n\n")
        self._flush()

    def _w(self, text: str) -> None:
        if not self._closed:
            self._fh.write(text)

    def _flush(self) -> None:
        if not self._closed:
            self._fh.flush()

    def _ts(self) -> str:
        return datetime.now().strftime("%H:%M:%S")

    def _close_input_table(self) -> None:
        if self._input_table:
            self._w("\n")
            self._input_table = False

    def _close_telem_table(self) -> None:
        if self._telem_table:
            self._w("\n")
            self._telem_table = False

    def _close_all(self) -> None:
        self._close_input_table()
        self._close_telem_table()
