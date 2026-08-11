"""Drone flight physics engine — flyable edition.

Design goals
------------
* Forgiving: the drone should not crash the moment the pilot's thumb
  slips.  Altitude-hold is ON by default in all modes except MANUAL.
* Responsive: inputs translate to visible motion within 2-3 frames.
* Directional: pushing the stick forward moves the drone forward on
  screen, not backward.

Controller mapping (WingMan Extreme Digital 3D / any joystick)
--------------------------------------------------------------
  X axis  (x_coord   -1…+1)  →  Roll / strafe Left-Right
  Y axis  (y_coord   -1…+1)  →  Pitch Forward-Back
                                 y_coord > 0  ⟹  FORWARD
                                 y_coord < 0  ⟹  BACKWARD
  Rz/Twist (twist_norm -1…+1) →  Yaw (rotate): negative = CCW, positive = CW
  Slider   (0…1)             →  Altitude control (centred at 0.5 = hover)
                                 > 0.5 = climb, < 0.5 = descend
                                 At exactly 0.5 the drone holds altitude.

  Button 1  →  ARM / DISARM toggle
  Button 2  →  Emergency LAND (auto-descend and disarm)
  Button 3  →  HOVER / STABLE mode toggle
  Button 4  →  Reset (return to origin)
  Button 5  →  SPORT mode  (2× speed)
  Button 6  →  PRECISION mode (0.4× speed, very smooth)
  Button 7  →  AUTO TAKE-OFF (climbs to 3 m and holds)
  Hat N     →  Altitude trim +
  Hat S     →  Altitude trim −

Flight modes
------------
  DISARMED  — motors off, cannot fly
  ARMED     — altitude-hold active; left stick = altitude, right stick = direction
  HOVER     — same as ARMED but with extra position damping
  SPORT     — 2× speed, same altitude-hold
  PRECISION — 0.4× speed, same altitude-hold
  TAKEOFF   — auto-climb to hover altitude, then enter HOVER
  LANDING   — auto-descend and disarm

Altitude-hold model
-------------------
The slider is treated as a *climb rate* command, not raw throttle:
  slider 0.5  → 0 m/s vertical  (hold altitude)
  slider 1.0  → +MAX_CLIMB_RATE (full climb)
  slider 0.0  → −MAX_CLIMB_RATE (full descent)
  Dead-zone ±0.08 around 0.5 → exactly 0 vertical velocity

This means the drone hovers automatically when the slider is centred,
and the pilot only needs to nudge it to change altitude.  This is the
same model used in DJI Phantom / Mavic consumer drones.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from enum import Enum


# ---------------------------------------------------------------------------
# Tuneable constants
# ---------------------------------------------------------------------------

GROUND_Y         = 0.0     # Y=0 is the ground

# Horizontal movement
MAX_SPEED_NORMAL    = 8.0  # m/s horizontal
MAX_SPEED_SPORT     = 18.0
MAX_SPEED_PRECISION = 3.0
MAX_SPEED_HOVER     = 6.0

# Vertical movement
MAX_CLIMB_RATE   = 5.0     # m/s  full-up slider
MAX_DESCENT_RATE = 4.0     # m/s  full-down slider
ALT_HOLD_DZ      = 0.08    # dead-zone either side of slider centre (0.5)

# Yaw
MAX_YAW_RATE_NORMAL    = 100.0  # deg/s
MAX_YAW_RATE_SPORT     = 200.0
MAX_YAW_RATE_PRECISION = 50.0

# Tilt (for rate-controlled modes)
TILT_MAX   = 28.0   # deg maximum pitch/roll
TILT_SPEED = 10.0   # deg/s² responsiveness (higher = snappier)
TILT_DAMP  = 12.0   # return-to-level speed (deg/s when stick is centred)

# Drag (fraction of velocity removed per second)
DRAG_H     = 2.5    # horizontal (higher = stops faster when stick released)
DRAG_V     = 4.0    # vertical

# Auto modes
HOVER_ALT    = 3.0  # m  default take-off altitude
LAND_SPEED   = 1.2  # m/s auto-land descent speed


class FlightMode(Enum):
    DISARMED  = "DISARMED"
    ARMED     = "ARMED"
    HOVER     = "HOVER"
    SPORT     = "SPORT"
    PRECISION = "PRECISION"
    LANDING   = "LANDING"
    TAKEOFF   = "TAKEOFF"


# ---------------------------------------------------------------------------
# State
# ---------------------------------------------------------------------------

@dataclass
class DroneState:
    """Complete flight state — updated every physics step."""

    # World position (m)
    x: float = 0.0   # East/West
    y: float = 0.0   # Altitude
    z: float = 0.0   # North/South (forward = negative Z in world)

    # World velocity (m/s)
    vx: float = 0.0
    vy: float = 0.0
    vz: float = 0.0

    # Attitude (deg)
    pitch: float = 0.0   # + = nose up
    roll:  float = 0.0   # + = right wing down
    yaw:   float = 0.0   # 0 = north, + clockwise

    # Rotor
    rotor_speed:  float = 0.0
    rotor_angles: list[float] = field(
        default_factory=lambda: [0.0, 90.0, 180.0, 270.0])

    # Mode
    mode: FlightMode = FlightMode.DISARMED

    # Display/derived
    throttle:       float = 0.5    # slider value 0-1
    altitude:       float = 0.0
    speed_h:        float = 0.0
    speed_v:        float = 0.0
    heading:        float = 0.0
    flight_command: str   = "Disarmed — press Button 1 to ARM"
    is_airborne:    bool  = False
    total_distance: float = 0.0
    flight_time:    float = 0.0


# ---------------------------------------------------------------------------
# Inputs
# ---------------------------------------------------------------------------

@dataclass
class DroneInput:
    """Normalised pilot inputs for one physics step.

    All axis values are in -1…+1 (or 0…1 for throttle) with dead-zones
    already applied by the caller.
    """
    # Stick axes — pre-dead-zoned
    roll:     float = 0.0   # -1 = left,    +1 = right
    pitch:    float = 0.0   # -1 = back,    +1 = forward   ← correct convention
    yaw:      float = 0.0   # -1 = CCW,     +1 = CW
    throttle: float = 0.5   # 0 = full down, 0.5 = hold, 1.0 = full up

    # Button events — True only on the *rising edge* (pressed this frame,
    # not held).  The simulator handles edge detection.
    btn_arm:       bool = False
    btn_land:      bool = False
    btn_hover:     bool = False
    btn_reset:     bool = False
    btn_sport:     bool = False
    btn_precision: bool = False
    btn_takeoff:   bool = False

    # Hat
    hat_up:   bool = False
    hat_down: bool = False


# ---------------------------------------------------------------------------
# Physics engine
# ---------------------------------------------------------------------------

class DronePhysics:
    """60 Hz flight physics with altitude-hold.

    Call ``step(inp, dt)`` every frame.
    """

    def __init__(self) -> None:
        self.state   = DroneState()
        self._prev_btn: dict[str, bool] = {}
        self._target_alt = HOVER_ALT   # used by TAKEOFF / trim

    def step(self, inp: DroneInput, dt: float) -> DroneState:
        dt = max(0.0, min(dt, 0.05))   # cap at 50 ms
        s  = self.state

        self._handle_buttons(inp)

        if s.mode == FlightMode.DISARMED:
            self._on_ground(dt)
        elif s.mode == FlightMode.TAKEOFF:
            self._auto_takeoff(dt)
        elif s.mode == FlightMode.LANDING:
            self._auto_land(dt)
        else:
            self._fly(inp, dt)

        self._clamp_ground()
        self._update_rotors(inp, dt)
        self._update_derived(inp, dt)
        return s

    def reset(self) -> None:
        self.state   = DroneState()
        self._prev_btn = {}
        self._target_alt = HOVER_ALT

    # ------------------------------------------------------------------
    # Button edge detection (fire once per press)
    # ------------------------------------------------------------------

    def _edge(self, name: str, val: bool) -> bool:
        prev = self._prev_btn.get(name, False)
        self._prev_btn[name] = val
        return val and not prev

    def _handle_buttons(self, inp: DroneInput) -> None:
        s = self.state

        if self._edge("arm", inp.btn_arm):
            if s.mode == FlightMode.DISARMED:
                s.mode = FlightMode.ARMED
                s.flight_command = "Armed — raise slider to fly"
            else:
                s.mode = FlightMode.DISARMED
                s.vx = s.vy = s.vz = 0.0
                s.flight_command = "Disarmed — press Button 1 to ARM"

        if self._edge("land", inp.btn_land):
            if s.mode not in (FlightMode.DISARMED, FlightMode.LANDING):
                s.mode = FlightMode.LANDING
                s.flight_command = "Auto-Landing…"

        if self._edge("hover", inp.btn_hover):
            if s.mode in (FlightMode.ARMED, FlightMode.SPORT,
                          FlightMode.PRECISION):
                s.mode = FlightMode.HOVER
                s.flight_command = "Hover / Stable"
            elif s.mode == FlightMode.HOVER:
                s.mode = FlightMode.ARMED
                s.flight_command = "Armed"

        if self._edge("reset", inp.btn_reset):
            self.reset()
            return

        if self._edge("sport", inp.btn_sport):
            if s.mode not in (FlightMode.DISARMED, FlightMode.LANDING,
                              FlightMode.TAKEOFF):
                s.mode = FlightMode.SPORT
                s.flight_command = "⚡ Sport Mode"

        if self._edge("precision", inp.btn_precision):
            if s.mode not in (FlightMode.DISARMED, FlightMode.LANDING,
                              FlightMode.TAKEOFF):
                s.mode = FlightMode.PRECISION
                s.flight_command = "🎯 Precision Mode"

        if self._edge("takeoff", inp.btn_takeoff):
            if s.mode in (FlightMode.ARMED, FlightMode.HOVER) and s.y < 0.5:
                s.mode = FlightMode.TAKEOFF
                self._target_alt = HOVER_ALT
                s.flight_command = f"Taking off → {self._target_alt:.1f} m"

        # Altitude trim
        if inp.hat_up:
            self._target_alt = min(self._target_alt + 0.04, 40.0)
        if inp.hat_down:
            self._target_alt = max(self._target_alt - 0.04, 0.5)

    # ------------------------------------------------------------------
    # Flight dynamics
    # ------------------------------------------------------------------

    def _fly(self, inp: DroneInput, dt: float) -> None:
        s = self.state

        mode_limits = {
            FlightMode.ARMED:     (MAX_SPEED_NORMAL,    MAX_YAW_RATE_NORMAL),
            FlightMode.HOVER:     (MAX_SPEED_HOVER,     MAX_YAW_RATE_NORMAL),
            FlightMode.SPORT:     (MAX_SPEED_SPORT,     MAX_YAW_RATE_SPORT),
            FlightMode.PRECISION: (MAX_SPEED_PRECISION, MAX_YAW_RATE_PRECISION),
        }
        max_spd, max_yaw = mode_limits.get(s.mode,
                                           (MAX_SPEED_NORMAL, MAX_YAW_RATE_NORMAL))

        # ── Yaw ─────────────────────────────────────────────────────
        s.yaw = (s.yaw + inp.yaw * max_yaw * dt) % 360.0

        # ── Horizontal velocity target (body-frame → world-frame) ────
        # inp.pitch > 0 = forward = along the drone's nose direction
        yaw_rad = math.radians(s.yaw)
        # Drone nose points in (sin(yaw), 0, -cos(yaw)) world direction
        fwd_x =  math.sin(yaw_rad)
        fwd_z = -math.cos(yaw_rad)
        rgt_x =  math.cos(yaw_rad)
        rgt_z =  math.sin(yaw_rad)

        target_vx = (inp.pitch * fwd_x + inp.roll * rgt_x) * max_spd
        target_vz = (inp.pitch * fwd_z + inp.roll * rgt_z) * max_spd

        # Smooth acceleration toward target (snappy = 12, floaty = 4)
        accel = 12.0 if s.mode == FlightMode.SPORT else 8.0
        alpha_h = min(dt * accel, 1.0)
        s.vx += (target_vx - s.vx) * alpha_h
        s.vz += (target_vz - s.vz) * alpha_h

        # ── Vertical: altitude-hold model ────────────────────────────
        # Slider 0.5 = hold altitude (0 vertical velocity).
        # Outside the dead-zone: linear climb/descent rate command.
        # At 0% slider: MAX_DESCENT_RATE * 0.6 (gentle, not instant)
        # At 100% slider: MAX_CLIMB_RATE (full climb)
        slider = inp.throttle   # 0.0 – 1.0
        deviation = slider - 0.5
        if abs(deviation) < ALT_HOLD_DZ:
            # Inside dead-zone: damp vertical velocity to zero quickly
            s.vy *= max(0.0, 1.0 - DRAG_V * 3.0 * dt)
        else:
            # Scale deviation to climb-rate command
            # Descent side (deviation < 0): gentler max to prevent fast crashes
            scale = (abs(deviation) - ALT_HOLD_DZ) / (0.5 - ALT_HOLD_DZ)
            scale = max(0.0, min(1.0, scale))
            if deviation > 0:
                target_vy =  scale * MAX_CLIMB_RATE
            else:
                # Cap descent to 30 % of max so 0 % throttle gives a
                # gentle ~1.2 m/s descent — pilot has 2-3s to react.
                target_vy = -scale * MAX_DESCENT_RATE * 0.30
            alpha_v = min(dt * 6.0, 1.0)
            s.vy += (target_vy - s.vy) * alpha_v

        # ── Horizontal drag ──────────────────────────────────────────
        drag_factor = max(0.0, 1.0 - DRAG_H * dt)
        if abs(inp.pitch) < 0.02 and abs(inp.roll) < 0.02:
            # No stick input: brake harder
            s.vx *= max(0.0, 1.0 - DRAG_H * 3.0 * dt)
            s.vz *= max(0.0, 1.0 - DRAG_H * 3.0 * dt)
        else:
            s.vx *= drag_factor
            s.vz *= drag_factor

        # ── Clamp to speed limits ────────────────────────────────────
        h_spd = math.sqrt(s.vx ** 2 + s.vz ** 2)
        if h_spd > max_spd:
            scale = max_spd / h_spd
            s.vx *= scale
            s.vz *= scale
        s.vy = max(-MAX_DESCENT_RATE, min(MAX_CLIMB_RATE, s.vy))

        # ── Integrate position ───────────────────────────────────────
        s.x += s.vx * dt
        s.y += s.vy * dt
        s.z += s.vz * dt

        # ── Visual tilt (follows velocity for visual feedback) ───────
        # Pitch: forward velocity → nose down; backward → nose up
        world_fwd = s.vx * fwd_x + s.vz * fwd_z
        world_rgt = s.vx * rgt_x + s.vz * rgt_z
        target_pitch =  world_fwd / max_spd * -TILT_MAX
        target_roll  = -world_rgt / max_spd *  TILT_MAX
        tilt_alpha = min(dt * TILT_SPEED, 1.0)
        s.pitch += (target_pitch - s.pitch) * tilt_alpha
        s.roll  += (target_roll  - s.roll)  * tilt_alpha

        self._update_command(inp, s)

    # ------------------------------------------------------------------
    # Flight command label
    # ------------------------------------------------------------------

    def _update_command(self, inp: DroneInput, s: DroneState) -> None:
        h_moving = math.sqrt(s.vx ** 2 + s.vz ** 2) > 0.4
        v_moving = abs(s.vy) > 0.15

        if not h_moving and not v_moving:
            s.flight_command = "Stable Hover" if s.y > 0.1 else "Grounded"
            return

        parts: list[str] = []

        if s.vy > 0.15:
            parts.append("Climbing")
        elif s.vy < -0.15:
            parts.append("Descending")

        if h_moving:
            yaw_rad = math.radians(s.yaw)
            fwd_x =  math.sin(yaw_rad); fwd_z = -math.cos(yaw_rad)
            rgt_x =  math.cos(yaw_rad); rgt_z =  math.sin(yaw_rad)
            df = s.vx * fwd_x + s.vz * fwd_z
            dr = s.vx * rgt_x + s.vz * rgt_z
            fwd_s = ("Forward"  if df >  0.4
                     else "Backward" if df < -0.4 else "")
            rgt_s = ("Right"    if dr >  0.4
                     else "Left"     if dr < -0.4 else "")
            if fwd_s and rgt_s:
                parts.append(f"{fwd_s}-{rgt_s}")
            elif fwd_s:
                parts.append(fwd_s)
            elif rgt_s:
                parts.append(rgt_s)

        if abs(inp.yaw) > 0.15:
            parts.append("Rotating-CW" if inp.yaw > 0 else "Rotating-CCW")

        s.flight_command = " + ".join(parts) if parts else "Moving"

    # ------------------------------------------------------------------
    # Auto modes
    # ------------------------------------------------------------------

    def _auto_takeoff(self, dt: float) -> None:
        s = self.state
        remaining = self._target_alt - s.y
        if remaining > 0.05:
            climb = min(MAX_CLIMB_RATE, remaining * 3.0)
            s.vy = climb
            s.y += s.vy * dt
            s.flight_command = f"Taking Off → {self._target_alt:.1f} m"
        else:
            s.y   = self._target_alt
            s.vy  = 0.0
            s.mode = FlightMode.HOVER
            s.flight_command = "Stable Hover"

    def _auto_land(self, dt: float) -> None:
        s = self.state
        if s.y > 0.05:
            s.vy  = -LAND_SPEED
            s.vx *= max(0.0, 1.0 - 4.0 * dt)
            s.vz *= max(0.0, 1.0 - 4.0 * dt)
            s.y  += s.vy * dt
            s.flight_command = "Auto-Landing…"
        else:
            s.y = 0.0; s.vy = 0.0; s.vx = 0.0; s.vz = 0.0
            s.pitch = 0.0; s.roll = 0.0
            s.mode = FlightMode.DISARMED
            s.rotor_speed = 0.0
            s.flight_command = "Landed — press Button 1 to ARM again"

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _on_ground(self, dt: float) -> None:
        s = self.state
        s.vx = s.vy = s.vz = 0.0
        s.pitch *= max(0.0, 1.0 - 12.0 * dt)
        s.roll  *= max(0.0, 1.0 - 12.0 * dt)
        s.flight_command = "Disarmed — press Button 1 to ARM"

    def _clamp_ground(self) -> None:
        s = self.state
        if s.y <= GROUND_Y:
            s.y = GROUND_Y
            if s.vy < 0:
                s.vy = 0.0

    def _update_rotors(self, inp: DroneInput, dt: float) -> None:
        s = self.state
        if s.mode == FlightMode.DISARMED:
            target = 0.0
        elif s.mode in (FlightMode.TAKEOFF, FlightMode.LANDING):
            target = 0.7
        else:
            # Speed proportional to how hard the drone is working
            effort = max(abs(inp.pitch), abs(inp.roll),
                         abs(inp.yaw), abs(inp.throttle - 0.5) * 2)
            target = 0.4 + effort * 0.6
        s.rotor_speed += (target - s.rotor_speed) * min(dt * 10.0, 1.0)

        # Spin individual rotors (alternating directions)
        spin = s.rotor_speed * 900.0 * dt
        for i in range(4):
            d = 1 if i % 2 == 0 else -1
            s.rotor_angles[i] = (s.rotor_angles[i] + spin * d) % 360.0

    def _update_derived(self, inp: DroneInput, dt: float) -> None:
        s = self.state
        s.altitude  = round(max(0.0, s.y), 3)
        s.heading   = s.yaw % 360.0
        s.speed_h   = round(math.sqrt(s.vx ** 2 + s.vz ** 2), 2)
        s.speed_v   = round(s.vy, 2)
        s.is_airborne = s.y > 0.05
        s.throttle  = inp.throttle
        s.total_distance += s.speed_h * dt
        if s.is_airborne:
            s.flight_time += dt
