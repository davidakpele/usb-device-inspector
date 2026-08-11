"""Drone flight physics engine.

``DronePhysics`` takes normalised joystick inputs every frame and integrates
a realistic-enough flight model to make the simulator feel responsive and
natural.  It is pure Python with no external dependencies — only the stdlib
``math`` module is used so it can run on the Qt timer thread safely.

Controller mapping (Logitech WingMan / standard joystick layout):
  X axis  (-1…+1)  →  Roll    (bank left / right)
  Y axis  (-1…+1)  →  Pitch   (nose down = forward / nose up = back)
  Rz/Twist(-1…+1)  →  Yaw     (rotate left / right in place)
  Slider  (0…1)    →  Throttle (0 = full down / 1 = full up)
  Hat N/S          →  Altitude trim (nudge up / down)
  Button 1         →  ARM / DISARM toggle
  Button 2         →  Emergency LAND
  Button 3         →  HOVER / STABLE mode toggle
  Button 4         →  Reset position
  Button 5         →  Sport mode (2× speed)
  Button 6         →  Slow / Precision mode (0.4× speed)
  Button 7         →  Take-off (auto climb to hover altitude)

Flight modes
------------
  DISARMED  – motors off, no movement, drone sits on ground
  ARMED     – motors spinning at idle, responds to throttle
  HOVER     – auto-stabilise; small stick inputs produce drift, not pitch
  SPORT     – doubled speed limits, snappier response
  PRECISION – halved speed limits, very smooth
  LANDING   – auto-descend and disarm
  TAKEOFF   – auto-climb to hover altitude then switch to HOVER
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from enum import Enum


# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

GRAVITY        = 9.81          # m/s²
MASS           = 0.8           # kg
MAX_THRUST     = MASS * GRAVITY * 2.2   # N  (hover at ~45 % throttle)
HOVER_THROTTLE = 0.46          # throttle fraction that produces 1g thrust
GROUND_Y       = 0.0           # world Y = 0 is the ground

# Speed limits (m/s) per mode
_SPEED = {
    "NORMAL":    {"xy": 6.0,  "z": 4.0,  "yaw": 90.0},
    "SPORT":     {"xy": 14.0, "z": 8.0,  "yaw": 160.0},
    "PRECISION": {"xy": 2.5,  "z": 2.0,  "yaw": 40.0},
    "HOVER":     {"xy": 4.0,  "z": 3.0,  "yaw": 60.0},
}

# Physics tuning
DRAG_XY    = 0.85   # velocity damping per second (horizontal)
DRAG_Z     = 0.80   # velocity damping per second (vertical)
TILT_RATE  = 180.0  # deg/s max pitch / roll rate
TILT_MAX   = 30.0   # deg max pitch / roll angle
TILT_DAMP  = 8.0    # return-to-level rate (deg/s when no input)
YAW_RATE   = 120.0  # deg/s max yaw rate
HOVER_ALT  = 3.0    # m  default take-off / hover altitude
LAND_SPEED = 0.8    # m/s descent speed during auto-land


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
    """Complete flight state snapshot — read by the renderer every frame."""

    # World-space position (metres)
    x: float = 0.0     # East (+) / West (-)
    y: float = 0.0     # Altitude (up = +)
    z: float = 0.0     # South (+) / North (-)  (screen depth)

    # Velocity (m/s) in world space
    vx: float = 0.0
    vy: float = 0.0
    vz: float = 0.0

    # Attitude (degrees)
    pitch: float = 0.0   # + = nose up
    roll:  float = 0.0   # + = right wing down
    yaw:   float = 0.0   # 0 = north, + = clockwise

    # Rotor spin (0-1)
    rotor_speed: float = 0.0

    # Flight mode
    mode: FlightMode = FlightMode.DISARMED

    # Throttle level (0-1, as set by slider)
    throttle: float = 0.0

    # Derived / display
    altitude:       float = 0.0   # = y
    speed_h:        float = 0.0   # horizontal speed m/s
    speed_v:        float = 0.0   # vertical speed m/s
    heading:        float = 0.0   # = yaw
    flight_command: str   = "Idle"
    is_airborne:    bool  = False

    # Accumulated totals
    total_distance: float = 0.0
    flight_time:    float = 0.0

    # Rotor animation angles (individual, for visual effect)
    rotor_angles: list[float] = field(default_factory=lambda: [0.0, 0.0, 0.0, 0.0])


# ---------------------------------------------------------------------------
# Inputs (set by the simulator each frame from the joystick)
# ---------------------------------------------------------------------------

@dataclass
class DroneInput:
    """Normalised controller inputs for one physics step."""
    roll:     float = 0.0    # -1 (left) … +1 (right)
    pitch:    float = 0.0    # -1 (forward) … +1 (back)
    yaw:      float = 0.0    # -1 (CCW) … +1 (CW)
    throttle: float = 0.0    # 0 (down) … 1 (up)

    # Button events (True = pressed this frame)
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
    """Integrates drone flight physics given normalised joystick inputs.

    Call ``step(inputs, dt)`` every frame.  ``state`` is updated in-place
    and also returned for convenience.
    """

    def __init__(self) -> None:
        self.state = DroneState()
        self._prev_btn: dict[str, bool] = {}   # edge detection
        self._takeoff_target = HOVER_ALT

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def step(self, inp: DroneInput, dt: float) -> DroneState:
        """Advance physics by ``dt`` seconds given ``inp`` controller state."""
        dt = max(0.0, min(dt, 0.1))   # clamp to 100 ms max step
        s = self.state

        self._handle_buttons(inp)
        self._update_rotor_speed(inp, dt)

        if s.mode == FlightMode.DISARMED:
            self._apply_gravity_grounded(dt)
        elif s.mode == FlightMode.TAKEOFF:
            self._auto_takeoff(dt)
        elif s.mode == FlightMode.LANDING:
            self._auto_land(dt)
        else:
            self._apply_flight(inp, dt)

        self._clamp_to_ground()
        self._update_derived(inp, dt)
        return s

    def reset(self) -> None:
        """Reset to initial on-ground state."""
        self.state = DroneState()
        self._prev_btn = {}

    # ------------------------------------------------------------------
    # Button edge detection
    # ------------------------------------------------------------------

    def _edge(self, name: str, value: bool) -> bool:
        """Return True only on the rising edge (first press)."""
        prev = self._prev_btn.get(name, False)
        self._prev_btn[name] = value
        return value and not prev

    def _handle_buttons(self, inp: DroneInput) -> None:
        s = self.state

        if self._edge("arm", inp.btn_arm):
            if s.mode == FlightMode.DISARMED:
                s.mode = FlightMode.ARMED
                s.flight_command = "Armed"
            else:
                s.mode = FlightMode.DISARMED
                s.rotor_speed = 0.0
                s.flight_command = "Disarmed"

        if self._edge("land", inp.btn_land):
            if s.mode not in (FlightMode.DISARMED, FlightMode.LANDING):
                s.mode = FlightMode.LANDING
                s.flight_command = "Auto-Landing"

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
            if s.mode in (FlightMode.ARMED, FlightMode.HOVER,
                          FlightMode.PRECISION):
                s.mode = FlightMode.SPORT
                s.flight_command = "Sport Mode"

        if self._edge("precision", inp.btn_precision):
            if s.mode in (FlightMode.ARMED, FlightMode.HOVER,
                          FlightMode.SPORT):
                s.mode = FlightMode.PRECISION
                s.flight_command = "Precision Mode"

        if self._edge("takeoff", inp.btn_takeoff):
            if s.mode == FlightMode.ARMED and s.y < 0.3:
                s.mode = FlightMode.TAKEOFF
                self._takeoff_target = HOVER_ALT
                s.flight_command = "Auto Take-off"

        # Altitude trim via hat
        if inp.hat_up:
            self._takeoff_target = min(self._takeoff_target + 0.05, 30.0)
        if inp.hat_down:
            self._takeoff_target = max(self._takeoff_target - 0.05, 0.5)

    # ------------------------------------------------------------------
    # Rotor speed
    # ------------------------------------------------------------------

    def _update_rotor_speed(self, inp: DroneInput, dt: float) -> None:
        s = self.state
        if s.mode == FlightMode.DISARMED:
            target = 0.0
        elif s.mode in (FlightMode.TAKEOFF, FlightMode.LANDING):
            target = 0.6
        else:
            # Idle + throttle contribution
            target = 0.25 + inp.throttle * 0.75
        s.rotor_speed += (target - s.rotor_speed) * min(dt * 8.0, 1.0)
        # Spin individual rotors (alternating direction for visual)
        spin = s.rotor_speed * 720 * dt   # degrees per step
        for i in range(4):
            direction = 1 if i % 2 == 0 else -1
            s.rotor_angles[i] = (s.rotor_angles[i] + spin * direction) % 360.0

    # ------------------------------------------------------------------
    # Flight dynamics
    # ------------------------------------------------------------------

    def _apply_flight(self, inp: DroneInput, dt: float) -> None:
        s = self.state
        mode_key = {
            FlightMode.ARMED:     "NORMAL",
            FlightMode.HOVER:     "HOVER",
            FlightMode.SPORT:     "SPORT",
            FlightMode.PRECISION: "PRECISION",
        }.get(s.mode, "NORMAL")
        lim = _SPEED[mode_key]

        # ── Yaw ─────────────────────────────────────────────────────
        yaw_rate = inp.yaw * lim["yaw"]
        s.yaw = (s.yaw + yaw_rate * dt) % 360.0

        # ── Pitch / Roll (tilt) ─────────────────────────────────────
        # In HOVER mode inputs produce velocity directly; in other
        # modes they tilt the drone which then accelerates.
        if s.mode == FlightMode.HOVER:
            # Velocity-controlled: stick → horizontal velocity
            yaw_rad = math.radians(s.yaw)
            fwd_x =  math.sin(yaw_rad)
            fwd_z = -math.cos(yaw_rad)
            right_x =  math.cos(yaw_rad)
            right_z =  math.sin(yaw_rad)

            target_vx = (-inp.pitch * fwd_x + inp.roll * right_x) * lim["xy"]
            target_vz = (-inp.pitch * fwd_z + inp.roll * right_z) * lim["xy"]

            # Smooth approach to target velocity
            alpha = min(dt * 5.0, 1.0)
            s.vx += (target_vx - s.vx) * alpha
            s.vz += (target_vz - s.vz) * alpha

            # Visual tilt follows velocity
            s.pitch = -s.vy * 2.0
            s.roll  =  s.vx * 2.0
        else:
            # Rate-controlled: stick → tilt angle → acceleration
            target_pitch = -inp.pitch * TILT_MAX
            target_roll  =  inp.roll  * TILT_MAX
            tilt_alpha = min(dt * TILT_RATE / TILT_MAX, 1.0)
            s.pitch += (target_pitch - s.pitch) * tilt_alpha
            s.roll  += (target_roll  - s.roll)  * tilt_alpha

            # Body-frame tilt → world-frame acceleration
            yaw_rad   = math.radians(s.yaw)
            pitch_rad = math.radians(s.pitch)
            roll_rad  = math.radians(s.roll)

            ax = (math.sin(roll_rad)  * math.cos(pitch_rad)) * MAX_THRUST / MASS
            az = (-math.sin(pitch_rad)) * MAX_THRUST / MASS

            # Rotate to world frame
            world_ax = ax * math.cos(yaw_rad) - az * math.sin(yaw_rad)
            world_az = ax * math.sin(yaw_rad) + az * math.cos(yaw_rad)

            s.vx += world_ax * dt
            s.vz += world_az * dt

        # ── Throttle → vertical thrust ───────────────────────────────
        # Net vertical force = thrust - gravity
        thrust_ratio = inp.throttle / max(HOVER_THROTTLE, 0.01)
        net_accel_y = (thrust_ratio - 1.0) * GRAVITY
        s.vy += net_accel_y * dt

        # ── Drag ────────────────────────────────────────────────────
        s.vx *= max(0.0, 1.0 - DRAG_XY * dt)
        s.vz *= max(0.0, 1.0 - DRAG_XY * dt)
        s.vy *= max(0.0, 1.0 - DRAG_Z  * dt)

        # ── Clamp to speed limits ────────────────────────────────────
        h_speed = math.sqrt(s.vx ** 2 + s.vz ** 2)
        if h_speed > lim["xy"]:
            scale = lim["xy"] / h_speed
            s.vx *= scale
            s.vz *= scale
        s.vy = max(-lim["z"], min(lim["z"], s.vy))

        # ── Integrate position ───────────────────────────────────────
        s.x += s.vx * dt
        s.y += s.vy * dt
        s.z += s.vz * dt

        # ── Return-to-level when no input ───────────────────────────
        if s.mode != FlightMode.HOVER:
            if abs(inp.pitch) < 0.05:
                s.pitch *= max(0.0, 1.0 - TILT_DAMP * dt)
            if abs(inp.roll) < 0.05:
                s.roll  *= max(0.0, 1.0 - TILT_DAMP * dt)

        self._update_command(inp, s)

    def _update_command(self, inp: DroneInput, s: DroneState) -> None:
        """Derive a human-readable flight command from current motion."""
        moving_h = math.sqrt(s.vx ** 2 + s.vz ** 2) > 0.3
        moving_v = abs(s.vy) > 0.2

        if not moving_h and not moving_v:
            if s.y < 0.05:
                s.flight_command = "Grounded"
            else:
                s.flight_command = "Stable Hover"
            return

        parts: list[str] = []

        # Vertical
        if s.vy > 0.2:
            parts.append("Climbing")
        elif s.vy < -0.2:
            parts.append("Descending")

        # Horizontal — relative to drone heading
        if moving_h:
            yaw_rad = math.radians(s.yaw)
            fwd_x =  math.sin(yaw_rad)
            fwd_z = -math.cos(yaw_rad)
            right_x =  math.cos(yaw_rad)
            right_z =  math.sin(yaw_rad)

            dot_fwd   = s.vx * fwd_x   + s.vz * fwd_z
            dot_right = s.vx * right_x + s.vz * right_z

            fwd_s   = "Forward"  if dot_fwd   > 0.3 else ("Backward" if dot_fwd   < -0.3 else "")
            right_s = "Right"    if dot_right > 0.3 else ("Left"     if dot_right < -0.3 else "")

            if fwd_s and right_s:
                parts.append(f"{fwd_s}-{right_s}")
            elif fwd_s:
                parts.append(fwd_s)
            elif right_s:
                parts.append(right_s)

        if abs(inp.yaw) > 0.2:
            parts.append("Rotating-CW" if inp.yaw > 0 else "Rotating-CCW")

        s.flight_command = " + ".join(parts) if parts else "Moving"

    # ------------------------------------------------------------------
    # Auto modes
    # ------------------------------------------------------------------

    def _auto_takeoff(self, dt: float) -> None:
        s = self.state
        remaining = self._takeoff_target - s.y
        if remaining > 0.1:
            s.vy = min(3.0, remaining * 2.0)
            s.y += s.vy * dt
            s.flight_command = f"Taking Off → {self._takeoff_target:.1f}m"
        else:
            s.y   = self._takeoff_target
            s.vy  = 0.0
            s.mode = FlightMode.HOVER
            s.flight_command = "Stable Hover"

    def _auto_land(self, dt: float) -> None:
        s = self.state
        if s.y > 0.05:
            s.vy = -LAND_SPEED
            s.vx *= max(0.0, 1.0 - 3.0 * dt)
            s.vz *= max(0.0, 1.0 - 3.0 * dt)
            s.y += s.vy * dt
            s.flight_command = "Auto-Landing…"
        else:
            s.y    = 0.0
            s.vy   = 0.0
            s.vx   = 0.0
            s.vz   = 0.0
            s.pitch = 0.0
            s.roll  = 0.0
            s.mode = FlightMode.DISARMED
            s.rotor_speed = 0.0
            s.flight_command = "Landed"

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _apply_gravity_grounded(self, dt: float) -> None:
        s = self.state
        s.vx = s.vy = s.vz = 0.0
        s.pitch *= max(0.0, 1.0 - 10.0 * dt)
        s.roll  *= max(0.0, 1.0 - 10.0 * dt)

    def _clamp_to_ground(self) -> None:
        s = self.state
        if s.y <= GROUND_Y:
            s.y  = GROUND_Y
            if s.vy < 0:
                s.vy = 0.0
            # Hard landing: disarm if not already landing / disarmed
            if s.mode not in (FlightMode.DISARMED, FlightMode.LANDING,
                              FlightMode.TAKEOFF, FlightMode.ARMED):
                if s.vy < -2.0:
                    s.mode = FlightMode.DISARMED
                    s.flight_command = "Crashed!"

    def _update_derived(self, inp: DroneInput, dt: float) -> None:
        s = self.state
        s.altitude  = round(s.y, 3)
        s.heading   = s.yaw % 360.0
        s.speed_h   = round(math.sqrt(s.vx ** 2 + s.vz ** 2), 2)
        s.speed_v   = round(s.vy, 2)
        s.is_airborne = s.y > 0.05
        s.throttle  = inp.throttle
        dist = s.speed_h * dt
        s.total_distance += dist
        if s.is_airborne:
            s.flight_time += dt
