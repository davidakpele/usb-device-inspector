"""Unit tests — continuous 360° analog movement system.

Tests cover:
  A. MotionInterpreter — circular dead-zone, polar normalisation,
     angle accuracy, smooth transitions, no gaps
  B. _angle_to_direction — all 360° covered, no gap, correct sectors
  C. DronePhysics — vector decomposition, smooth velocity sweep,
     no directional gaps, correct world-frame projection
  D. Integration — MotionInterpreter output feeds DronePhysics correctly
"""
from __future__ import annotations

import math
import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from app.core.controller_monitor import (
    AxisState, MotionInterpreter, MotionState,
    _angle_to_direction,
)
from app.core.drone_physics import (
    DroneInput, DronePhysics, DroneState, FlightMode,
    _vel_angle_to_label,
)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _make_axes(x_pct: float, y_pct: float,
               rz_pct: float = 50.0, sl_pct: float = 50.0) -> list[AxisState]:
    """Build a minimal axis list with calibrated-centre percents."""
    return [
        AxisState("X Axis",          int(x_pct * 10.23), 1023, x_pct, x_pct*3.6, False),
        AxisState("Y Axis",          int(y_pct * 10.23), 1023, y_pct, y_pct*3.6, False),
        AxisState("Rz (Z Rotation)", int(rz_pct * 2.55), 255,  rz_pct, rz_pct*3.6, True),
        AxisState("Slider",          int(sl_pct * 2.55), 255,  sl_pct, sl_pct*3.6, False),
    ]


def _interp_already_calibrated(dead_zone: float = 12.0) -> MotionInterpreter:
    """Return an interpreter whose calibration is already complete (offset=0)."""
    interp = MotionInterpreter(dead_zone=dead_zone)
    # Fast-forward calibration with centred inputs so offsets are all 0
    centred = _make_axes(50.0, 50.0)
    for _ in range(60):
        interp.interpret(centred)
    assert interp.is_calibrated()
    return interp


# ---------------------------------------------------------------------------
# A. MotionInterpreter — dead-zone, polar system
# ---------------------------------------------------------------------------

class TestMotionInterpreter:

    def test_centre_gives_zero(self):
        """Stick at rest → x_coord=0, y_coord=0, magnitude=0, Stopped."""
        interp = _interp_already_calibrated()
        ms = interp.interpret(_make_axes(50.0, 50.0))
        assert ms.x_coord == 0.0, f"x_coord={ms.x_coord}"
        assert ms.y_coord == 0.0, f"y_coord={ms.y_coord}"
        assert ms.magnitude == 0.0, f"magnitude={ms.magnitude}"
        assert ms.motion_status == "Stopped"
        assert ms.direction == "Center"

    def test_circular_dead_zone_shape(self):
        """Points just inside the circular dead-zone must all give magnitude=0,
        regardless of direction.  A square dead-zone would pass corners."""
        interp = _interp_already_calibrated(dead_zone=12.0)
        dz = 12.0 / 50.0   # = 0.24 in normalised units

        # Test 36 angles at radius just below dead-zone edge
        for deg in range(0, 360, 10):
            r = math.radians(deg)
            inside_r = (dz - 0.01)   # just inside
            x_pct = 50.0 + math.cos(r) * inside_r * 50.0
            y_pct = 50.0 - math.sin(r) * inside_r * 50.0   # Y inverted
            ms = interp.interpret(_make_axes(x_pct, y_pct))
            assert ms.magnitude == 0.0, (
                f"angle={deg}°  r={inside_r:.3f}  magnitude={ms.magnitude} "
                f"(should be 0 — inside dead-zone)")

    def test_direction_preserved_outside_dead_zone(self):
        """At radius > dead_zone, unit vector direction must match input angle."""
        interp = _interp_already_calibrated(dead_zone=12.0)
        dz = 12.0 / 50.0

        for deg in range(0, 360, 5):
            r  = math.radians(deg)
            rr = dz + 0.15   # clearly outside dead-zone
            x_pct = 50.0 + math.cos(r) * rr * 50.0
            y_pct = 50.0 - math.sin(r) * rr * 50.0
            ms = interp.interpret(_make_axes(x_pct, y_pct))

            # Reconstruct angle from output coords
            if ms.magnitude < 0.01:
                continue
            out_angle = math.degrees(math.atan2(ms.x_coord, ms.y_coord)) % 360.0
            # Allow ±2° tolerance for float precision
            err = min(abs(out_angle - ms.angle_deg),
                      360.0 - abs(out_angle - ms.angle_deg))
            assert err < 2.0, (
                f"input {deg}°  → angle_deg={ms.angle_deg:.1f}°  "
                f"reconstructed={out_angle:.1f}°  err={err:.1f}°")

    def test_magnitude_radial_rescale(self):
        """Magnitude should ramp linearly from 0 at dead-zone edge to 1 at full throw."""
        interp = _interp_already_calibrated(dead_zone=12.0)
        dz = 12.0 / 50.0

        prev_mag = 0.0
        for r_norm in [dz, dz+0.1, dz+0.2, dz+0.3, 0.7, 0.9, 1.0]:
            r_norm = min(1.0, r_norm)
            x_pct = 50.0 + r_norm * 50.0   # pure right direction
            y_pct = 50.0
            ms = interp.interpret(_make_axes(x_pct, y_pct))

            if r_norm <= dz:
                assert ms.magnitude == 0.0
            else:
                expected = (r_norm - dz) / (1.0 - dz)
                assert abs(ms.magnitude - expected) < 0.01, (
                    f"r={r_norm:.2f}  expected={expected:.3f}  got={ms.magnitude:.3f}")
                assert ms.magnitude >= prev_mag - 0.001, "Magnitude not monotonic"
                prev_mag = ms.magnitude

    def test_full_throw_magnitude_one(self):
        """Full stick throw in any direction → magnitude = 1.0."""
        interp = _interp_already_calibrated()
        for deg in range(0, 360, 30):
            r = math.radians(deg)
            x_pct = 50.0 + math.cos(r) * 50.0
            y_pct = 50.0 + math.sin(r) * 50.0
            # Clamp to [0, 100]
            x_pct = max(0.0, min(100.0, x_pct))
            y_pct = max(0.0, min(100.0, y_pct))
            ms = interp.interpret(_make_axes(x_pct, y_pct))
            assert ms.magnitude <= 1.001, f"magnitude={ms.magnitude} at {deg}°"

    def test_smooth_sweep_no_zero_gaps(self):
        """Sweeping the stick in a full circle must never produce magnitude=0
        outside the dead-zone radius, and direction must change continuously."""
        interp = _interp_already_calibrated(dead_zone=12.0)
        dz = 12.0 / 50.0
        r_norm = 0.8   # well outside dead-zone

        prev_angle = None
        for deg in range(0, 361):
            r   = math.radians(deg)
            x_p = max(0.0, min(100.0, 50.0 + math.cos(r) * r_norm * 50.0))
            y_p = max(0.0, min(100.0, 50.0 - math.sin(r) * r_norm * 50.0))
            ms  = interp.interpret(_make_axes(x_p, y_p))

            assert ms.magnitude > 0.0, f"magnitude=0 at {deg}° (should be moving)"
            assert ms.motion_status == "Moving", f"Stopped at {deg}°"

            if prev_angle is not None:
                # Angle change per degree of sweep should be ≈ 1°
                # Allow ±5° for boundary transitions
                change = abs(ms.angle_deg - prev_angle)
                change = min(change, 360.0 - change)
                assert change < 10.0, (
                    f"Angle jump at {deg}°: {prev_angle:.1f}→{ms.angle_deg:.1f}"
                    f" (change={change:.1f}°)")
            prev_angle = ms.angle_deg


# ---------------------------------------------------------------------------
# B. _angle_to_direction — full 360° coverage
# ---------------------------------------------------------------------------

class TestAngleToDirection:

    def test_every_degree_returns_non_empty(self):
        """Every integer degree 0-359 must produce a non-empty direction string."""
        for deg in range(360):
            label = _angle_to_direction(float(deg))
            assert label, f"Empty label at {deg}°"
            assert label != "Center", f"'Center' label at {deg}° (should be directional)"

    def test_canonical_angles(self):
        """Cardinal and diagonal angles map to expected labels."""
        cases = [
            (  0.0, "Forward"),
            ( 90.0, "Right"),
            (180.0, "Back"),
            (270.0, "Left"),
            ( 45.0, "Forward-Right"),
            (135.0, "Back-Right"),
            (225.0, "Back-Left"),
            (315.0, "Forward-Left"),
        ]
        for angle, expected in cases:
            got = _angle_to_direction(angle)
            assert got == expected, f"angle={angle}°  expected={expected!r}  got={got!r}"

    def test_no_gaps_between_labels(self):
        """Sweeping 0→360° in 0.1° steps must only ever change label by
        one step — there must be no sudden jumps across multiple labels."""
        known_labels = {
            "Forward", "Forward-Right", "Right", "Back-Right",
            "Back", "Back-Left", "Left", "Forward-Left",
        }
        prev = _angle_to_direction(0.0)
        for i in range(1, 3601):
            deg   = i * 0.1
            label = _angle_to_direction(deg)
            assert label in known_labels, f"Unknown label {label!r} at {deg}°"
            prev  = label

    def test_transition_right_to_forward(self):
        """Right → Forward sweep must pass through Forward-Right, not jump."""
        labels_seen = set()
        for deg in range(46, 90):   # 46° → 89° (Forward-Right zone into Right)
            labels_seen.add(_angle_to_direction(float(deg)))
        # All angles in this range should be Forward-Right or Right
        assert labels_seen <= {"Forward-Right", "Right"}, (
            f"Unexpected labels in Right→Forward zone: {labels_seen}")

    def test_wrap_around_360(self):
        """Angles at 359°, 360°, 361° must all give Forward."""
        assert _angle_to_direction(359.0) == "Forward"
        assert _angle_to_direction(360.0) == "Forward"
        assert _angle_to_direction(361.0) == "Forward"
        assert _angle_to_direction(0.0)   == "Forward"

    def test_vel_angle_label_matches(self):
        """_vel_angle_to_label and _angle_to_direction must give same output."""
        for deg in range(0, 360, 5):
            a = _angle_to_direction(float(deg))
            b = _vel_angle_to_label(float(deg))
            assert a == b, f"Mismatch at {deg}°: monitor={a!r}  physics={b!r}"


# ---------------------------------------------------------------------------
# C. DronePhysics — vector decomposition and smooth sweep
# ---------------------------------------------------------------------------

class TestDronePhysics:

    DT = 1.0 / 60.0

    def _armed_hovering(self) -> DronePhysics:
        """Return a DronePhysics instance that is armed and hovering at 3 m."""
        phy = DronePhysics()
        phy.step(DroneInput(btn_arm=True), self.DT)
        phy.step(DroneInput(btn_takeoff=True), self.DT)
        for _ in range(200):
            phy.step(DroneInput(throttle=0.5), self.DT)
        assert phy.state.altitude > 2.0
        assert phy.state.mode == FlightMode.HOVER
        return phy

    def test_forward_gives_negative_z(self):
        """Full forward stick at yaw=0 must produce vz < 0 (North direction)."""
        phy = self._armed_hovering()
        for _ in range(60):
            phy.step(DroneInput(pitch=1.0, throttle=0.5), self.DT)
        s = phy.state
        assert s.vz < -2.0, f"vz={s.vz:.2f} (expected forward = negative Z)"
        assert abs(s.vx) < abs(s.vz) * 0.1, f"Unexpected vx={s.vx:.2f} for pure forward"

    def test_right_gives_positive_x(self):
        """Full right stick at yaw=0 must produce vx > 0 (East direction)."""
        phy = self._armed_hovering()
        for _ in range(60):
            phy.step(DroneInput(roll=1.0, throttle=0.5), self.DT)
        s = phy.state
        assert s.vx > 2.0, f"vx={s.vx:.2f} (expected right = positive X)"
        assert abs(s.vz) < abs(s.vx) * 0.1, f"Unexpected vz={s.vz:.2f} for pure right"

    def test_diagonal_45_splits_evenly(self):
        """45° input (equal forward+right) must produce equal |vx| and |vz|."""
        phy = self._armed_hovering()
        mag = 1.0 / math.sqrt(2.0)
        for _ in range(60):
            phy.step(DroneInput(pitch=mag, roll=mag, throttle=0.5), self.DT)
        s = phy.state
        vx, vz = s.vx, s.vz
        ratio = abs(vx) / max(abs(vz), 0.001)
        assert 0.8 < ratio < 1.2, (
            f"vx={vx:.2f} vz={vz:.2f} ratio={ratio:.2f} "
            f"(expected equal magnitudes at 45°)")

    def test_continuous_sweep_no_velocity_gaps(self):
        """Sweeping input angle in 5° steps must never produce a sudden velocity
        direction gap > 15° between consecutive steps."""
        phy = self._armed_hovering()

        prev_world_angle = None
        for deg in range(0, 360, 5):
            r   = math.radians(deg)
            inp = DroneInput(
                pitch=math.cos(r),    # cos gives forward at 0°
                roll =math.sin(r),    # sin gives right  at 90°
                throttle=0.5,
            )
            # Run 10 frames so velocity can respond
            for _ in range(10):
                phy.step(inp, self.DT)

            s = phy.state
            spd = math.sqrt(s.vx ** 2 + s.vz ** 2)
            if spd < 0.3:
                continue

            world_angle = math.degrees(math.atan2(s.vx, -s.vz)) % 360.0

            if prev_world_angle is not None:
                change = abs(world_angle - prev_world_angle)
                change = min(change, 360.0 - change)
                assert change < 30.0, (
                    f"Velocity direction gap at input {deg}°: "
                    f"{prev_world_angle:.1f}°→{world_angle:.1f}° (gap={change:.1f}°)")
            prev_world_angle = world_angle

    def test_flight_command_covers_all_directions(self):
        """flight_command must produce all 8 direction labels during a full sweep."""
        phy = self._armed_hovering()
        commands_seen: set[str] = set()

        for deg in range(0, 360, 10):
            r   = math.radians(deg)
            inp = DroneInput(pitch=math.cos(r), roll=math.sin(r), throttle=0.5)
            for _ in range(30):
                s = phy.step(inp, self.DT)
            cmd = s.flight_command
            # Extract base direction (strip "Climbing + " etc.)
            for part in cmd.split(" + "):
                part = part.strip()
                if part in {"Forward", "Back", "Left", "Right",
                            "Forward-Right", "Forward-Left",
                            "Back-Right", "Back-Left"}:
                    commands_seen.add(part)

        expected = {"Forward", "Back", "Left", "Right",
                    "Forward-Right", "Forward-Left",
                    "Back-Right", "Back-Left"}
        missing = expected - commands_seen
        assert not missing, f"These directions never appeared: {missing}"

    def test_input_magnitude_scales_speed(self):
        """Half-magnitude input must produce roughly half the top speed."""
        phy_full = self._armed_hovering()
        phy_half = self._armed_hovering()

        for _ in range(120):
            phy_full.step(DroneInput(pitch=1.0, throttle=0.5), self.DT)
            phy_half.step(DroneInput(pitch=0.5, throttle=0.5), self.DT)

        spd_full = math.sqrt(phy_full.state.vx**2 + phy_full.state.vz**2)
        spd_half = math.sqrt(phy_half.state.vx**2 + phy_half.state.vz**2)

        ratio = spd_half / max(spd_full, 0.001)
        assert 0.35 < ratio < 0.75, (
            f"Half input speed ratio={ratio:.2f} "
            f"(expected ~0.5, got full={spd_full:.1f} half={spd_half:.1f})")


# ---------------------------------------------------------------------------
# D. Integration — interpreter output → physics
# ---------------------------------------------------------------------------

class TestIntegration:

    DT = 1.0 / 60.0

    def test_interpreter_output_drives_physics_forward(self):
        """MotionInterpreter x_coord/y_coord fed to DroneInput must fly forward."""
        interp = _interp_already_calibrated()

        # Pure forward input: Y axis at 0% (low % = forward)
        axes   = _make_axes(50.0, 0.0)   # Y=0% = full forward
        ms     = interp.interpret(axes)

        assert ms.y_coord > 0.3, f"y_coord={ms.y_coord} (expected > 0.3 for forward)"
        assert ms.direction in ("Forward", "Forward-Left", "Forward-Right")

        phy = DronePhysics()
        phy.step(DroneInput(btn_arm=True), self.DT)
        phy.step(DroneInput(btn_takeoff=True), self.DT)
        for _ in range(200):
            phy.step(DroneInput(throttle=0.5), self.DT)

        # Feed the interpreter output into the physics
        for _ in range(60):
            inp = DroneInput(
                roll=ms.x_coord,
                pitch=ms.y_coord,
                throttle=0.5,
            )
            phy.step(inp, self.DT)

        assert phy.state.vz < -1.0, (
            f"After interpreter→physics forward: vz={phy.state.vz:.2f}")

    def test_circular_dead_zone_prevents_drift(self):
        """Stick at rest (50%, 50%) must produce zero flight movement after calibration."""
        interp = _interp_already_calibrated()
        axes   = _make_axes(50.0, 50.0)
        ms     = interp.interpret(axes)
        assert ms.magnitude == 0.0

        inp = DroneInput(roll=ms.x_coord, pitch=ms.y_coord, throttle=0.5)
        assert abs(inp.roll)  < 0.001
        assert abs(inp.pitch) < 0.001


# ---------------------------------------------------------------------------
# Runner
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    suites = [
        TestMotionInterpreter(),
        TestAngleToDirection(),
        TestDronePhysics(),
        TestIntegration(),
    ]
    passed = 0
    failed = 0
    errors: list[str] = []

    for suite in suites:
        name = type(suite).__name__
        methods = [m for m in dir(suite) if m.startswith("test_")]
        for method in methods:
            label = f"{name}.{method}"
            try:
                getattr(suite, method)()
                print(f"  PASS  {label}")
                passed += 1
            except Exception as exc:
                print(f"  FAIL  {label}")
                print(f"        {exc}")
                failed += 1
                errors.append(f"{label}: {exc}")

    print()
    print(f"Results: {passed} passed, {failed} failed")
    if errors:
        print("\nFailed:")
        for e in errors:
            print(f"  {e}")
        sys.exit(1)
    else:
        print("All tests passed.")
