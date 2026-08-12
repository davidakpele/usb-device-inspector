"""Fix drone_3d_view.py: restore all missing def lines and fix incomplete _qpt calls."""
import re, pathlib

src = pathlib.Path("app/ui/drone_3d_view.py").read_text(encoding="utf-8")

# ---------------------------------------------------------------------------
# 1. Restore missing def lines
# ---------------------------------------------------------------------------

FIXES = [
    # (section comment, missing def line to insert before the body)
    (
        "    # Tactical grid — 5 m spacing, 50 m sector labels\n    # ------------------------------------------------------------------\n\n    \n        s   = self._state",
        "    # Tactical grid — 5 m spacing, 50 m sector labels\n    # ------------------------------------------------------------------\n\n    def _draw_grid(self, p, cx, cy, cc, sc, z=90.0) -> None:\n        s   = self._state",
    ),
    (
        "    # Origin marker — runway-style cross\n    # ------------------------------------------------------------------\n\n    \n        L = 3.0",
        "    # Origin marker — runway-style cross\n    # ------------------------------------------------------------------\n\n    def _draw_origin_marker(self, p, cx, cy, cc, sc, z=90.0) -> None:\n        L = 3.0",
    ),
    (
        "    # Ground compass with large tactical labels\n    # ------------------------------------------------------------------\n\n    \n        s = self._state\n        R = 12.0",
        "    # Ground compass with large tactical labels\n    # ------------------------------------------------------------------\n\n    def _draw_compass_ground(self, p, cx, cy, cc, sc, z=90.0) -> None:\n        s = self._state\n        R = 12.0",
    ),
    (
        "    # ==================================================================\n    # TRAIL\n    # ==================================================================\n\n    \n        trail = list(self._trail)",
        "    # ==================================================================\n    # TRAIL\n    # ==================================================================\n\n    def _draw_trail(self, p, cx, cy, cc, sc, z=90.0) -> None:\n        trail = list(self._trail)",
    ),
    (
        "    # ==================================================================\n    # SHADOW\n    # ==================================================================\n\n    \n        s = self._state\n        if s.y < 0.1:",
        "    # ==================================================================\n    # SHADOW\n    # ==================================================================\n\n    def _draw_shadow(self, p, cx, cy, cc, sc, z=90.0) -> None:\n        s = self._state\n        if s.y < 0.1:",
    ),
    (
        "    # ==================================================================\n    # DRONE — MQ-style quadrotor UAV\n    # ==================================================================\n\n    \n        s   = self._state\n        ARM = 0.75",
        "    # ==================================================================\n    # DRONE — MQ-style quadrotor UAV\n    # ==================================================================\n\n    def _draw_drone(self, p, cx, cy, cc, sc, z=90.0) -> None:\n        s   = self._state\n        ARM = 0.75",
    ),
    (
        "        p.drawText(int(nsx)-14, int(nsy)-16, 28, 12,\n                   Qt.AlignmentFlag.AlignCenter, \"FWD\")\n\n    \n        R     = 0.55",
        "        p.drawText(int(nsx)-14, int(nsy)-16, 28, 12,\n                   Qt.AlignmentFlag.AlignCenter, \"FWD\")\n\n    def _draw_rotors(self, p, cx, cy, cc, sc, motors, s: DroneState, z=90.0) -> None:\n        R     = 0.55",
    ),
    (
        "    # ==================================================================\n    # VELOCITY VECTOR\n    # ==================================================================\n\n    \n        s   = self._state\n        spd = s.speed_h",
        "    # ==================================================================\n    # VELOCITY VECTOR\n    # ==================================================================\n\n    def _draw_velocity_vector(self, p, cx, cy, cc, sc, z=90.0) -> None:\n        s   = self._state\n        spd = s.speed_h",
    ),
]

for old, new in FIXES:
    if old in src:
        src = src.replace(old, new)
        print(f"Fixed: {new.split(chr(10))[4].strip()[:60]}")
    else:
        print(f"NOT FOUND (may already be fixed): {new.split(chr(10))[4].strip()[:60]}")

# ---------------------------------------------------------------------------
# 2. Fix _qpt / _proj calls missing the z argument
#    These look like: _qpt(..., cx, cy, cc, sc)  with no z at the end
# ---------------------------------------------------------------------------

# Fix _qpt calls missing z
before = src.count("cc, sc)")
src = re.sub(r"(_qpt\([^)]+), cx, cy, cc, sc\)", r"\1, cx, cy, cc, sc, z)", src)
after_qpt = src.count("cc, sc)")

# Fix _proj calls missing z
src = re.sub(r"(_proj\([^)]+), cx, cy, cc, sc\)", r"\1, cx, cy, cc, sc, z)", src)
after_proj = src.count("cc, sc)")

# Remove any accidental double-z insertions
double_z = src.count("cc, sc, z, z)")
if double_z:
    src = src.replace(", cc, sc, z, z)", ", cc, sc, z)")
    print(f"Removed {double_z} double-z insertions")

print(f"_qpt/_proj calls fixed: {before - after_proj} remaining bare 'cc, sc)' calls")

# ---------------------------------------------------------------------------
# 3. Write back
# ---------------------------------------------------------------------------
pathlib.Path("app/ui/drone_3d_view.py").write_bytes(src.encode("utf-8"))
print("Written (UTF-8, no BOM)")
