"""Military-grade 3D drone renderer — drone_3d_view.py

Design language: tactical operations centre / UAV ground-control station.

Environment
-----------
* Deep night-operations sky with star field and horizon glow
* Large tactical grid (5 m spacing) with 50 m sector labels
* Terrain surface: dark olive/grey with subtle texture lines
* Runway-style centre cross-hair at origin
* Range rings (25 m, 50 m, 100 m) from origin — tactical distance markers
* Sector boundary markers every 45° on the outer ring

Aircraft — Realistic Fixed-Wing Reconnaissance UAV
---------------------------------------------------
Coordinate convention (body frame):
  +X  = right wing tip direction
  +Y  = up (dorsal)
  −Z  = forward (nose direction)   ← standard aerospace convention

Model hierarchy (all in body frame, rotated to world via b2w):
  Fuselage          — tapered hexagonal cross-section, nose ogive, tail boom
  MainWings         — swept trapezoidal panels, left and right
  HorizontalStab    — smaller horizontal tail panels, left and right
  VerticalStab      — single dorsal fin
  LeftAileron       — trailing-edge panel on left wing (deflects with roll)
  RightAileron      — trailing-edge panel on right wing (deflects opposite)
  Elevator          — trailing edge of horizontal stab (deflects with pitch)
  Rudder            — trailing edge of vertical fin (deflects with yaw)
  SensorPod         — streamlined blister below nose
  PropDisc          — single forward tractor propeller (blur disc at speed)
  LandingGear       — nose gear + two main gear (retract above 1.5 m AGL)
  NavLights         — wingtip + tail strobes (aviation standard colours)

Control surface animation
--------------------------
  DroneState.roll  → aileron deflection  (±AILERON_MAX deg)
  DroneState.pitch → elevator deflection (±ELEVATOR_MAX deg)
  DroneState.yaw   → rudder deflection   (rate-based, ±RUDDER_MAX deg)
  DroneState.rotor_speed → propeller blur disc opacity + spinner rotation

HUD — tactical GCS style (unchanged from original)
---------------------------------------------------
* Top-centre: threat/mode badge
* Top-left:   full telemetry panel
* Top-right:  digital compass rose
* Bottom-left: artificial horizon (ADI)
* Bottom-right: altitude + throttle ladder tapes
* Bottom-right-corner: radar mini-map
* Right edge: status strip
"""
from __future__ import annotations

import math
import random
from collections import deque

from PySide6.QtCore import QPointF, QRectF, Qt
from PySide6.QtGui import (
    QColor, QFont, QFontMetrics, QLinearGradient,
    QPainter, QPainterPath, QPen, QBrush,
    QPolygonF, QRadialGradient,
)
from PySide6.QtWidgets import QWidget

from app.core.drone_physics import DroneState, FlightMode

# ===========================================================================
# Scene constants
# ===========================================================================

SCALE     = 90.0    # pixels per metre — base scale (zoom overrides this)
TRAIL_LEN = 300     # position history length

# ---------------------------------------------------------------------------
# Tactical colour palette — military amber/green-on-black
# ---------------------------------------------------------------------------

_C_BG       = QColor(6,   8,  12)
_C_SKY_MID  = QColor(8,  14,  24)
_C_SKY_HRZ  = QColor(18,  32,  22)
_C_GRID     = QColor(32,  52,  32, 80)
_C_GRID_AX  = QColor(40,  90,  40, 140)
_C_TERRAIN  = QColor(18,  28,  14)
_C_AMBER    = QColor(255, 176,   0)
_C_GREEN    = QColor( 40, 220,  80)
_C_RED      = QColor(220,  40,  40)
_C_BLUE     = QColor( 60, 160, 240)
_C_WHITE    = QColor(230, 240, 220)
_C_DIM      = QColor( 80, 100,  80)

# ---------------------------------------------------------------------------
# Fixed-wing UAV colour palette — aerospace composite / matte tactical
# ---------------------------------------------------------------------------

# Fuselage — matte grey-green composite (think RQ-4 / Predator tones)
_C_FUSE_DARK   = QColor(42,  48,  44)   # base composite
_C_FUSE_MID    = QColor(62,  70,  64)   # panel mid
_C_FUSE_LIGHT  = QColor(88,  98,  90)   # highlight edge
_C_FUSE_BELLY  = QColor(52,  58,  54)   # underside slightly lighter

# Wings / stabilisers
_C_WING_TOP    = QColor(50,  58,  52)   # dorsal surface
_C_WING_BOTTOM = QColor(68,  76,  70)   # ventral surface (lighter)
_C_WING_LE     = QColor(30,  36,  32)   # leading-edge dark strip
_C_WING_TIP    = QColor(34,  40,  36)

# Control surfaces — slight contrast to distinguish from fixed structure
_C_CS_NEUTRAL  = QColor(56,  66,  58)
_C_CS_ACTIVE   = QColor(76,  90,  78)
_C_CS_EDGE     = QColor(28,  34,  30)

# Sensor / camera pod
_C_SENSOR_BODY = QColor(24,  28,  26)
_C_SENSOR_LENS = QColor(20,  40,  80, 220)
_C_LENS_REFL   = QColor(160, 200, 255, 60)

# Propeller
_C_PROP_HUB    = QColor(30,  36,  32)
_C_PROP_BLADE  = QColor(22,  28,  24)
_C_PROP_DISC   = QColor(50,  70,  50, 80)   # blur disc at speed

# Landing gear
_C_GEAR_STRUT  = QColor(28,  34,  30)
_C_GEAR_WHEEL  = QColor(20,  24,  22)

# Nav lights — aviation standard
_C_NAV_GRN  = QColor(  0, 255,  80, 240)   # port (left) wingtip = green
_C_NAV_RED  = QColor(255,  30,  30, 240)   # starboard (right) wingtip = red
_C_STROBE   = QColor(255, 255, 220, 240)   # tail strobe = white

# Panel lines
_C_PANEL    = QColor(22,  26,  23, 180)

# Mode colours (HUD)
_MODE_RGB: dict[str, tuple[int,int,int]] = {
    "DISARMED":  ( 60,  70,  60),
    "ARMED":     (200, 160,   0),
    "HOVER":     ( 40, 180,  80),
    "SPORT":     (220,  60,  40),
    "PRECISION": ( 40, 160, 220),
    "LANDING":   (180, 140,   0),
    "TAKEOFF":   ( 40, 200, 120),
}

_PN = Qt.PenStyle.NoPen   # shorthand

# ---------------------------------------------------------------------------
# Fixed-wing UAV geometry constants (body frame, metres)
# ---------------------------------------------------------------------------

# Fuselage
FUSE_LENGTH  = 2.20   # nose-to-tail total length
FUSE_NOSE_Z  = -1.10  # nose tip (−Z = forward)
FUSE_TAIL_Z  =  1.10  # tail end
FUSE_W_MAX   =  0.18  # max half-width at widest section
FUSE_H_MAX   =  0.14  # max half-height
FUSE_NOSE_PX =  0.60  # fuselage section where max width occurs (fraction)

# Main wings  (semi-span, root chord, tip chord, sweep of leading edge)
WING_SPAN_HALF = 1.50   # half-span (each side)
WING_ROOT_Z_LE = -0.30  # root leading-edge Z (slight forward sweep)
WING_ROOT_Z_TE =  0.45  # root trailing-edge Z
WING_TIP_Z_LE  =  0.10  # tip leading-edge Z (swept back relative to root)
WING_TIP_Z_TE  =  0.55  # tip trailing-edge Z
WING_ROOT_Y    =  0.00  # wing root Y (flush with lower fuselage)
WING_TIP_DY    = -0.04  # small dihedral (tip below root)
WING_THICK     =  0.04  # wing maximum thickness

# Aileron (inboard edge, outboard edge as fraction of half-span)
AILE_SPAN_IN   = 0.55   # inboard at 55 % of half-span
AILE_SPAN_OUT  = 0.92   # outboard at 92 %
AILE_CHORD     = 0.22   # aileron chord as fraction of local wing chord
AILERON_MAX    = 22.0   # degrees max deflection

# Horizontal stabiliser
HSTAB_SPAN_HALF = 0.55
HSTAB_ROOT_Z_LE =  0.72
HSTAB_ROOT_Z_TE =  0.98
HSTAB_TIP_Z_LE  =  0.78
HSTAB_TIP_Z_TE  =  1.00
HSTAB_Y         = -0.02   # slightly below fuselage centre
ELEV_CHORD      =  0.30   # elevator chord fraction of hstab chord
ELEVATOR_MAX    = 20.0

# Vertical stabiliser
VSTAB_BASE_Z    =  0.70   # root Z (at tail)
VSTAB_TIP_Z     =  0.95
VSTAB_BASE_Y    =  0.00
VSTAB_TIP_Y     =  0.38   # height of fin
VSTAB_LE_OFFSET = -0.10   # leading-edge forward offset at tip
RUDDER_CHORD    =  0.32   # rudder chord fraction
RUDDER_MAX      = 18.0

# Sensor / camera pod (under nose)
SENSOR_Z        = -0.75   # forward position
SENSOR_Y        = -0.10   # below fuselage
SENSOR_RADIUS   =  0.065

# Propeller (nose tractor)
PROP_Z          = -1.12   # just forward of nose
PROP_RADIUS     =  0.28   # blade tip radius
PROP_BLADES     =  2      # two-blade propeller
PROP_SPINNER_R  =  0.045

# Landing gear
NOSE_GEAR_Z     = -0.70
NOSE_GEAR_Y     = -0.18
MAIN_GEAR_X     =  0.30
MAIN_GEAR_Z     =  0.10
MAIN_GEAR_Y     = -0.18
WHEEL_RADIUS    =  0.048

# Engine / exhaust (twin exhausts at rear belly for realism)
ENGINE_Y        = -0.08
ENGINE_Z        =  0.90
ENGINE_RADIUS   =  0.04

# ===========================================================================
# Projection helpers  (identical to original — do not change)
# ===========================================================================

def _proj(wx: float, wy: float, wz: float,
          cx: float, cy: float,
          cc: float, sc: float,
          scale: float = 90.0) -> tuple[float, float]:
    """Oblique projection — nearly top-down / satellite look."""
    rx =  wx * cc + wz * sc
    rz = -wx * sc + wz * cc
    DEPTH_X = 0.20
    DEPTH_Y = 0.58
    return (cx + rx * scale - rz * scale * DEPTH_X,
            cy - wy * scale + rz * scale * DEPTH_Y)


def _qpt(wx, wy, wz, cx, cy, cc, sc, scale=90.0) -> QPointF:
    sx, sy = _proj(wx, wy, wz, cx, cy, cc, sc, scale)
    return QPointF(sx, sy)


def _arrow_head(p: QPainter, tip: QPointF, base: QPointF,
                head_len=14, head_w=7) -> None:
    dx = tip.x() - base.x(); dy = tip.y() - base.y()
    ln = math.sqrt(dx * dx + dy * dy)
    if ln < 1:
        return
    ux, uy = dx / ln, dy / ln
    px, py = -uy, ux
    poly = QPolygonF([
        tip,
        QPointF(tip.x() - ux * head_len + px * head_w,
                tip.y() - uy * head_len + py * head_w),
        QPointF(tip.x() - ux * head_len - px * head_w,
                tip.y() - uy * head_len - py * head_w),
    ])
    p.setPen(_PN)
    p.drawPolygon(poly)


# ===========================================================================
# Fixed-Wing UAV geometry builder
# ===========================================================================

class _FixedWingUAV:
    """Generates and draws all geometry for the fixed-wing reconnaissance UAV.

    All coordinates are defined in body frame and transformed to world frame
    via the b2w() closure provided by Drone3DWidget._draw_drone().

    Body-frame coordinate convention:
        +X  right wing tip
        +Y  up (dorsal)
        −Z  forward (nose)

    The b2w(bx, by, bz) function applies roll → pitch → yaw rotation then
    translates to world position (drone.x, drone.y, drone.z).
    """

    # ------------------------------------------------------------------ #
    # Fuselage — tapered hexagonal cross-section                          #
    # ------------------------------------------------------------------ #

    @staticmethod
    def _fuselage_cross(fz: float) -> list[tuple[float, float, float]]:
        """Return 8-point cross-section polygon of the fuselage at station fz.

        The cross-section is widest at FUSE_NOSE_PX fraction and tapers
        toward nose (ogive) and tail boom.
        """
        # Taper factor: 0 at nose/tail, 1 at widest point
        t = (fz - FUSE_NOSE_Z) / (FUSE_TAIL_Z - FUSE_NOSE_Z)  # 0→1 nose to tail
        # Nose ogive: narrow up to ~20 % then open to full chord
        if t < 0.18:
            taper = math.sin(t / 0.18 * math.pi * 0.5)
        elif t < 0.55:
            taper = 1.0
        else:
            # Tail boom taper — shrinks to ~20 % at extreme tail
            taper = 1.0 - 0.8 * ((t - 0.55) / 0.45) ** 1.5

        hw = FUSE_W_MAX * taper   # half-width
        hh = FUSE_H_MAX * taper   # half-height

        # 8-point hex-ish cross-section: top, top-right, right, btm-right,
        # bottom, btm-left, left, top-left
        pts = [
            ( 0.0,   hh,       fz),    # top
            ( hw*0.6, hh*0.65, fz),    # top-right
            ( hw,    0.0,      fz),    # right
            ( hw*0.6,-hh*0.65, fz),    # btm-right
            ( 0.0,  -hh,       fz),    # bottom
            (-hw*0.6,-hh*0.65, fz),    # btm-left
            (-hw,    0.0,      fz),    # left
            (-hw*0.6, hh*0.65, fz),    # top-left
        ]
        return pts

    @classmethod
    def draw_fuselage(cls, p: QPainter, b2w, wp) -> None:
        """Draw fuselage as a series of longitudinal quads with shading."""
        # Sample stations from nose to tail
        stations = []
        for i in range(17):
            fz = FUSE_NOSE_Z + i * (FUSE_TAIL_Z - FUSE_NOSE_Z) / 16
            stations.append(cls._fuselage_cross(fz))

        # Draw longitudinal "planks" (quad strips connecting adjacent stations)
        # Top surface — lighter
        for seg in range(len(stations) - 1):
            front = stations[seg]
            rear  = stations[seg + 1]
            # Top quad: vertices 0,1,7 form top arc; 0 is apex
            tq = QPolygonF([
                QPointF(*wp(*b2w(*front[0]))),
                QPointF(*wp(*b2w(*front[1]))),
                QPointF(*wp(*b2w(*rear[1]))),
                QPointF(*wp(*b2w(*rear[0]))),
            ])
            p.setBrush(QBrush(_C_FUSE_LIGHT))
            p.setPen(QPen(_C_PANEL, 0.5))
            p.drawPolygon(tq)

            tq2 = QPolygonF([
                QPointF(*wp(*b2w(*front[7]))),
                QPointF(*wp(*b2w(*front[0]))),
                QPointF(*wp(*b2w(*rear[0]))),
                QPointF(*wp(*b2w(*rear[7]))),
            ])
            p.setBrush(QBrush(_C_FUSE_MID))
            p.setPen(QPen(_C_PANEL, 0.5))
            p.drawPolygon(tq2)

            # Side quads
            side_r = QPolygonF([
                QPointF(*wp(*b2w(*front[1]))),
                QPointF(*wp(*b2w(*front[2]))),
                QPointF(*wp(*b2w(*rear[2]))),
                QPointF(*wp(*b2w(*rear[1]))),
            ])
            p.setBrush(QBrush(_C_FUSE_MID))
            p.setPen(QPen(_C_PANEL, 0.5))
            p.drawPolygon(side_r)

            side_l = QPolygonF([
                QPointF(*wp(*b2w(*front[6]))),
                QPointF(*wp(*b2w(*front[7]))),
                QPointF(*wp(*b2w(*rear[7]))),
                QPointF(*wp(*b2w(*rear[6]))),
            ])
            p.setBrush(QBrush(_C_FUSE_MID))
            p.setPen(QPen(_C_PANEL, 0.5))
            p.drawPolygon(side_l)

            # Bottom quads (belly — slightly lighter for visual separation)
            bq = QPolygonF([
                QPointF(*wp(*b2w(*front[3]))),
                QPointF(*wp(*b2w(*front[4]))),
                QPointF(*wp(*b2w(*rear[4]))),
                QPointF(*wp(*b2w(*rear[3]))),
            ])
            p.setBrush(QBrush(_C_FUSE_BELLY))
            p.setPen(QPen(_C_PANEL, 0.5))
            p.drawPolygon(bq)

            bq2 = QPolygonF([
                QPointF(*wp(*b2w(*front[4]))),
                QPointF(*wp(*b2w(*front[5]))),
                QPointF(*wp(*b2w(*rear[5]))),
                QPointF(*wp(*b2w(*rear[4]))),
            ])
            p.setBrush(QBrush(_C_FUSE_BELLY))
            p.setPen(QPen(_C_PANEL, 0.5))
            p.drawPolygon(bq2)

        # Nose ogive cap
        nose_tip = b2w(0, 0, FUSE_NOSE_Z - 0.02)
        nring    = stations[0]
        nose_cap = QPolygonF([QPointF(*wp(*b2w(*v))) for v in nring])
        p.setBrush(QBrush(_C_FUSE_DARK))
        p.setPen(QPen(_C_FUSE_LIGHT, 1))
        p.drawPolygon(nose_cap)

        # Tail-boom cap
        tail_ring = stations[-1]
        tail_cap  = QPolygonF([QPointF(*wp(*b2w(*v))) for v in tail_ring])
        p.setBrush(QBrush(_C_FUSE_DARK))
        p.setPen(QPen(_C_PANEL, 0.5))
        p.drawPolygon(tail_cap)

        # Panel lines — two accent lines along fuselage sides
        p.setPen(QPen(_C_PANEL, 1))
        prev_r = prev_l = None
        for ring in stations[::3]:
            pr = QPointF(*wp(*b2w(*ring[2])))  # right
            pl = QPointF(*wp(*b2w(*ring[6])))  # left
            if prev_r:
                p.drawLine(prev_r, pr)
                p.drawLine(prev_l, pl)
            prev_r, prev_l = pr, pl

    # ------------------------------------------------------------------ #
    # Main wings                                                           #
    # ------------------------------------------------------------------ #

    @staticmethod
    def _wing_quad(b2w, wp,
                   x_root: float, x_tip: float,
                   z_le_root: float, z_te_root: float,
                   z_le_tip: float, z_te_tip: float,
                   y_root: float, y_tip: float,
                   top_col: QColor, bot_col: QColor,
                   p: QPainter) -> None:
        """Draw one wing panel as a top-surface and bottom-surface quad."""
        # Top surface
        top = QPolygonF([
            QPointF(*wp(*b2w(x_root, y_root + WING_THICK * 0.5, z_le_root))),
            QPointF(*wp(*b2w(x_root, y_root + WING_THICK * 0.5, z_te_root))),
            QPointF(*wp(*b2w(x_tip,  y_tip  + WING_THICK * 0.3, z_te_tip))),
            QPointF(*wp(*b2w(x_tip,  y_tip  + WING_THICK * 0.3, z_le_tip))),
        ])
        p.setBrush(QBrush(top_col))
        p.setPen(QPen(_C_PANEL, 0.5))
        p.drawPolygon(top)

        # Bottom surface
        bot = QPolygonF([
            QPointF(*wp(*b2w(x_root, y_root - WING_THICK * 0.5, z_le_root))),
            QPointF(*wp(*b2w(x_root, y_root - WING_THICK * 0.5, z_te_root))),
            QPointF(*wp(*b2w(x_tip,  y_tip  - WING_THICK * 0.3, z_te_tip))),
            QPointF(*wp(*b2w(x_tip,  y_tip  - WING_THICK * 0.3, z_le_tip))),
        ])
        p.setBrush(QBrush(bot_col))
        p.setPen(QPen(_C_PANEL, 0.5))
        p.drawPolygon(bot)

        # Leading edge dark strip
        le = QPolygonF([
            QPointF(*wp(*b2w(x_root, y_root + WING_THICK * 0.5, z_le_root))),
            QPointF(*wp(*b2w(x_root, y_root - WING_THICK * 0.5, z_le_root))),
            QPointF(*wp(*b2w(x_tip,  y_tip  - WING_THICK * 0.3, z_le_tip))),
            QPointF(*wp(*b2w(x_tip,  y_tip  + WING_THICK * 0.3, z_le_tip))),
        ])
        p.setBrush(QBrush(_C_WING_LE))
        p.setPen(QPen(_C_PANEL, 0.5))
        p.drawPolygon(le)

    @classmethod
    def draw_wings(cls, p: QPainter, b2w, wp,
                   aileron_deflection: float = 0.0) -> None:
        """Draw main wings with ailerons.

        aileron_deflection: degrees, positive = left aileron up / right down.
        """
        for side in (+1, -1):   # +1 = right, -1 = left
            xs = side * WING_SPAN_HALF
            yt = WING_ROOT_Y + WING_TIP_DY

            # Inboard panel (root → aileron inboard edge)
            x_aile_in  = side * WING_SPAN_HALF * AILE_SPAN_IN
            z_le_ai_in = WING_ROOT_Z_LE + (WING_TIP_Z_LE - WING_ROOT_Z_LE) * AILE_SPAN_IN
            z_te_ai_in = WING_ROOT_Z_TE + (WING_TIP_Z_TE - WING_ROOT_Z_TE) * AILE_SPAN_IN
            y_ai_in    = WING_ROOT_Y + (yt - WING_ROOT_Y) * AILE_SPAN_IN

            cls._wing_quad(b2w, wp,
                           x_root=0,       x_tip=x_aile_in,
                           z_le_root=WING_ROOT_Z_LE, z_te_root=WING_ROOT_Z_TE,
                           z_le_tip=z_le_ai_in,      z_te_tip=z_te_ai_in,
                           y_root=WING_ROOT_Y, y_tip=y_ai_in,
                           top_col=_C_WING_TOP, bot_col=_C_WING_BOTTOM, p=p)

            # Outboard panel (aileron inboard → tip, without aileron zone)
            x_aile_out  = side * WING_SPAN_HALF * AILE_SPAN_OUT
            z_le_ao_out = WING_ROOT_Z_LE + (WING_TIP_Z_LE - WING_ROOT_Z_LE) * AILE_SPAN_OUT
            z_te_ao_out = WING_ROOT_Z_TE + (WING_TIP_Z_TE - WING_ROOT_Z_TE) * AILE_SPAN_OUT
            y_ao_out    = WING_ROOT_Y + (yt - WING_ROOT_Y) * AILE_SPAN_OUT

            # Chord split: fixed leading portion + aileron trailing portion
            chord_in  = z_te_ai_in  - z_le_ai_in
            chord_out = z_te_ao_out - z_le_ao_out
            split_in  = z_le_ai_in  + chord_in  * (1.0 - AILE_CHORD)
            split_out = z_le_ao_out + chord_out * (1.0 - AILE_CHORD)

            # Fixed forward portion between aileron in/out
            cls._wing_quad(b2w, wp,
                           x_root=x_aile_in,  x_tip=x_aile_out,
                           z_le_root=z_le_ai_in,  z_te_root=split_in,
                           z_le_tip=z_le_ao_out,  z_te_tip=split_out,
                           y_root=y_ai_in, y_tip=y_ao_out,
                           top_col=_C_WING_TOP, bot_col=_C_WING_BOTTOM, p=p)

            # Outboard tip panel (beyond aileron)
            cls._wing_quad(b2w, wp,
                           x_root=x_aile_out, x_tip=xs,
                           z_le_root=z_le_ao_out, z_te_root=z_te_ao_out,
                           z_le_tip=WING_TIP_Z_LE, z_te_tip=WING_TIP_Z_TE,
                           y_root=y_ao_out, y_tip=yt,
                           top_col=_C_WING_TIP, bot_col=_C_WING_BOTTOM, p=p)

            # ── Aileron panel (deflected) ─────────────────────────────
            # Left aileron deflects opposite to right (differential)
            sign    = +1 if side > 0 else -1
            defl_r  = math.radians(sign * aileron_deflection)  # trailing edge up = negative Y
            # Aileron hinge is at split_in / split_out line at y_ai_in / y_ao_out
            # Deflection rotates trailing edge around hinge line

            def _aile_pt(bx, by_base, bz_hinge, bz_te, hinge_y, defl):
                """Compute deflected aileron trailing edge point."""
                chord_len  = bz_te - bz_hinge
                defl_dy    = -math.sin(defl) * chord_len
                defl_dz    =  math.cos(defl) * chord_len - chord_len
                return b2w(bx, hinge_y + defl_dy, bz_hinge + chord_len + defl_dz)

            cs_col = _C_CS_ACTIVE if abs(aileron_deflection) > 1.0 else _C_CS_NEUTRAL

            aile_q = QPolygonF([
                QPointF(*wp(*b2w(x_aile_in,  y_ai_in,  split_in))),
                QPointF(*wp(*_aile_pt(x_aile_in,  0, split_in,  z_te_ai_in,  y_ai_in,  defl_r))),
                QPointF(*wp(*_aile_pt(x_aile_out, 0, split_out, z_te_ao_out, y_ao_out, defl_r))),
                QPointF(*wp(*b2w(x_aile_out, y_ao_out, split_out))),
            ])
            p.setBrush(QBrush(cs_col))
            p.setPen(QPen(_C_CS_EDGE, 1))
            p.drawPolygon(aile_q)

    # ------------------------------------------------------------------ #
    # Horizontal stabiliser + elevator                                     #
    # ------------------------------------------------------------------ #

    @classmethod
    def draw_hstab(cls, p: QPainter, b2w, wp,
                   elevator_deflection: float = 0.0) -> None:
        for side in (+1, -1):
            xs = side * HSTAB_SPAN_HALF
            yt = HSTAB_Y
            chord_r = HSTAB_ROOT_Z_TE - HSTAB_ROOT_Z_LE
            chord_t = HSTAB_TIP_Z_TE  - HSTAB_TIP_Z_LE
            split_r = HSTAB_ROOT_Z_LE + chord_r * (1.0 - ELEV_CHORD)
            split_t = HSTAB_TIP_Z_LE  + chord_t * (1.0 - ELEV_CHORD)

            # Fixed forward portion
            cls._wing_quad(b2w, wp,
                           x_root=0,   x_tip=xs,
                           z_le_root=HSTAB_ROOT_Z_LE, z_te_root=split_r,
                           z_le_tip=HSTAB_TIP_Z_LE,  z_te_tip=split_t,
                           y_root=yt, y_tip=yt,
                           top_col=_C_WING_TOP, bot_col=_C_WING_BOTTOM, p=p)

            # Elevator (trailing portion — deflects with pitch)
            defl_r = math.radians(elevator_deflection)

            def _elev_pt(bx, bz_hinge, bz_te, hinge_y, defl):
                chord_len = bz_te - bz_hinge
                defl_dy   = -math.sin(defl) * chord_len
                defl_dz   =  math.cos(defl) * chord_len - chord_len
                return b2w(bx, hinge_y + defl_dy, bz_hinge + chord_len + defl_dz)

            cs_col = _C_CS_ACTIVE if abs(elevator_deflection) > 1.0 else _C_CS_NEUTRAL
            elev_q = QPolygonF([
                QPointF(*wp(*b2w(0,  yt, split_r))),
                QPointF(*wp(*_elev_pt(0,  split_r, HSTAB_ROOT_Z_TE, yt, defl_r))),
                QPointF(*wp(*_elev_pt(xs, split_t, HSTAB_TIP_Z_TE,  yt, defl_r))),
                QPointF(*wp(*b2w(xs, yt, split_t))),
            ])
            p.setBrush(QBrush(cs_col))
            p.setPen(QPen(_C_CS_EDGE, 1))
            p.drawPolygon(elev_q)

    # ------------------------------------------------------------------ #
    # Vertical stabiliser + rudder                                         #
    # ------------------------------------------------------------------ #

    @classmethod
    def draw_vstab(cls, p: QPainter, b2w, wp,
                   rudder_deflection: float = 0.0) -> None:
        """Draw dorsal fin with animated rudder."""
        root_le = (0, VSTAB_BASE_Y,      VSTAB_BASE_Z)
        root_te = (0, VSTAB_BASE_Y,      VSTAB_BASE_Z + 0.28)
        tip_le  = (0, VSTAB_TIP_Y,       VSTAB_TIP_Z + VSTAB_LE_OFFSET)
        tip_te  = (0, VSTAB_TIP_Y,       VSTAB_TIP_Z)

        # Chord split for rudder
        root_chord = root_te[2] - root_le[2]
        tip_chord  = tip_te[2]  - tip_le[2]
        split_r_z  = root_le[2] + root_chord * (1.0 - RUDDER_CHORD)
        split_t_z  = tip_le[2]  + tip_chord  * (1.0 - RUDDER_CHORD)

        # Fixed forward portion — right face
        fin_fwd = QPolygonF([
            QPointF(*wp(*b2w(*root_le))),
            QPointF(*wp(*b2w(0, VSTAB_BASE_Y, split_r_z))),
            QPointF(*wp(*b2w(0, VSTAB_TIP_Y,  split_t_z))),
            QPointF(*wp(*b2w(*tip_le))),
        ])
        p.setBrush(QBrush(_C_WING_TOP))
        p.setPen(QPen(_C_PANEL, 0.5))
        p.drawPolygon(fin_fwd)

        # Rudder (deflected in X direction)
        defl_r = math.radians(rudder_deflection)

        def _rud_pt(by, bz_hinge, bz_te):
            chord_len = bz_te - bz_hinge
            defl_dx   =  math.sin(defl_r) * chord_len
            defl_dz   =  math.cos(defl_r) * chord_len - chord_len
            return b2w(defl_dx, by, bz_hinge + chord_len + defl_dz)

        cs_col = _C_CS_ACTIVE if abs(rudder_deflection) > 1.0 else _C_CS_NEUTRAL
        rud_q = QPolygonF([
            QPointF(*wp(*b2w(0, VSTAB_BASE_Y, split_r_z))),
            QPointF(*wp(*_rud_pt(VSTAB_BASE_Y, split_r_z, root_te[2]))),
            QPointF(*wp(*_rud_pt(VSTAB_TIP_Y,  split_t_z, tip_te[2]))),
            QPointF(*wp(*b2w(0, VSTAB_TIP_Y,  split_t_z))),
        ])
        p.setBrush(QBrush(cs_col))
        p.setPen(QPen(_C_CS_EDGE, 1))
        p.drawPolygon(rud_q)

        # Fin leading edge dark strip
        fin_le = QPolygonF([
            QPointF(*wp(*b2w(-0.008, VSTAB_BASE_Y, VSTAB_BASE_Z))),
            QPointF(*wp(*b2w( 0.008, VSTAB_BASE_Y, VSTAB_BASE_Z))),
            QPointF(*wp(*b2w( 0.008, VSTAB_TIP_Y,  VSTAB_TIP_Z + VSTAB_LE_OFFSET))),
            QPointF(*wp(*b2w(-0.008, VSTAB_TIP_Y,  VSTAB_TIP_Z + VSTAB_LE_OFFSET))),
        ])
        p.setBrush(QBrush(_C_WING_LE))
        p.setPen(_PN)
        p.drawPolygon(fin_le)

    # ------------------------------------------------------------------ #
    # Sensor / camera pod                                                  #
    # ------------------------------------------------------------------ #

    @staticmethod
    def draw_sensor_pod(p: QPainter, b2w, wp) -> None:
        """Draw streamlined electro-optical sensor pod under nose."""
        cx_b, cy_b, cz_b = 0.0, SENSOR_Y, SENSOR_Z
        R  = SENSOR_RADIUS
        RL = R * 0.65   # lens aperture

        # Streamlined body (oval projected)
        body_pts = []
        for deg in range(0, 360, 20):
            rad = math.radians(deg)
            bx  = math.cos(rad) * R * 0.7
            by  = cy_b + math.sin(rad) * R
            bz  = cz_b
            body_pts.append(QPointF(*wp(*b2w(bx, by, bz))))
        p.setBrush(QBrush(_C_SENSOR_BODY))
        p.setPen(QPen(QColor(40, 50, 44), 1))
        p.drawPolygon(QPolygonF(body_pts))

        # Forward dome (camera aperture)
        lens_pts = []
        for deg in range(0, 360, 18):
            rad = math.radians(deg)
            bx  = math.cos(rad) * RL * 0.8
            by  = cy_b + math.sin(rad) * RL
            bz  = cz_b - R * 0.55
            lens_pts.append(QPointF(*wp(*b2w(bx, by, bz))))
        p.setBrush(QBrush(_C_SENSOR_LENS))
        p.setPen(QPen(QColor(60, 100, 180, 180), 1))
        p.drawPolygon(QPolygonF(lens_pts))

        # Lens reflection highlight
        refl_pts = []
        for deg in range(300, 420, 20):
            rad = math.radians(deg)
            bx  = math.cos(rad) * RL * 0.35
            by  = cy_b + math.sin(rad) * RL * 0.35 + RL * 0.2
            bz  = cz_b - R * 0.58
            refl_pts.append(QPointF(*wp(*b2w(bx, by, bz))))
        p.setBrush(QBrush(_C_LENS_REFL))
        p.setPen(_PN)
        if len(refl_pts) >= 3:
            p.drawPolygon(QPolygonF(refl_pts))

    # ------------------------------------------------------------------ #
    # Propeller (nose tractor)                                             #
    # ------------------------------------------------------------------ #

    @staticmethod
    def draw_propeller(p: QPainter, b2w, wp,
                       prop_angle: float = 0.0,
                       rotor_speed: float = 0.0) -> None:
        """Draw nose tractor propeller — disc blur at speed, blades when slow."""
        pz = PROP_Z
        # Spinner hub
        hub_pts = []
        for deg in range(0, 360, 24):
            rad = math.radians(deg)
            bx  = math.cos(rad) * PROP_SPINNER_R
            by  = math.sin(rad) * PROP_SPINNER_R
            hub_pts.append(QPointF(*wp(*b2w(bx, by, pz))))
        p.setBrush(QBrush(_C_PROP_HUB))
        p.setPen(QPen(QColor(50, 60, 52), 1))
        p.drawPolygon(QPolygonF(hub_pts))

        if rotor_speed > 0.05:
            # Blur disc — fades in with speed
            disc_alpha = int(min(1.0, rotor_speed * 1.4) * 120)
            disc_pts = []
            for deg in range(0, 361, 12):
                rad = math.radians(deg)
                bx  = math.cos(rad) * PROP_RADIUS
                by  = math.sin(rad) * PROP_RADIUS
                disc_pts.append(QPointF(*wp(*b2w(bx, by, pz))))
            # Outer glow ring
            glow_col = QColor(60, 80, 64, disc_alpha // 2)
            p.setBrush(QBrush(glow_col))
            p.setPen(QPen(QColor(30, 45, 32, disc_alpha), 1))
            p.drawPolygon(QPolygonF(disc_pts))

            # Inner disc
            inner_pts = []
            for deg in range(0, 361, 12):
                rad = math.radians(deg)
                bx  = math.cos(rad) * PROP_SPINNER_R * 2.2
                by  = math.sin(rad) * PROP_SPINNER_R * 2.2
                inner_pts.append(QPointF(*wp(*b2w(bx, by, pz))))
            p.setBrush(QBrush(QColor(50, 65, 52, disc_alpha)))
            p.setPen(_PN)
            p.drawPolygon(QPolygonF(inner_pts))

        # Blade lines — visible when slow
        blade_alpha = int(max(0.0, 1.0 - rotor_speed * 2.5) * 220)
        if blade_alpha > 10:
            bp = QPen(QColor(_C_PROP_BLADE.red(), _C_PROP_BLADE.green(),
                              _C_PROP_BLADE.blue(), blade_alpha), 3,
                      Qt.PenStyle.SolidLine, Qt.PenCapStyle.RoundCap)
            p.setPen(bp)
            cpt = QPointF(*wp(*b2w(0, 0, pz)))
            for blade in range(PROP_BLADES):
                ba_r = math.radians(prop_angle + blade * (360 / PROP_BLADES))
                tip  = QPointF(*wp(*b2w(math.cos(ba_r) * PROP_RADIUS,
                                        math.sin(ba_r) * PROP_RADIUS,
                                        pz)))
                p.drawLine(cpt, tip)

    # ------------------------------------------------------------------ #
    # Landing gear                                                         #
    # ------------------------------------------------------------------ #

    @staticmethod
    def draw_landing_gear(p: QPainter, b2w, wp,
                          altitude: float = 0.0) -> None:
        """Draw tricycle landing gear — retract above 1.5 m AGL."""
        if altitude > 1.5:
            return
        alpha = int(255 * max(0.0, 1.0 - altitude / 1.5))
        gear_pen   = QPen(QColor(28, 34, 30, alpha), 2)
        wheel_brush = QBrush(QColor(20, 24, 22, alpha))

        gear_positions = [
            # (strut_top_bx, strut_top_by, strut_top_bz,
            #  wheel_bx,    wheel_by,    wheel_bz,    wheel_rx, wheel_ry)
            (0, -FUSE_H_MAX * 0.6, NOSE_GEAR_Z,
             0,  NOSE_GEAR_Y,     NOSE_GEAR_Z,  0.04, 0.025),
            (-MAIN_GEAR_X, -FUSE_H_MAX * 0.4, MAIN_GEAR_Z,
             -MAIN_GEAR_X,  MAIN_GEAR_Y,       MAIN_GEAR_Z,  0.055, 0.032),
            ( MAIN_GEAR_X, -FUSE_H_MAX * 0.4, MAIN_GEAR_Z,
              MAIN_GEAR_X,  MAIN_GEAR_Y,       MAIN_GEAR_Z,  0.055, 0.032),
        ]
        for (tx, ty, tz, wx, wy, wz, wr, wry) in gear_positions:
            top_pt   = QPointF(*wp(*b2w(tx, ty, tz)))
            wheel_pt = QPointF(*wp(*b2w(wx, wy, wz)))

            # Strut
            p.setPen(gear_pen)
            p.drawLine(top_pt, wheel_pt)

            # Wheel
            whl_pts = []
            for deg in range(0, 361, 30):
                rad = math.radians(deg)
                bx  = wx + math.cos(rad) * wr
                by  = wy + math.sin(rad) * wry
                whl_pts.append(QPointF(*wp(*b2w(bx, by, wz))))
            p.setBrush(wheel_brush)
            p.setPen(QPen(QColor(28, 34, 30, alpha), 1))
            p.drawPolygon(QPolygonF(whl_pts))

    # ------------------------------------------------------------------ #
    # Navigation lights                                                    #
    # ------------------------------------------------------------------ #

    @staticmethod
    def draw_nav_lights(p: QPainter, b2w, wp,
                        armed: bool, rotor_speed: float,
                        frame: int) -> None:
        """Aviation standard navigation lights + tail strobe."""
        if not armed:
            return

        la = min(255, int(rotor_speed * 300))

        # Port (left) wingtip — GREEN
        lx, ly = wp(*b2w(-WING_SPAN_HALF, WING_ROOT_Y, WING_TIP_Z_LE + 0.1))
        col = QColor(_C_NAV_GRN.red(), _C_NAV_GRN.green(), _C_NAV_GRN.blue(), la)
        glow = QRadialGradient(QPointF(lx, ly), 16)
        glow.setColorAt(0.0, QColor(col.red(), col.green(), col.blue(), 60))
        glow.setColorAt(1.0, QColor(col.red(), col.green(), col.blue(), 0))
        p.setBrush(QBrush(glow)); p.setPen(_PN)
        p.drawEllipse(QPointF(lx, ly), 16, 10)
        p.setBrush(QBrush(col)); p.setPen(_PN)
        p.drawEllipse(QPointF(lx, ly), 4, 3)

        # Starboard (right) wingtip — RED
        rx, ry = wp(*b2w( WING_SPAN_HALF, WING_ROOT_Y, WING_TIP_Z_LE + 0.1))
        col = QColor(_C_NAV_RED.red(), _C_NAV_RED.green(), _C_NAV_RED.blue(), la)
        glow = QRadialGradient(QPointF(rx, ry), 16)
        glow.setColorAt(0.0, QColor(col.red(), col.green(), col.blue(), 60))
        glow.setColorAt(1.0, QColor(col.red(), col.green(), col.blue(), 0))
        p.setBrush(QBrush(glow)); p.setPen(_PN)
        p.drawEllipse(QPointF(rx, ry), 16, 10)
        p.setBrush(QBrush(col)); p.setPen(_PN)
        p.drawEllipse(QPointF(rx, ry), 4, 3)

        # Tail strobe — WHITE  (blinks at 30-frame cycle)
        strobe_on = (frame % 30) < 8
        alpha_s   = la if strobe_on else la // 5
        tx_b, ty_b = wp(*b2w(0, 0, FUSE_TAIL_Z))
        col_s = QColor(_C_STROBE.red(), _C_STROBE.green(), _C_STROBE.blue(), alpha_s)
        glow_s = QRadialGradient(QPointF(tx_b, ty_b), 12)
        glow_s.setColorAt(0.0, QColor(col_s.red(), col_s.green(), col_s.blue(), 50))
        glow_s.setColorAt(1.0, QColor(col_s.red(), col_s.green(), col_s.blue(), 0))
        p.setBrush(QBrush(glow_s)); p.setPen(_PN)
        p.drawEllipse(QPointF(tx_b, ty_b), 12, 8)
        p.setBrush(QBrush(col_s)); p.setPen(_PN)
        p.drawEllipse(QPointF(tx_b, ty_b), 3, 2)

    # ------------------------------------------------------------------ #
    # Engine exhaust (cosmetic detail)                                     #
    # ------------------------------------------------------------------ #

    @staticmethod
    def draw_engine_detail(p: QPainter, b2w, wp, rotor_speed: float) -> None:
        """Draw engine exhaust ring and heat shimmer at rear belly."""
        for side in (+1, -1):
            ex = side * 0.06
            ey, ez = ENGINE_Y, ENGINE_Z
            pts = []
            for deg in range(0, 361, 30):
                rad = math.radians(deg)
                bx  = ex + math.cos(rad) * ENGINE_RADIUS
                by  = ey + math.sin(rad) * ENGINE_RADIUS * 0.6
                pts.append(QPointF(*wp(*b2w(bx, by, ez))))
            exhaust_alpha = int(min(1.0, rotor_speed) * 120)
            p.setBrush(QBrush(QColor(20, 25, 22, exhaust_alpha + 40)))
            p.setPen(QPen(QColor(50, 55, 50, 120), 1))
            p.drawPolygon(QPolygonF(pts))

            # Heat shimmer glow (orange tinge when speed > 0.3)
            if rotor_speed > 0.3:
                glow_alpha = int((rotor_speed - 0.3) / 0.7 * 60)
                ex_pt = wp(*b2w(ex, ey, ez + ENGINE_RADIUS))
                g = QRadialGradient(QPointF(*ex_pt), 14)
                g.setColorAt(0.0, QColor(255, 140, 40, glow_alpha))
                g.setColorAt(1.0, QColor(255, 100, 20, 0))
                p.setBrush(QBrush(g)); p.setPen(_PN)
                p.drawEllipse(QPointF(*ex_pt), 14, 9)


# ===========================================================================
# Main widget — identical public API to original Drone3DWidget
# ===========================================================================

class Drone3DWidget(QWidget):
    """Military-grade tactical UAV ground-control display.

    Displays a realistic fixed-wing reconnaissance UAV over a tactical
    ground grid. All physics state (position, attitude, control surfaces)
    is driven by DroneState from drone_physics.py.

    Public API (unchanged from original):
        update_state(state: DroneState)
        set_camera_yaw(deg: float)
        orbit(delta: float)
        zoom_in() / zoom_out() / zoom_reset()
        wheelEvent(event)
    """

    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self._state = DroneState()
        self._trail: deque[tuple[float, float, float]] = deque(maxlen=TRAIL_LEN)
        self._cam_yaw = 25.0   # three-quarter front-side view by default
        rad = math.radians(self._cam_yaw)
        self._cam_cos = math.cos(rad)
        self._cam_sin = math.sin(rad)

        # Smooth camera follow point
        self._fx = self._fy = self._fz = 0.0

        # Zoom
        self._zoom: float = 85.0
        self._zoom_min: float = 20.0
        self._zoom_max: float = 220.0

        # Frame counter (nav light strobe)
        self._frame = 0

        # Propeller angle (degrees, updated each frame)
        self._prop_angle: float = 0.0

        # Star field (seeded for reproducibility)
        rng = random.Random(42)
        self._stars = [(rng.random(), rng.random() * 0.48,
                        rng.randint(30, 110)) for _ in range(120)]

        self.setMinimumSize(560, 440)
        self.setStyleSheet("background:#06080C;")
        self.setFocusPolicy(Qt.FocusPolicy.WheelFocus)

    # ------------------------------------------------------------------
    # Public API (preserved exactly)
    # ------------------------------------------------------------------

    def update_state(self, state: DroneState) -> None:
        self._state = state
        self._frame += 1

        # Advance propeller angle from rotor_speed
        # rotor_speed 0→1 maps to 0→720 deg/frame (visible spin)
        self._prop_angle = (self._prop_angle + state.rotor_speed * 18.0) % 360.0

        if state.is_airborne or state.y > 0.05:
            self._trail.append((state.x, state.y, state.z))
        elif state.y < 0.01:
            self._trail.clear()

        # Smooth camera follow
        ah = 0.18
        av = 0.08
        self._fx += (state.x - self._fx) * ah
        self._fz += (state.z - self._fz) * ah
        self._fy += (state.y - self._fy) * av
        self.update()

    def set_camera_yaw(self, deg: float) -> None:
        self._cam_yaw = deg % 360.0
        rad = math.radians(self._cam_yaw)
        self._cam_cos = math.cos(rad)
        self._cam_sin = math.sin(rad)
        self.update()

    def orbit(self, delta: float) -> None:
        self.set_camera_yaw(self._cam_yaw + delta)

    def zoom_in(self) -> None:
        self._zoom = min(self._zoom_max, self._zoom * 1.20)
        self.update()

    def zoom_out(self) -> None:
        self._zoom = max(self._zoom_min, self._zoom / 1.20)
        self.update()

    def zoom_reset(self) -> None:
        self._zoom = SCALE
        self.update()

    def wheelEvent(self, event) -> None:  # noqa: N802
        delta  = event.angleDelta().y()
        factor = 1.12 if event.modifiers() & Qt.KeyboardModifier.ControlModifier else 1.25
        if delta > 0:
            self._zoom = min(self._zoom_max, self._zoom * factor)
        elif delta < 0:
            self._zoom = max(self._zoom_min, self._zoom / factor)
        self.update()
        event.accept()

    # ------------------------------------------------------------------
    # Paint entry point
    # ------------------------------------------------------------------

    def paintEvent(self, _ev) -> None:  # noqa: N802
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        w, h = self.width(), self.height()
        cc, sc = self._cam_cos, self._cam_sin
        z = self._zoom

        DEPTH_X = 0.20
        DEPTH_Y = 0.58
        VERT_OFFSET = 30
        rx_ = self._fx * cc + self._fz * sc
        rz_ = -self._fx * sc + self._fz * cc
        cx  = w * 0.5 - z * (rx_ - rz_ * DEPTH_X)
        cy  = h * 0.5 + VERT_OFFSET + self._fy * z - rz_ * z * DEPTH_Y

        self._draw_environment(p, w, h, cx, cy, cc, sc, z)
        self._draw_trail(p, cx, cy, cc, sc, z)
        self._draw_shadow(p, cx, cy, cc, sc, z)
        self._draw_drone(p, cx, cy, cc, sc, z)
        self._draw_velocity_vector(p, cx, cy, cc, sc, z)
        self._draw_hud(p, w, h)
        p.end()

    # ==================================================================
    # ENVIRONMENT  (unchanged from original)
    # ==================================================================

    def _draw_environment(self, p, w, h, cx, cy, cc, sc, z=90.0) -> None:
        self._draw_sky(p, w, h)
        self._draw_terrain(p, w, h, cx, cy, cc, sc, z)
        self._draw_range_rings(p, cx, cy, cc, sc, z)
        self._draw_grid(p, cx, cy, cc, sc, z)
        self._draw_origin_marker(p, cx, cy, cc, sc, z)
        self._draw_compass_ground(p, cx, cy, cc, sc, z)

    def _draw_sky(self, p, w, h) -> None:
        g = QLinearGradient(0, 0, 0, h * 0.72)
        g.setColorAt(0.0, _C_BG)
        g.setColorAt(0.5, _C_SKY_MID)
        g.setColorAt(1.0, _C_SKY_HRZ)
        p.fillRect(0, 0, w, int(h * 0.72), QBrush(g))

        hy = int(h * 0.65)
        hg = QLinearGradient(0, hy - 8, 0, hy + 8)
        hg.setColorAt(0.0, QColor(0, 0, 0, 0))
        hg.setColorAt(0.5, QColor(40, 90, 40, 60))
        hg.setColorAt(1.0, QColor(0, 0, 0, 0))
        p.fillRect(0, hy - 8, w, 16, QBrush(hg))

        for fx, fy, alpha in self._stars:
            p.setPen(QPen(QColor(200, 230, 200, alpha), 1))
            p.drawPoint(int(fx * w), int(fy * h))

        p.fillRect(0, int(h * 0.65), w, h, QBrush(_C_TERRAIN))

    def _draw_terrain(self, p, w, h, cx, cy, cc, sc, z=90.0) -> None:
        s = self._state
        tex_pen = QPen(QColor(25, 38, 20, 40), 1)
        p.setPen(tex_pen)
        ox = round(s.x / 10) * 10
        oz = round(s.z / 10) * 10
        for i in range(-8, 9):
            wx = ox + i * 10
            p.drawLine(
                _qpt(wx, 0, oz - 80, cx, cy, cc, sc, z),
                _qpt(wx + 40, 0, oz + 40, cx, cy, cc, sc, z))

    def _draw_range_rings(self, p, cx, cy, cc, sc, z=90.0) -> None:
        s = self._state
        for ring_r, label in ((25, "25"), (50, "50"), (100, "100")):
            alpha = max(20, 80 - int(s.altitude * 2))
            pen = QPen(QColor(40, 100, 40, alpha), 1, Qt.PenStyle.DotLine)
            p.setPen(pen)
            pts = []
            for deg in range(0, 361, 6):
                rad = math.radians(deg)
                wx = math.sin(rad) * ring_r
                wz = -math.cos(rad) * ring_r
                pts.append(_qpt(wx, 0, wz, cx, cy, cc, sc, z))
            for i in range(len(pts) - 1):
                p.drawLine(pts[i], pts[i + 1])
            lx, ly = _proj(ring_r, 0, 0, cx, cy, cc, sc, z)
            p.setFont(QFont("Consolas", 7))
            p.setPen(QPen(QColor(40, 120, 40, alpha + 40)))
            p.drawText(int(lx) + 2, int(ly) - 2, 28, 12,
                       Qt.AlignmentFlag.AlignLeft, f"{label}m")

    def _draw_grid(self, p, cx, cy, cc, sc, z=90.0) -> None:
        s   = self._state
        GAP = 5.0
        N   = 20
        ox  = round(s.x / GAP) * GAP
        oz  = round(s.z / GAP) * GAP

        for i in range(-N, N + 1):
            is_major = (i % 5 == 0)
            is_axis  = (i == 0)
            if is_axis:
                pen = QPen(QColor(50, 130, 50, 160), 1)
            elif is_major:
                pen = QPen(QColor(35, 80, 35, 100), 1)
            else:
                pen = QPen(QColor(22, 45, 22, 55), 1)
            p.setPen(pen)

            wz = oz + i * GAP
            p.drawLine(_qpt(ox - N * GAP, 0, wz, cx, cy, cc, sc, z),
                       _qpt(ox + N * GAP, 0, wz, cx, cy, cc, sc, z))
            wx = ox + i * GAP
            p.drawLine(_qpt(wx, 0, oz - N * GAP, cx, cy, cc, sc, z),
                       _qpt(wx, 0, oz + N * GAP, cx, cy, cc, sc, z))

        p.setFont(QFont("Consolas", 7))
        for d in (-75, -50, -25, 0, 25, 50, 75):
            wx = round((s.x + d) / 25) * 25
            sx, sy = _proj(wx, 0, oz, cx, cy, cc, sc, z)
            p.setPen(QPen(QColor(50, 120, 50, 100)))
            p.drawText(int(sx) - 14, int(sy) + 2, 28, 11,
                       Qt.AlignmentFlag.AlignCenter, f"{int(wx)}")

    def _draw_origin_marker(self, p, cx, cy, cc, sc, z=90.0) -> None:
        L = 3.0
        pen = QPen(_C_AMBER, 2)
        p.setPen(pen)
        p.drawLine(_qpt(-L, 0, 0, cx, cy, cc, sc, z),
                   _qpt( L, 0, 0, cx, cy, cc, sc, z))
        p.drawLine(_qpt(0, 0, -L, cx, cy, cc, sc, z),
                   _qpt(0, 0,  L, cx, cy, cc, sc, z))
        ox, oy = _proj(0, 0, 0, cx, cy, cc, sc, z)
        p.setBrush(QBrush(_C_AMBER))
        p.setPen(_PN)
        p.drawEllipse(QPointF(ox, oy), 4, 3)

    def _draw_compass_ground(self, p, cx, cy, cc, sc, z=90.0) -> None:
        s = self._state
        R = 12.0

        dirs = [
            ("N",  0, -R, _C_GREEN),
            ("S",  0,  R, _C_RED),
            ("E",  R,  0, _C_AMBER),
            ("W", -R,  0, _C_BLUE),
        ]
        p.setFont(QFont("Consolas", 10, QFont.Weight.Bold))
        for lbl, dx, dz, col in dirs:
            sx, sy = _proj(s.x + dx, 0, s.z + dz, cx, cy, cc, sc, z)
            p.setBrush(QBrush(QColor(col.red(), col.green(), col.blue(), 170)))
            p.setPen(_PN)
            p.drawEllipse(QPointF(sx, sy), 16, 10)
            p.setPen(QPen(QColor(0, 0, 0, 200)))
            p.drawText(int(sx) - 16, int(sy) - 10, 32, 20,
                       Qt.AlignmentFlag.AlignCenter, lbl)

        yr = math.radians(s.yaw)
        nx = s.x + math.sin(yr) * 6.0
        nz = s.z - math.cos(yr) * 6.0
        p.setPen(QPen(QColor(255, 176, 0, 200), 2))
        p.drawLine(_qpt(s.x, 0, s.z, cx, cy, cc, sc, z),
                   _qpt(nx,  0, nz,  cx, cy, cc, sc, z))

    # ==================================================================
    # TRAIL  (unchanged)
    # ==================================================================

    def _draw_trail(self, p, cx, cy, cc, sc, z=90.0) -> None:
        trail = list(self._trail)
        n = len(trail)
        if n < 2:
            return
        for i in range(1, n):
            frac  = i / n
            alpha = int(180 * frac)
            r = int(255 * (1 - frac) * 0.7)
            g = int(180 * frac + 60)
            p.setPen(QPen(QColor(r, g, 40, alpha), 2))
            ax, ay = _proj(*trail[i - 1], cx, cy, cc, sc, z)
            bx, by = _proj(*trail[i],     cx, cy, cc, sc, z)
            p.drawLine(QPointF(ax, ay), QPointF(bx, by))

    # ==================================================================
    # SHADOW  (unchanged)
    # ==================================================================

    def _draw_shadow(self, p, cx, cy, cc, sc, z=90.0) -> None:
        s = self._state
        if s.y < 0.1:
            return
        alpha = max(0, int(120 - s.y * 5))
        if alpha <= 0:
            return
        sx, sy = _proj(s.x, 0, s.z, cx, cy, cc, sc, z)
        rx = max(8, int(70 - s.y * 2))
        ry = int(rx * 0.30)
        sg = QRadialGradient(QPointF(sx, sy), rx)
        sg.setColorAt(0.0, QColor(0, 0, 0, alpha))
        sg.setColorAt(1.0, QColor(0, 0, 0, 0))
        p.setBrush(QBrush(sg))
        p.setPen(_PN)
        p.drawEllipse(QPointF(sx, sy), rx, ry)

    # ==================================================================
    # DRONE — Fixed-Wing Reconnaissance UAV
    # ==================================================================

    def _draw_drone(self, p, cx, cy, cc, sc, z=90.0) -> None:
        """Draw the complete fixed-wing UAV with animated control surfaces.

        Architecture:
            DroneState.roll        → aileron deflection
            DroneState.pitch       → elevator deflection
            DroneState.yaw rate    → rudder deflection (approx from yaw)
            DroneState.rotor_speed → propeller disc blur + blade visibility
        All geometry lives in body frame; b2w() maps to world frame.
        _proj() / _qpt() project world frame to screen pixels.
        """
        s = self._state

        # ── Body-to-world rotation ────────────────────────────────────
        yr = math.radians(s.yaw)
        pr = math.radians(s.pitch)
        rr = math.radians(s.roll)
        cyr, syr = math.cos(yr), math.sin(yr)
        cpr, spr = math.cos(pr), math.sin(pr)
        crr, srr = math.cos(rr), math.sin(rr)

        def b2w(bx: float, by: float, bz: float) -> tuple[float, float, float]:
            """Rotate body frame (roll→pitch→yaw) and translate to world."""
            # Roll (around body Z axis — but we use X/Y for roll in our convention)
            cx2 = bx * crr - by * srr
            cy2 = bx * srr + by * crr
            # Pitch (around body X)
            dy2 = cy2 * cpr - bz * spr
            dz  = cy2 * spr + bz * cpr
            # Yaw (around world Y)
            ex  = cx2 * cyr - dz * syr
            ez  = cx2 * syr + dz * cyr
            return ex + s.x, dy2 + s.y, ez + s.z

        def wp(wx: float, wy: float, wz: float) -> tuple[float, float]:
            """World → screen pixel."""
            return _proj(wx, wy, wz, cx, cy, cc, sc, z)

        # ── Control surface deflections from physics state ────────────
        # Ailerons: roll angle maps to deflection
        aileron_defl = max(-AILERON_MAX,
                           min(AILERON_MAX, s.roll * (AILERON_MAX / 28.0)))

        # Elevator: pitch angle maps to deflection (nose-up = elevator up)
        elevator_defl = max(-ELEVATOR_MAX,
                            min(ELEVATOR_MAX, -s.pitch * (ELEVATOR_MAX / 28.0)))

        # Rudder: approximate from yaw rate using velocity-derived heading change
        # We use a fraction of the horizontal speed in Z direction as proxy
        yaw_input = getattr(s, '_last_yaw_input', 0.0)
        rudder_defl = max(-RUDDER_MAX,
                          min(RUDDER_MAX, s.roll * (RUDDER_MAX / 28.0) * 0.4))

        # ── Altitude stem (draw below drone) ─────────────────────────
        if s.y > 0.3:
            p.setPen(QPen(QColor(40, 100, 40, 60), 1,
                          Qt.PenStyle.DashLine))
            p.drawLine(_qpt(s.x, 0,   s.z, cx, cy, cc, sc, z),
                       _qpt(s.x, s.y, s.z, cx, cy, cc, sc, z))
            for alt_m in range(5, int(s.y) + 1, 5):
                tx, ty = _proj(s.x, alt_m, s.z, cx, cy, cc, sc, z)
                p.setPen(QPen(QColor(40, 130, 40, 100), 1))
                p.drawLine(QPointF(tx - 6, ty), QPointF(tx + 6, ty))

        # ── Landing gear (drawn first, below fuselage) ────────────────
        _FixedWingUAV.draw_landing_gear(p, b2w, wp, altitude=s.y)

        # ── Engine exhaust detail ─────────────────────────────────────
        _FixedWingUAV.draw_engine_detail(p, b2w, wp, s.rotor_speed)

        # ── Propeller (behind wings in painter's order for top-down) ──
        _FixedWingUAV.draw_propeller(p, b2w, wp,
                                      prop_angle=self._prop_angle,
                                      rotor_speed=s.rotor_speed)

        # ── Wings (drawn before fuselage for correct layering) ────────
        _FixedWingUAV.draw_wings(p, b2w, wp,
                                  aileron_deflection=aileron_defl)

        # ── Horizontal stabiliser + elevator ─────────────────────────
        _FixedWingUAV.draw_hstab(p, b2w, wp,
                                  elevator_deflection=elevator_defl)

        # ── Vertical stabiliser + rudder ──────────────────────────────
        _FixedWingUAV.draw_vstab(p, b2w, wp,
                                  rudder_deflection=rudder_defl)

        # ── Fuselage (drawn on top of wings for correct overlap) ──────
        _FixedWingUAV.draw_fuselage(p, b2w, wp)

        # ── Sensor pod ────────────────────────────────────────────────
        _FixedWingUAV.draw_sensor_pod(p, b2w, wp)

        # ── Navigation lights ─────────────────────────────────────────
        _FixedWingUAV.draw_nav_lights(
            p, b2w, wp,
            armed=(s.mode != FlightMode.DISARMED),
            rotor_speed=s.rotor_speed,
            frame=self._frame,
        )

        # ── FWD nose marker (tactical, same as original) ──────────────
        yr_use = math.radians(s.yaw)
        nd  = 1.4   # forward of nose
        nwx = s.x + math.sin(yr_use) * nd
        nwy = s.y
        nwz = s.z  - math.cos(yr_use) * nd
        np_ = QPointF(*wp(nwx, nwy, nwz))
        p.setBrush(QBrush(_C_AMBER))
        p.setPen(QPen(QColor(0, 0, 0, 180), 1))
        p.drawEllipse(np_, 5, 4)
        nsx, nsy = _proj(nwx, nwy, nwz, cx, cy, cc, sc, z)
        p.setPen(QPen(_C_AMBER))
        p.setFont(QFont("Consolas", 8, QFont.Weight.Bold))
        p.drawText(int(nsx) - 14, int(nsy) - 16, 28, 12,
                   Qt.AlignmentFlag.AlignCenter, "FWD")

        # ── Engine wash (turbulence glow when flying) ─────────────────
        if s.rotor_speed > 0.2 and s.y > 0.5:
            cx_s, cy_s = _proj(s.x, s.y, s.z, cx, cy, cc, sc, z)
            cpt = QPointF(cx_s, cy_s)
            for scale_r, alpha_r in ((80, 20), (50, 35)):
                ga = int(s.rotor_speed * alpha_r)
                glow = QRadialGradient(cpt, scale_r)
                glow.setColorAt(0.0, QColor(40, 100, 40, ga))
                glow.setColorAt(0.7, QColor(20, 60, 20, ga // 3))
                glow.setColorAt(1.0, QColor(0, 0, 0, 0))
                p.setBrush(QBrush(glow))
                p.setPen(_PN)
                p.drawEllipse(cpt, scale_r, int(scale_r * 0.30))

    # ==================================================================
    # VELOCITY VECTOR  (unchanged from original)
    # ==================================================================

    def _draw_velocity_vector(self, p, cx, cy, cc, sc, z=90.0) -> None:
        s   = self._state
        spd = s.speed_h
        if spd < 0.4:
            return
        vlen = math.sqrt(s.vx ** 2 + s.vz ** 2)
        if vlen < 0.01:
            return
        scale = min(5.0, spd / 1.5)
        uvx, uvz = s.vx / vlen, s.vz / vlen
        tx  = s.x + uvx * scale
        tz  = s.z + uvz * scale
        bp  = _qpt(s.x, s.y, s.z, cx, cy, cc, sc, z)
        tp  = _qpt(tx,  s.y, tz,  cx, cy, cc, sc, z)

        for width, alpha in ((7, 40), (4, 80), (2, 200)):
            p.setPen(QPen(QColor(_C_GREEN.red(), _C_GREEN.green(),
                                  _C_GREEN.blue(), alpha), width))
            p.drawLine(bp, tp)

        p.setBrush(QBrush(_C_GREEN))
        _arrow_head(p, tp, bp, head_len=14, head_w=8)

        p.setFont(QFont("Consolas", 9, QFont.Weight.Bold))
        spd_txt = f"{spd:.1f}"
        p.setBrush(QBrush(QColor(0, 20, 0, 180)))
        p.setPen(_PN)
        p.drawRect(int(tp.x()) + 4, int(tp.y()) - 10, 40, 14)
        p.setPen(QPen(_C_GREEN))
        p.drawText(int(tp.x()) + 6, int(tp.y()) - 10, 38, 14,
                   Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter,
                   f"{spd_txt}m/s")

    # ==================================================================
    # HUD — Tactical Ground-Control Station display  (unchanged)
    # ==================================================================

    def _draw_hud(self, p: QPainter, w: int, h: int) -> None:
        s = self._state
        self._hud_scanlines(p, w, h)
        self._hud_mode_badge(p, w, s)
        self._hud_telemetry(p, s)
        self._hud_compass_rose(p, w, s)
        self._hud_adi(p, h, s)
        self._hud_altitude_ladder(p, w, h, s)
        self._hud_minimap(p, w, h)
        self._hud_threat_strip(p, w, h, s)
        self._hud_crosshair(p, w, h)

    # ------------------------------------------------------------------
    # Scanlines
    # ------------------------------------------------------------------

    def _hud_scanlines(self, p: QPainter, w: int, h: int) -> None:
        p.setPen(QPen(QColor(0, 0, 0, 18), 1))
        for y in range(0, h, 3):
            p.drawLine(0, y, w, y)

    # ------------------------------------------------------------------
    # Centre reticle
    # ------------------------------------------------------------------

    def _hud_crosshair(self, p: QPainter, w: int, h: int) -> None:
        cx, cy = w // 2, h // 2
        col = QColor(40, 220, 80, 90)
        p.setPen(QPen(col, 1))
        gap = 12
        arm = 28
        p.drawLine(cx - arm - gap, cy, cx - gap, cy)
        p.drawLine(cx + gap, cy, cx + arm + gap, cy)
        p.drawLine(cx, cy - arm - gap, cx, cy - gap)
        p.drawLine(cx, cy + gap, cx, cy + arm + gap)
        for dx, dy in ((-1, -1), (1, -1), (1, 1), (-1, 1)):
            p.drawLine(cx + dx * gap, cy + dy * gap,
                       cx + dx * (gap + 10), cy + dy * gap)
            p.drawLine(cx + dx * gap, cy + dy * gap,
                       cx + dx * gap, cy + dy * (gap + 10))

    # ------------------------------------------------------------------
    # Mode badge + flight command
    # ------------------------------------------------------------------

    def _hud_mode_badge(self, p: QPainter, w: int, s: DroneState) -> None:
        mode = s.mode.value
        rgb  = _MODE_RGB.get(mode, (60, 70, 60))
        r, g, b = rgb

        f = QFont("Consolas", 12, QFont.Weight.Bold)
        p.setFont(f)
        mode_txt = f"[ {mode} ]"
        fm = p.fontMetrics()
        mw = fm.horizontalAdvance(mode_txt) + 16
        mh = fm.height() + 8
        mx = (w - mw) // 2
        my = 8

        p.setBrush(QBrush(QColor(0, 10, 0, 200)))
        p.setPen(QPen(QColor(r, g, b, 200), 1))
        p.drawRect(mx, my, mw, mh)
        blen = 8
        p.setPen(QPen(QColor(r, g, b), 2))
        for bx, by, sx_, sy_ in (
            (mx,      my,      1,  1),
            (mx + mw, my,     -1,  1),
            (mx,      my + mh, 1, -1),
            (mx + mw, my + mh,-1, -1),
        ):
            p.drawLine(bx, by, bx + sx_ * blen, by)
            p.drawLine(bx, by, bx, by + sy_ * blen)
        p.setPen(QPen(QColor(r, g, b)))
        p.drawText(mx, my, mw, mh, Qt.AlignmentFlag.AlignCenter, mode_txt)

        cmd = s.flight_command
        if cmd and "Disarmed" not in cmd:
            f2 = QFont("Segoe UI", 14, QFont.Weight.Bold)
            p.setFont(f2)
            fm2 = p.fontMetrics()
            cw  = fm2.horizontalAdvance(cmd) + 24
            ch  = fm2.height() + 10
            cbx = (w - cw) // 2
            cby = my + mh + 4

            br, bg_, bb = r, g, b
            cmd_low = cmd.lower()
            if   "forward"  in cmd_low: br, bg_, bb = 40, 220, 80
            elif "back"     in cmd_low: br, bg_, bb = 220, 60, 40
            elif "right"    in cmd_low: br, bg_, bb = 255, 176, 0
            elif "left"     in cmd_low: br, bg_, bb = 60, 160, 240
            elif "climb"    in cmd_low: br, bg_, bb = 40, 220, 80
            elif "descend"  in cmd_low: br, bg_, bb = 220, 80, 40

            p.setBrush(QBrush(QColor(br, bg_, bb, 180)))
            p.setPen(_PN)
            p.drawRoundedRect(cbx, cby, cw, ch, 5, 5)
            p.setPen(QPen(QColor(0, 10, 0)))
            p.drawText(cbx, cby, cw, ch, Qt.AlignmentFlag.AlignCenter, cmd)

    # ------------------------------------------------------------------
    # Telemetry block
    # ------------------------------------------------------------------

    def _hud_telemetry(self, p: QPainter, s: DroneState) -> None:
        p.setBrush(QBrush(QColor(0, 10, 0, 180)))
        p.setPen(QPen(_C_GREEN, 1))
        p.drawRect(6, 6, 160, 158)
        p.setFont(QFont("Consolas", 7, QFont.Weight.Bold))
        p.setPen(QPen(_C_AMBER))
        p.drawText(8, 6, 158, 14, Qt.AlignmentFlag.AlignCenter, "◆ UAV TELEMETRY ◆")
        p.setPen(QPen(QColor(_C_AMBER.red(), _C_AMBER.green(), _C_AMBER.blue(), 100)))
        p.drawLine(8, 20, 164, 20)

        rows = [
            ("ALT",      f"{s.altitude:7.2f} m",    _C_GREEN),
            ("H-SPD",    f"{s.speed_h:6.2f} m/s",  _C_GREEN),
            ("V-SPD",    f"{s.speed_v:+6.2f} m/s", _C_AMBER if abs(s.speed_v) > 0.3 else _C_GREEN),
            ("HDG",      f"{s.heading:6.1f}°",      _C_AMBER),
            ("PITCH",    f"{s.pitch:+6.1f}°",       _C_WHITE),
            ("ROLL",     f"{s.roll:+6.1f}°",        _C_WHITE),
            ("X POS",    f"{s.x:7.1f} m",           _C_BLUE),
            ("Z POS",    f"{s.z:7.1f} m",           _C_BLUE),
            ("DIST",     f"{s.total_distance:6.0f} m", _C_DIM),
            ("FLT TIME", f"{s.flight_time:6.1f} s", _C_DIM),
        ]
        p.setFont(QFont("Consolas", 9))
        y = 24
        for label, value, col in rows:
            p.setPen(QPen(_C_DIM))
            p.drawText(8, y, 58, 14, Qt.AlignmentFlag.AlignRight, label)
            p.setPen(QPen(col))
            p.drawText(70, y, 94, 14, Qt.AlignmentFlag.AlignLeft, value)
            y += 14

    # ------------------------------------------------------------------
    # Compass rose
    # ------------------------------------------------------------------

    def _hud_compass_rose(self, p: QPainter, w: int, s: DroneState) -> None:
        cx, cy, r = w - 58, 60, 46

        p.setBrush(QBrush(QColor(0, 10, 0, 200)))
        p.setPen(QPen(_C_GREEN, 1))
        p.drawEllipse(QPointF(cx, cy), r, r)

        p.setFont(QFont("Consolas", 7, QFont.Weight.Bold))
        for deg in range(0, 360, 10):
            rad_h = math.radians(deg - s.heading)
            sin_h, cos_h = math.sin(rad_h), math.cos(rad_h)
            is_card  = (deg % 90 == 0)
            is_major = (deg % 30 == 0)
            tick_len = 10 if is_card else (6 if is_major else 3)
            inner = r - tick_len
            ox = cx + inner * sin_h
            oy = cy - inner * cos_h
            tx = cx + (r - 2) * sin_h
            ty = cy - (r - 2) * cos_h
            col = _C_AMBER if is_card else (_C_WHITE if is_major else _C_DIM)
            p.setPen(QPen(col, 2 if is_card else 1))
            p.drawLine(QPointF(ox, oy), QPointF(tx, ty))
            if is_major:
                lbl = {0: "N", 90: "E", 180: "S", 270: "W"}.get(deg, str(deg))
                lx = cx + (inner - 10) * sin_h
                ly = cy - (inner - 10) * cos_h
                p.setPen(QPen(_C_GREEN if deg == 0 else col))
                p.drawText(int(lx) - 8, int(ly) - 7, 16, 14,
                           Qt.AlignmentFlag.AlignCenter, lbl)

        p.setPen(QPen(_C_AMBER, 2))
        p.drawLine(QPointF(cx, cy - r + 4), QPointF(cx, cy - 10))
        tri = QPolygonF([
            QPointF(cx, cy - r + 2),
            QPointF(cx - 5, cy - r + 12),
            QPointF(cx + 5, cy - r + 12),
        ])
        p.setBrush(QBrush(_C_AMBER))
        p.setPen(_PN)
        p.drawPolygon(tri)

        p.setBrush(QBrush(QColor(0, 20, 0, 210)))
        p.setPen(QPen(_C_AMBER, 1))
        p.drawRect(int(cx) - 20, int(cy) - 9, 40, 18)
        p.setFont(QFont("Consolas", 10, QFont.Weight.Bold))
        p.setPen(QPen(_C_AMBER))
        p.drawText(int(cx) - 20, int(cy) - 9, 40, 18,
                   Qt.AlignmentFlag.AlignCenter, f"{s.heading:05.1f}")

        p.setBrush(QBrush(_C_GREEN))
        p.setPen(_PN)
        p.drawEllipse(QPointF(cx, cy), 3, 3)

    # ------------------------------------------------------------------
    # ADI
    # ------------------------------------------------------------------

    def _hud_adi(self, p: QPainter, h: int, s: DroneState) -> None:
        cx, cy, r = 58, h - 70, 46

        p.setClipRect(int(cx - r), int(cy - r), r * 2, r * 2)
        p.setBrush(QBrush(QColor(0, 30, 80, 200)))
        p.setPen(_PN)
        p.drawEllipse(QPointF(cx, cy), r, r)

        pitch_off = s.pitch * (r / 55.0)
        rr_rad    = math.radians(-s.roll)
        cos_r     = math.cos(rr_rad)
        sin_r     = math.sin(rr_rad)
        path = QPainterPath()
        hx1 = cx - r * cos_r - pitch_off * sin_r
        hy1 = cy - r * sin_r + pitch_off * cos_r
        hx2 = cx + r * cos_r + pitch_off * sin_r
        hy2 = cy + r * sin_r - pitch_off * cos_r
        path.moveTo(hx1, hy1)
        path.lineTo(hx2, hy2)
        path.lineTo(cx + r, cy + r + 2)
        path.lineTo(cx - r, cy + r + 2)
        path.closeSubpath()
        p.setBrush(QBrush(QColor(60, 40, 10, 200)))
        p.drawPath(path)
        p.setClipping(False)

        p.setFont(QFont("Consolas", 7))
        for pitch_mark in (-20, -10, 0, 10, 20):
            pm_off  = (pitch_mark - s.pitch) * (r / 55.0)
            pm_rotx = -pm_off * sin_r
            pm_roty =  pm_off * cos_r
            px = cx + pm_rotx
            py = cy + pm_roty
            line_w = 20 if pitch_mark == 0 else 12
            col = _C_WHITE if pitch_mark == 0 else _C_DIM
            p.setPen(QPen(col, 1))
            p.drawLine(QPointF(px - line_w, py), QPointF(px + line_w, py))
            if pitch_mark != 0:
                p.setPen(QPen(_C_DIM))
                p.drawText(int(px) + line_w + 2, int(py) - 5, 20, 10,
                           Qt.AlignmentFlag.AlignLeft, str(abs(pitch_mark)))

        p.setBrush(Qt.BrushStyle.NoBrush)
        p.setPen(QPen(_C_GREEN, 2))
        p.drawEllipse(QPointF(cx, cy), r, r)
        p.setPen(QPen(_C_AMBER, 2))
        p.drawLine(int(cx) - 14, int(cy), int(cx) - 5, int(cy))
        p.drawLine(int(cx) + 5,  int(cy), int(cx) + 14, int(cy))
        p.drawLine(int(cx), int(cy) - 4, int(cx), int(cy) + 4)

        p.setFont(QFont("Consolas", 7))
        p.setPen(QPen(_C_DIM))
        p.drawText(int(cx) - r, int(cy) + r + 2, r * 2, 11,
                   Qt.AlignmentFlag.AlignCenter, "ADI")

    # ------------------------------------------------------------------
    # Altitude & throttle ladder tapes
    # ------------------------------------------------------------------

    def _hud_altitude_ladder(self, p: QPainter, w: int, h: int,
                              s: DroneState) -> None:
        self._ladder_tape(p, w - 100, h - 170, 18, 160,
                          s.altitude, 30.0, _C_BLUE,
                          "ALT", "m", step=5)
        thr_pct = (s.throttle - 0.5) * 200
        self._ladder_tape(p, w - 76, h - 170, 18, 160,
                          thr_pct, 100.0, _C_GREEN,
                          "THR", "%", step=25,
                          centre_label="HOLD")

    def _ladder_tape(self, p, x, y, bw, bh, value, max_val,
                     col, label, unit, step=5, centre_label="") -> None:
        p.setBrush(QBrush(QColor(0, 10, 0, 200)))
        p.setPen(QPen(_C_DIM, 1))
        p.drawRect(x, y, bw, bh)

        frac = max(0.0, min(1.0,
                            (value + max_val) / (2 * max_val)
                            if centre_label else value / max_val))
        fh = int(bh * frac)
        if fh > 0:
            p.setBrush(QBrush(QColor(col.red(), col.green(), col.blue(), 120)))
            p.setPen(_PN)
            p.drawRect(x + 1, y + bh - fh, bw - 2, fh)

        p.setFont(QFont("Consolas", 7))
        for tick in range(0, int(max_val) + 1, step):
            frac_t = ((tick + max_val) / (2 * max_val)
                      if centre_label else tick / max_val)
            ty_ = y + bh - int(bh * frac_t)
            is_maj = (tick % (step * 2) == 0)
            p.setPen(QPen(col if is_maj else _C_DIM, 1))
            p.drawLine(x + bw - 5, ty_, x + bw, ty_)
            if is_maj:
                p.drawText(x - 24, ty_ - 5, 22, 10,
                           Qt.AlignmentFlag.AlignRight, str(tick))

        ptr_y = y + bh - int(bh * frac)
        ptr = QPolygonF([
            QPointF(x, ptr_y),
            QPointF(x - 8, ptr_y - 6),
            QPointF(x - 8, ptr_y + 6),
        ])
        p.setBrush(QBrush(col))
        p.setPen(_PN)
        p.drawPolygon(ptr)
        p.setBrush(QBrush(QColor(0, 20, 0, 220)))
        p.setPen(QPen(col, 1))
        p.drawRect(x - 34, ptr_y - 8, 32, 16)
        p.setFont(QFont("Consolas", 8, QFont.Weight.Bold))
        p.setPen(QPen(col))
        p.drawText(x - 34, ptr_y - 8, 32, 16,
                   Qt.AlignmentFlag.AlignCenter, f"{value:.0f}")

        if centre_label:
            mid_y = y + bh // 2
            p.setPen(QPen(_C_AMBER, 1, Qt.PenStyle.DashLine))
            p.drawLine(x, mid_y, x + bw, mid_y)

        p.setFont(QFont("Consolas", 7, QFont.Weight.Bold))
        p.setPen(QPen(col))
        p.drawText(x, y - 12, bw, 11, Qt.AlignmentFlag.AlignCenter, label)
        p.setFont(QFont("Consolas", 7))
        p.setPen(QPen(_C_DIM))
        p.drawText(x, y + bh + 2, bw, 10, Qt.AlignmentFlag.AlignCenter, unit)

    # ------------------------------------------------------------------
    # Threat / status strip
    # ------------------------------------------------------------------

    def _hud_threat_strip(self, p: QPainter, w: int, h: int,
                           s: DroneState) -> None:
        x = w - 52
        p.setBrush(QBrush(QColor(0, 8, 0, 190)))
        p.setPen(QPen(_C_GREEN, 1))
        p.drawRect(x, 100, 46, 220)

        p.setFont(QFont("Consolas", 7, QFont.Weight.Bold))
        p.setPen(QPen(_C_AMBER))
        p.drawText(x, 100, 46, 12, Qt.AlignmentFlag.AlignCenter, "STATUS")
        p.setPen(QPen(QColor(_C_AMBER.red(), _C_AMBER.green(),
                              _C_AMBER.blue(), 80)))
        p.drawLine(x, 112, x + 46, 112)

        items = []
        items.append(("MODE", s.mode.value[:6], _MODE_RGB.get(s.mode.value, (60, 70, 60))))
        items.append(("SPD",  f"{s.speed_h:.1f}", (40, 180, 80)))
        items.append(("ALT",  f"{s.altitude:.1f}", (40, 160, 220)))
        items.append(("HDG",  f"{s.heading:.0f}°", (200, 160, 0)))
        items.append(("THR",  f"{(s.throttle - 0.5) * 200:+.0f}%",
                      (40, 220, 80) if abs(s.throttle - 0.5) < 0.1 else (220, 140, 0)))
        if s.is_airborne:
            items.append(("AIR", "BORN", (40, 220, 80)))
        else:
            items.append(("GND", "LOCK", (200, 80, 40)))

        p.setFont(QFont("Consolas", 7))
        iy = 116
        for key, val, rgb_ in items:
            r2, g2, b2 = rgb_
            p.setPen(QPen(_C_DIM))
            p.drawText(x + 2, iy, 44, 11, Qt.AlignmentFlag.AlignLeft, key)
            p.setPen(QPen(QColor(r2, g2, b2)))
            p.drawText(x + 2, iy + 10, 44, 11, Qt.AlignmentFlag.AlignRight, val)
            p.setPen(QPen(QColor(30, 50, 30, 60)))
            p.drawLine(x + 2, iy + 21, x + 44, iy + 21)
            iy += 24

    # ------------------------------------------------------------------
    # Radar mini-map
    # ------------------------------------------------------------------

    def _hud_minimap(self, p: QPainter, w: int, h: int) -> None:
        s  = self._state
        MR   = 68
        MSCL = 5.5
        mx   = w - MR - 10
        my   = h - MR - 10

        bg_g = QRadialGradient(QPointF(mx, my), MR)
        bg_g.setColorAt(0.0, QColor(0, 20, 0, 210))
        bg_g.setColorAt(1.0, QColor(0, 8, 0, 220))
        p.setBrush(QBrush(bg_g))
        p.setPen(QPen(_C_GREEN, 1))
        p.drawEllipse(QPointF(mx, my), MR, MR)

        for rr in (MR * 0.33, MR * 0.66):
            p.setPen(QPen(QColor(30, 90, 30, 60), 1))
            p.setBrush(Qt.BrushStyle.NoBrush)
            p.drawEllipse(QPointF(mx, my), rr, rr)

        p.setPen(QPen(QColor(30, 80, 30, 50), 1))
        p.drawLine(QPointF(mx - MR + 4, my), QPointF(mx + MR - 4, my))
        p.drawLine(QPointF(mx, my - MR + 4), QPointF(mx, my + MR - 4))

        p.setFont(QFont("Consolas", 7, QFont.Weight.Bold))
        for lbl, dx, dy, col in (
            ("N", 0, -1, _C_GREEN), ("S", 0, 1, _C_RED),
            ("E", 1,  0, _C_AMBER), ("W", -1, 0, _C_BLUE),
        ):
            p.setPen(QPen(col))
            p.drawText(int(mx + dx * (MR - 9)) - 6,
                       int(my + dy * (MR - 9)) - 6,
                       12, 12, Qt.AlignmentFlag.AlignCenter, lbl)

        p.setClipRect(int(mx - MR), int(my - MR), MR * 2, MR * 2)
        trail = list(self._trail)
        n = len(trail)
        for i, (tx, _, tz) in enumerate(trail):
            a  = int(150 * i / max(n, 1))
            rx = mx + (tx - s.x) * MSCL
            ry = my + (tz - s.z) * MSCL
            p.setBrush(QBrush(QColor(40, 200, 80, a)))
            p.setPen(_PN)
            p.drawEllipse(QPointF(rx, ry), 2, 2)

        p.setBrush(QBrush(_C_GREEN))
        p.setPen(QPen(QColor(0, 0, 0, 180), 1))
        p.drawEllipse(QPointF(mx, my), 5, 5)
        yr  = math.radians(s.yaw)
        ahx = mx + math.sin(yr) * 16
        ahy = my - math.cos(yr) * 16
        p.setPen(QPen(_C_AMBER, 2))
        p.drawLine(QPointF(mx, my), QPointF(ahx, ahy))
        p.setBrush(QBrush(_C_AMBER))
        _arrow_head(p, QPointF(ahx, ahy), QPointF(mx, my), 6, 4)

        p.setClipping(False)

        p.setFont(QFont("Consolas", 7))
        p.setPen(QPen(_C_DIM))
        coord_txt = f"X{s.x:+6.1f} Z{s.z:+6.1f}"
        p.drawText(int(mx - MR), int(my + MR + 2), MR * 2, 11,
                   Qt.AlignmentFlag.AlignCenter, coord_txt)

        p.setPen(QPen(QColor(30, 100, 30, 120)))
        p.setFont(QFont("Consolas", 7, QFont.Weight.Bold))
        p.drawText(int(mx - MR), int(my - MR - 12), MR * 2, 11,
                   Qt.AlignmentFlag.AlignCenter, "◆ RADAR ◆")
