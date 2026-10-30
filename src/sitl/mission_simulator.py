"""
Deterministic 6-DOF Closed-Loop Quadrotor Mission Simulator
for Project 8: 5-Inch Enclosed Ducted-Fan Quadrotor (IIIT Hyderabad EC4.402).

What it is
----------
A standalone stand-in for ArduPilot SITL that needs nothing but NumPy/Matplotlib.
It flies the real mission file (missions/iiith_5in_enclosed_mission.waypoints)
through an ArduCopter-style cascaded controller and writes ArduPilot-like
telemetry CSVs that src/analysis/flight_log_analyzer.py evaluates exactly as it
would a real DataFlash log.

Model
-----
- Rigid body, NED frame (x north, y east, z down), quaternion attitude, full
  3x3 diagonal inertia (taken from outputs/sizing/mass_cg_inertia_audit.json
  when present, otherwise the fallback estimates below).
- 4 motors in a true-X at (+/-97.5, +/-97.5) mm, first-order thrust lag,
  ArduPilot motor order/spin directions, yaw from reaction torque only.
- Max thrust per motor scales with loaded pack voltage squared and local air
  density: T_max = 1.98 kgf * (1.09/1.225) * (V/24)^2 * duct net multiplier.
- Electrical power from momentum theory, P = T^1.5 / sqrt(2 rho A) / (FM eta),
  times (1 + kappa) where kappa = 0.20 enclosure loss for Stage 2.
- 6S LiPo: OCV(SoC) curve, 18 mOhm pack resistance, coulomb counting.
- Aerodynamics: duct inlet momentum (ram) drag  F = m_dot * V_rel  (dominant
  for a ducted box, see duct_aero.compute_crosswind_lean), bluff-body form drag
  of the 353 x 353 x 100 mm box, duct-lip pitching moment in crossflow.
- Wind: mean + first-order Gauss-Markov (Dryden-like) turbulence.
- Navigation estimate: GPS/SBAS error as Gauss-Markov drift + white noise,
  baro altitude drift; controllers only ever see the estimate (like the EKF),
  the log records both estimate (what a real log shows) and truth.

Runs
----
  A  : Stage 1 bare quad - ALT_HOLD + 60 s LOITER + LAND (Gates 4-7, 9)
  B0 : Stage 2 enclosed (legs) - ALT_HOLD + 60 s LOITER in 15 km/h wind
  B  : Stage 2 enclosed (legs) - full AUTO mission in 15 km/h wind + turbulence
  B25: same mission at WPNAV_SPEED 2.5 m/s (lean-saturation mitigation study)
  B10: same mission in 10 km/h wind (operating-limit study)
  C1 : RC-loss mid-mission -> RTL
  C2 : Low battery (pack started at 42 %) -> RTL -> critical -> LAND
  C3 : Geofence breach (rogue waypoint 150 m out vs FENCE_RADIUS 120 m) -> RTL
  D  : Stage 2 still-air endurance hover to 80 % pack used (checks 5.4/4.5 min)
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src" / "sizing"))

from duct_aero import get_locked_5in_geometry, evaluate_duct_aero  # noqa: E402
from sizing_model import (  # noqa: E402
    AVIONICS_POWER_W,
    FIGURE_OF_MERIT,
    GRAVITY_MS2,
    MOTOR_ESC_EFF,
    RHO_HYD_KGM3,
    RHO_SL_KGM3,
    get_locked_5in_bom,
    summarize_bom,
)

# ============================================================================
# 1. Site, mission & autopilot constants (mirror config/ardupilot/*.param)
# ============================================================================
HOME_LAT, HOME_LON, HOME_AMSL = 17.4455, 78.3489, 540.0
M_PER_DEG_LAT = 110674.14      # at 17.4455 N
M_PER_DEG_LON = 106231.00

DT = 0.004                     # 250 Hz physics + control
LOG_HZ = 10.0
ARM_M = 0.0975                 # motor offset from centre on X and Y (m)
PROP_D_M = 0.127
DISC_AREA_M2 = math.pi * PROP_D_M ** 2 / 4.0
K_YAW_M = 0.0125               # reaction torque / thrust (m), 5" tri-blade estimate

WPNAV_SPEED = 3.5              # m/s
WPNAV_ACCEL = 1.2              # m/s^2
WPNAV_SPEED_UP = 1.5           # 150 cm/s, as .param
WPNAV_SPEED_DN = 1.0           # 100 cm/s, as .param
WPNAV_RADIUS = 1.0             # 100 cm, as config/ardupilot/*.param
LAND_SPEED = 0.4
LAND_ALT_LOW = 10.0
RTL_ALT = 15.0
RTL_LOIT_TIME = 3.0
ANGLE_MAX_DEG = 30.0
FENCE_RADIUS = 120.0
FENCE_ALT_MAX = 40.0
LGR_DEPLOY_ALT = 3.0
LGR_RETRACT_ALT = 5.0
GEAR_TRANSIT_S = 1.5           # MG90S sweep, end-point to end-point
BATT_LOW_VOLT = 21.0           # resting-voltage estimate, as ArduPilot
BATT_CRT_VOLT = 19.8
BATT_CAPACITY_MAH = 2600.0
BATT_LOW_MAH = 780.0           # 30 % remaining -> RTL
BATT_CRT_MAH = 450.0           # 17 % remaining -> LAND (matches .param files)
MOT_THST_EXPO = 0.65
MOT_SPIN_MIN, MOT_SPIN_MAX = 0.15, 0.95

# Gate thresholds (docs/wiring_and_bringup_guide.md)
WIND_MISSION_MS = 15.0 / 3.6   # 4.17 m/s


# ============================================================================
# 2. Vehicle definitions
# ============================================================================
@dataclass
class Vehicle:
    name: str
    stage: str
    mass_kg: float
    inertia: Tuple[float, float, float]
    thrust_mult: float          # duct + grille + box net multiplier on thrust
    power_loss: float           # kappa: extra hover power fraction
    ram_factor: float           # fraction of full momentum drag (1 = ducted)
    side_area_m2: float
    top_area_m2: float
    cd_side: float
    lip_height_m: float         # lever arm for crossflow pitching moment
    motor_tau_s: float
    vibe_base: float
    thst_hover_init: float
    has_legs: bool = True

    def tmax_n(self, volts: float) -> float:
        """Usable max thrust per motor at the present loaded pack voltage."""
        base = 1.98 * GRAVITY_MS2 * (RHO_HYD_KGM3 / RHO_SL_KGM3)
        return base * (volts / 24.0) ** 2 * self.thrust_mult


def _load_inertia(key_words: List[str], fallback: Tuple[float, float, float]) -> Tuple[Tuple[float, float, float], str]:
    """Pull (Ixx, Iyy, Izz) for a stage from the CAD audit JSON if it exists."""
    path = ROOT / "outputs" / "sizing" / "mass_cg_inertia_audit.json"
    if not path.exists():
        return fallback, "fallback estimate (audit JSON not found)"
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return fallback, "fallback estimate (audit JSON unreadable)"

    found: List[Tuple[str, Tuple[float, float, float]]] = []

    def walk(node, trail: str):
        if isinstance(node, dict):
            keys = {k.lower(): k for k in node}
            ixx = next((keys[k] for k in keys if k.startswith("ixx")), None)
            iyy = next((keys[k] for k in keys if k.startswith("iyy")), None)
            izz = next((keys[k] for k in keys if k.startswith("izz")), None)
            if ixx and iyy and izz:
                try:
                    vals = (float(node[ixx]), float(node[iyy]), float(node[izz]))
                    found.append((trail.lower(), vals))
                except (TypeError, ValueError):
                    pass
            for k, v in node.items():
                walk(v, f"{trail}/{k}")
        elif isinstance(node, list):
            for i, v in enumerate(node):
                walk(v, f"{trail}/{i}")

    walk(data, "")
    for trail, vals in found:
        if all(w in trail for w in key_words):
            # Accept either kg m^2 or g mm^2 / kg mm^2 by magnitude check
            scale = 1.0
            if vals[2] > 10.0:
                scale = 1e-9 if vals[2] > 1e4 * 10 else 1e-6
            v = tuple(x * scale for x in vals)
            if 1e-4 < v[2] < 1.0:
                return v, f"mass_cg_inertia_audit.json ({trail})"
    return fallback, "fallback estimate (stage not found in audit JSON)"


def build_vehicles() -> Dict[str, Vehicle]:
    geom = get_locked_5in_geometry(1.5)
    encl = evaluate_duct_aero(geom, include_grilles=True, include_enclosurebox=True)

    stage1_mass = summarize_bom(get_locked_5in_bom(True), stage1_only=True)["all_up_mass_kg"]
    stage2_mass = summarize_bom(get_locked_5in_bom(True))["all_up_mass_kg"]

    i1, src1 = _load_inertia(["inertia_headline", "stage 1 bare"], (0.0068, 0.0068, 0.0122))
    i2, src2 = _load_inertia(["inertia_headline", "legs up"], (0.0215, 0.0215, 0.0390))

    s1 = Vehicle(
        name="Stage 1 Bare Quad (open props, legs, 180 g payload)", stage="stage1",
        mass_kg=stage1_mass, inertia=i1, thrust_mult=1.0, power_loss=0.0,
        ram_factor=0.5, side_area_m2=0.020, top_area_m2=0.045, cd_side=1.0,
        lip_height_m=0.0, motor_tau_s=0.035, vibe_base=5.5,
        thst_hover_init=0.20,
    )
    s2 = Vehicle(
        name="Stage 2 Enclosed 5-inch (retractable legs)", stage="stage2",
        mass_kg=stage2_mass, inertia=i2, thrust_mult=encl.net_thrust_multiplier,
        power_loss=0.20, ram_factor=1.0,
        side_area_m2=geom.box_width_mm * geom.box_height_mm * 1e-6,
        top_area_m2=(geom.box_width_mm * 1e-3) ** 2 * 0.25,  # grilles let air through
        cd_side=1.15, lip_height_m=0.030, motor_tau_s=0.045, vibe_base=7.5,
        thst_hover_init=0.34,
    )
    s1.inertia_source, s2.inertia_source = src1, src2  # type: ignore[attr-defined]
    return {"stage1": s1, "stage2": s2}


# ============================================================================
# 3. Battery
# ============================================================================
_SOC_PTS = np.array([0.0, 0.05, 0.10, 0.20, 0.30, 0.40, 0.50, 0.60, 0.70, 0.80, 0.90, 1.0])
_OCV_PTS = np.array([3.30, 3.45, 3.55, 3.65, 3.71, 3.76, 3.80, 3.85, 3.92, 4.00, 4.09, 4.20])


class Battery:
    def __init__(self, cells: int = 6, cap_mah: float = 2600.0, r_pack: float = 0.018, soc0: float = 1.0):
        self.cells, self.cap_mah, self.r = cells, cap_mah, r_pack
        self.used_mah = (1.0 - soc0) * cap_mah
        self.current = 0.0
        self.voltage = self.ocv()

    @property
    def soc(self) -> float:
        return max(0.0, 1.0 - self.used_mah / self.cap_mah)

    def ocv(self) -> float:
        return self.cells * float(np.interp(self.soc, _SOC_PTS, _OCV_PTS))

    def draw(self, power_w: float, dt: float) -> None:
        # Solve P = V I with V = OCV - I R  ->  I = (OCV - sqrt(OCV^2 - 4 R P)) / 2R
        ocv = self.ocv()
        disc = max(ocv * ocv - 4.0 * self.r * power_w, 0.0)
        self.current = (ocv - math.sqrt(disc)) / (2.0 * self.r)
        self.voltage = ocv - self.current * self.r
        self.used_mah += self.current * dt / 3.6

    @property
    def resting_voltage_est(self) -> float:
        return self.voltage + self.current * self.r


# ============================================================================
# 4. Math helpers
# ============================================================================
def quat_to_rot(q: np.ndarray) -> np.ndarray:
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


def quat_to_euler(q: np.ndarray) -> Tuple[float, float, float]:
    w, x, y, z = q
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return roll, pitch, yaw


def wrap_pi(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


def ne_to_latlon(n: float, e: float) -> Tuple[float, float]:
    return HOME_LAT + n / M_PER_DEG_LAT, HOME_LON + e / M_PER_DEG_LON


def latlon_to_ne(lat: float, lon: float) -> Tuple[float, float]:
    return (lat - HOME_LAT) * M_PER_DEG_LAT, (lon - HOME_LON) * M_PER_DEG_LON


def thrust_frac_to_pwm(f: float) -> float:
    f = min(max(f, 0.0), 1.0)
    e = MOT_THST_EXPO
    u = (-(1 - e) + math.sqrt((1 - e) ** 2 + 4 * e * f)) / (2 * e)
    return 1000.0 + 1000.0 * (MOT_SPIN_MIN + (MOT_SPIN_MAX - MOT_SPIN_MIN) * u)


class GaussMarkov:
    """First-order Gauss-Markov process (used for turbulence and GPS drift)."""

    def __init__(self, rng: np.random.Generator, sigma: float, tau: float, dims: int = 3):
        self.rng, self.sigma, self.tau = rng, np.asarray(sigma, float), tau
        self.x = np.zeros(dims)

    def step(self, dt: float) -> np.ndarray:
        a = math.exp(-dt / self.tau)
        self.x = a * self.x + self.sigma * math.sqrt(1 - a * a) * self.rng.standard_normal(self.x.shape)
        return self.x


# ============================================================================
# 5. Mission items
# ============================================================================
@dataclass
class Item:
    kind: str                    # takeoff | wp | hold | land | gear | marker | rtl
    seq: int = -1
    n: float = 0.0
    e: float = 0.0
    alt: float = 0.0
    hold_s: float = 0.0
    mode: str = "AUTO"
    gear_up: Optional[bool] = None
    pilot: bool = False          # ALT_HOLD: no GPS position loop, pilot corrects slowly


def load_waypoints(path: Path) -> List[Item]:
    lines = path.read_text(encoding="utf-8").strip().splitlines()
    if not lines[0].startswith("QGC WPL 110"):
        raise ValueError(f"{path} is not a QGC WPL 110 file")
    items: List[Item] = []
    for ln in lines[1:]:
        f = ln.split("\t")
        seq, cmd = int(f[0]), int(f[3])
        p1, p2 = float(f[4]), float(f[5])
        lat, lon, alt = float(f[8]), float(f[9]), float(f[10])
        if seq == 0:
            continue  # home
        n, e = latlon_to_ne(lat, lon) if (lat != 0.0 or lon != 0.0) else (0.0, 0.0)
        if cmd == 22:
            items.append(Item("takeoff", seq, n, e, alt))
        elif cmd == 16:
            items.append(Item("wp", seq, n, e, alt, hold_s=p1))
        elif cmd == 19:
            items.append(Item("wp", seq, n, e, alt, hold_s=p1))
        elif cmd == 21:
            items.append(Item("land", seq, n, e, 0.0))
        elif cmd == 20:
            items.append(Item("rtl", seq))
        elif cmd == 2520:
            items.append(Item("gear", seq, gear_up=(p2 >= 0.5)))
        elif cmd == 189:
            items.append(Item("marker", seq))
        else:
            print(f"  [mission] seq {seq}: MAV_CMD {cmd} not modelled, skipped")
    return items


# ============================================================================
# 6. Simulator
# ============================================================================
@dataclass
class RunConfig:
    run_id: str
    title: str
    vehicle: str
    items: List[Item]
    wind_mean_ms: float = 0.0
    wind_from_deg: float = 315.0
    turb_sigma: float = 0.0
    soc0: float = 1.0
    rc_loss_t: Optional[float] = None
    stop_at_used_frac: Optional[float] = None
    failsafes: bool = True
    wp_speed: float = WPNAV_SPEED
    max_time_s: float = 600.0
    seed: int = 8


class Sim:
    def __init__(self, cfg: RunConfig, veh: Vehicle):
        self.cfg, self.veh = cfg, veh
        self.rng = np.random.default_rng(cfg.seed)
        self.t = 0.0
        # True state
        self.p = np.zeros(3)                  # NED (m)
        self.v = np.zeros(3)
        self.q = np.array([1.0, 0.0, 0.0, 0.0])
        self.w = np.zeros(3)                  # body rates
        self.I = np.array(veh.inertia)
        self.T = np.zeros(4)                  # actual motor thrusts (N)
        self.batt = Battery(soc0=cfg.soc0)
        # Motor geometry: ArduPilot QUAD/X order, 1 FR CCW, 2 RL CCW, 3 FL CW, 4 RR CW
        self.rx = np.array([+ARM_M, -ARM_M, +ARM_M, -ARM_M])
        self.ry = np.array([+ARM_M, -ARM_M, -ARM_M, +ARM_M])
        self.spin = np.array([+1.0, +1.0, -1.0, -1.0])
        B = np.vstack([np.ones(4), -self.ry, self.rx, K_YAW_M * self.spin])
        self.Binv = np.linalg.inv(B)
        # Environment
        wf = math.radians(cfg.wind_from_deg)
        self.wind_mean = -cfg.wind_mean_ms * np.array([math.cos(wf), math.sin(wf), 0.0])
        self.turb = GaussMarkov(self.rng, [cfg.turb_sigma, cfg.turb_sigma, 0.45 * cfg.turb_sigma], 5.0)
        self.gps_drift = GaussMarkov(self.rng, [0.30, 0.30, 0.15], 25.0)
        self.wind = self.wind_mean.copy()
        # Controller state
        self.i_vel = np.zeros(3)
        self.i_rate = np.zeros(3)
        self.prev_rate_err = np.zeros(3)
        self.thr_hover = veh.thst_hover_init
        self.armed = False
        self.landed = True
        self.ground_t = 0.0
        # Guidance state
        self.items = list(cfg.items)
        self.idx = 0
        self.mode = "STABILIZE"
        self.item_t0 = 0.0
        self.seg_a = np.zeros(3)
        self.seg_s = 0.0
        self.seg_v = 0.0
        self.alt_ramp = 0.0
        self.hold_t0: Optional[float] = None
        self.tgt = np.zeros(3)                 # target NE + alt(up)
        self.tgt_vel = np.zeros(2)
        self.z_mode = "alt"
        self.z_rate = 0.0
        self.active_wp: Optional[Item] = None
        self.rtl_phase: Optional[str] = None
        self.failsafe: Optional[str] = None
        self.gear_pos = 1.0                    # 1 = down (deployed), 0 = up
        self.gear_cmd_down = True
        self.events: List[Tuple[float, str]] = []
        self.rows: List[Dict] = []
        self.est_p = np.zeros(3)
        self.est_v = np.zeros(3)
        self.done = False
        self.max_dist_home = 0.0
        self.fence_breached = False
        self.batt_low_flag = False
        self.batt_crt_flag = False
        self.pending_event = ""

    # ------------------------------------------------------------------ events
    def event(self, msg: str) -> None:
        self.events.append((round(self.t, 2), msg))
        self.pending_event = (self.pending_event + "; " if self.pending_event else "") + msg

    # ----------------------------------------------------------------- estimate
    def update_estimate(self) -> None:
        drift = self.gps_drift.step(DT)
        white = self.rng.standard_normal(3) * np.array([0.06, 0.06, 0.04])
        self.est_p = self.p + drift + white
        self.est_v = self.v + self.rng.standard_normal(3) * 0.04

    @property
    def est_alt(self) -> float:
        return -self.est_p[2]

    # ----------------------------------------------------------------- guidance
    def start_item(self) -> None:
        while self.idx < len(self.items):
            it = self.items[self.idx]
            self.item_t0 = self.t
            if it.kind == "gear":
                self.gear_cmd_down = not it.gear_up
                self.event(f"MISSION gear {'RETRACT' if it.gear_up else 'DEPLOY'} (seq {it.seq})")
                self.idx += 1
                continue
            if it.kind == "marker":
                self.event(f"DO_LAND_START marker (seq {it.seq})")
                self.idx += 1
                continue
            if it.kind == "rtl":
                self.enter_rtl("mission RTL")
                return
            self.mode = it.mode
            self.active_wp = it
            self.seg_a = np.array([self.tgt[0], self.tgt[1], self.tgt[2]])
            self.seg_s, self.seg_v, self.hold_t0 = 0.0, 0.0, None
            if it.kind == "takeoff":
                if not self.armed:
                    self.armed = True
                    self.event("ARMED")
                self.tgt = np.array([self.est_p[0], self.est_p[1], 0.0])
                self.alt_ramp = 0.0
            return
        self.done = True

    def enter_rtl(self, reason: str) -> None:
        if self.mode in ("RTL", "LAND"):
            return
        self.mode = "RTL"
        self.rtl_phase = "climb"
        self.failsafe = reason
        self.active_wp = None
        self.tgt = np.array([self.est_p[0], self.est_p[1], max(self.est_alt, 0.0)])
        self.seg_s, self.seg_v = 0.0, 0.0
        self.event(f"MODE RTL ({reason})")

    def enter_land(self, reason: str) -> None:
        if self.mode == "LAND":
            return
        self.mode = "LAND"
        self.rtl_phase = None
        self.active_wp = Item("land", -1, self.est_p[0], self.est_p[1], 0.0)
        self.tgt = np.array([self.est_p[0], self.est_p[1], self.est_alt])
        self.event(f"MODE LAND ({reason})")

    def advance_carrot(self, b_ne: np.ndarray, b_alt: float, speed: float) -> bool:
        """Moves a target point along segment a->b. Returns True when it has reached b."""
        a_ne = self.seg_a[:2]
        d = b_ne - a_ne
        L = float(np.linalg.norm(d))
        u = d / L if L > 1e-6 else np.zeros(2)
        remaining = L - self.seg_s
        v_des = min(speed, math.sqrt(max(2.0 * WPNAV_ACCEL * remaining, 0.0)))
        # Leash: do not run the carrot more than 4 m ahead of the vehicle
        carrot = a_ne + u * self.seg_s
        if np.linalg.norm(carrot - self.est_p[:2]) > 4.0:
            v_des = min(v_des, 0.5 * self.seg_v)
        dv = max(-WPNAV_ACCEL * DT, min(WPNAV_ACCEL * DT, v_des - self.seg_v))
        self.seg_v = max(0.0, self.seg_v + dv)
        self.seg_s = min(L, self.seg_s + self.seg_v * DT)
        frac = self.seg_s / L if L > 1e-6 else 1.0
        self.tgt[:2] = a_ne + u * self.seg_s
        self.tgt[2] = self.seg_a[2] + (b_alt - self.seg_a[2]) * frac
        self.tgt_vel = u * self.seg_v
        self.z_mode = "alt"
        return self.seg_s >= L - 1e-3

    def guidance(self) -> None:
        self.tgt_vel = np.zeros(2)
        self.z_mode = "alt"
        if self.mode == "RTL":
            self.rtl_guidance()
            return
        if self.mode == "LAND":
            self.land_guidance()
            return
        it = self.active_wp
        if it is None:
            return
        if it.kind == "takeoff":
            self.alt_ramp = min(it.alt, self.alt_ramp + WPNAV_SPEED_UP * DT)
            self.tgt[2] = self.alt_ramp
            if self.alt_ramp >= it.alt and self.est_alt > it.alt - 0.5:
                self.event(f"TAKEOFF complete {it.alt:.0f} m")
                self.idx += 1
                self.start_item()
        elif it.kind in ("wp", "hold"):
            if it.kind == "hold":
                self.tgt[2] = it.alt if it.alt > 0 else self.tgt[2]
                if self.t - self.item_t0 >= it.hold_s:
                    self.idx += 1
                    self.start_item()
                return
            arrived = self.advance_carrot(np.array([it.n, it.e]), it.alt, self.cfg.wp_speed)
            dist = float(np.linalg.norm(np.array([it.n, it.e]) - self.est_p[:2]))
            if arrived and dist <= WPNAV_RADIUS:
                if self.hold_t0 is None:
                    self.hold_t0 = self.t
                    self.event(f"Reached WP seq {it.seq} (err {dist:.2f} m)")
                if self.t - self.hold_t0 >= it.hold_s:
                    self.idx += 1
                    self.start_item()
        elif it.kind == "land":
            self.land_guidance()

    def rtl_guidance(self) -> None:
        if self.rtl_phase == "climb":
            target_alt = max(RTL_ALT, self.tgt[2])
            self.tgt[2] = min(target_alt, self.tgt[2] + WPNAV_SPEED_UP * DT) if self.tgt[2] < target_alt else self.tgt[2]
            if self.est_alt >= target_alt - 0.5:
                self.rtl_phase = "return"
                self.seg_a = np.array([self.est_p[0], self.est_p[1], self.tgt[2]])
                self.seg_s = self.seg_v = 0.0
        elif self.rtl_phase == "return":
            arrived = self.advance_carrot(np.zeros(2), self.seg_a[2], self.cfg.wp_speed)
            if arrived and np.linalg.norm(self.est_p[:2]) < WPNAV_RADIUS:
                self.rtl_phase = "loiter"
                self.item_t0 = self.t
        elif self.rtl_phase == "loiter":
            if self.t - self.item_t0 >= RTL_LOIT_TIME:
                self.rtl_phase = "land"
                self.active_wp = Item("land", -1, 0.0, 0.0, 0.0)
        elif self.rtl_phase == "land":
            self.land_guidance()

    def land_guidance(self) -> None:
        it = self.active_wp
        if it is not None:
            self.tgt[0], self.tgt[1] = it.n, it.e
        self.z_mode = "rate"
        self.z_rate = -(WPNAV_SPEED_DN if self.est_alt > LAND_ALT_LOW else LAND_SPEED)
        if self.mode not in ("RTL", "LAND"):
            self.mode = it.mode if it else "AUTO"

    # ----------------------------------------------------------------- failsafes
    def check_failsafes(self) -> None:
        if not self.armed or self.landed or not self.cfg.failsafes:
            return
        if self.cfg.rc_loss_t is not None and self.t >= self.cfg.rc_loss_t and self.failsafe is None:
            self.event("FAILSAFE radio (RC lost)")
            self.enter_rtl("RC failsafe")
        rem_mah = BATT_CAPACITY_MAH - self.batt.used_mah
        v_rest = self.batt.resting_voltage_est
        if not self.batt_low_flag and (rem_mah <= BATT_LOW_MAH or v_rest <= BATT_LOW_VOLT):
            self.batt_low_flag = True
            self.event(f"FAILSAFE battery LOW ({rem_mah:.0f} mAh, {v_rest:.2f} V rest)")
            self.enter_rtl("battery low")
        if not self.batt_crt_flag and (rem_mah <= BATT_CRT_MAH or v_rest <= BATT_CRT_VOLT):
            self.batt_crt_flag = True
            self.event(f"FAILSAFE battery CRITICAL ({rem_mah:.0f} mAh, {v_rest:.2f} V rest)")
            self.enter_land("battery critical")
        dist = float(np.linalg.norm(self.est_p[:2]))
        if not self.fence_breached and (dist > FENCE_RADIUS or self.est_alt > FENCE_ALT_MAX):
            self.fence_breached = True
            self.event(f"FENCE breach ({dist:.1f} m from home, alt {self.est_alt:.1f} m)")
            self.enter_rtl("geofence")

    # ----------------------------------------------------------------- gear
    def update_gear(self) -> None:
        if self.veh.has_legs and self.armed and not self.landed:
            climbing = self.est_v[2] < -0.2
            descending = self.est_v[2] > 0.1
            if self.est_alt > LGR_RETRACT_ALT and climbing and self.gear_cmd_down and self.mode != "LAND" \
                    and self.rtl_phase not in ("land",):
                self.gear_cmd_down = False
                self.event(f"GEAR retract (alt {self.est_alt:.1f} m)")
            if self.est_alt < LGR_DEPLOY_ALT and descending and not self.gear_cmd_down:
                self.gear_cmd_down = True
                self.event(f"GEAR deploy (alt {self.est_alt:.1f} m)")
        target = 1.0 if self.gear_cmd_down else 0.0
        step = DT / GEAR_TRANSIT_S
        self.gear_pos += max(-step, min(step, target - self.gear_pos))

    # ----------------------------------------------------------------- control
    def control(self, tmax: float) -> np.ndarray:
        veh, m, g = self.veh, self.veh.mass_kg, GRAVITY_MS2
        if not self.armed:
            return np.zeros(4)
        roll, pitch, yaw = quat_to_euler(self.q)

        # --- vertical (alt -> climb rate -> accel)
        if self.z_mode == "alt":
            vz_t = 1.0 * (self.tgt[2] - self.est_alt)
            vz_t = max(-WPNAV_SPEED_DN, min(WPNAV_SPEED_UP, vz_t))
        else:
            vz_t = self.z_rate
            self.tgt[2] = self.est_alt
        vz_up = -self.est_v[2]
        ez = vz_t - vz_up
        self.i_vel[2] = max(-2.0, min(2.0, self.i_vel[2] + ez * DT))
        az_up = 4.0 * ez + 2.0 * self.i_vel[2]

        # --- horizontal (pos -> vel -> accel)
        pilot = self.active_wp is not None and self.active_wp.pilot and self.mode == "ALT_HOLD"
        # PSC_POSXY_P 1.0, PSC_VELXY_P 2.0, PSC_VELXY_I 1.0, PSC_VELXY_IMAX 8 m/s^2 (ArduPilot allows 10)
        kp_pos, kp_vel, ki_vel = (0.25, 0.8, 0.3) if pilot else (1.0, 2.0, 1.0)
        e_pos = self.tgt[:2] - self.est_p[:2]
        v_t = self.tgt_vel + kp_pos * e_pos
        vn = float(np.linalg.norm(v_t))
        vmax = WPNAV_SPEED + 1.0
        if vn > vmax:
            v_t *= vmax / vn
        e_vel = v_t - self.est_v[:2]
        self.i_vel[:2] = np.clip(self.i_vel[:2] + e_vel * DT, -8.0 / ki_vel, 8.0 / ki_vel)
        a_ne = kp_vel * e_vel + ki_vel * self.i_vel[:2]
        a_lim = g * math.tan(math.radians(ANGLE_MAX_DEG))
        an = float(np.linalg.norm(a_ne))
        if an > a_lim:
            a_ne *= a_lim / an
        if self.landed and self.mode != "LAND":
            a_ne = np.zeros(2)
            self.i_vel[:2] = 0.0
        cy, sy = math.cos(yaw), math.sin(yaw)
        a_fwd = a_ne[0] * cy + a_ne[1] * sy
        a_rgt = -a_ne[0] * sy + a_ne[1] * cy
        pitch_d = -math.atan2(a_fwd, g + az_up)
        roll_d = math.atan2(a_rgt * math.cos(pitch_d), g + az_up)
        lim = math.radians(ANGLE_MAX_DEG)
        pitch_d = max(-lim, min(lim, pitch_d))
        roll_d = max(-lim, min(lim, roll_d))
        yaw_d = 0.0  # WP_YAW_BEHAVIOR = 0: hold heading north (weak yaw axis in a ducted box)
        self.des_att = (roll_d, pitch_d, yaw_d)

        # --- collective
        tilt = max(math.cos(roll) * math.cos(pitch), 0.5)
        t_sum = m * (g + az_up) / tilt
        t_sum = max(0.08 * 4 * tmax, min(0.98 * 4 * tmax, t_sum))
        if self.landed and self.z_mode == "rate":
            t_sum = 0.0

        # --- attitude (angle P -> rate PID -> torque), ArduPilot-like rate limits
        rate_max = np.radians([180.0, 180.0, 45.0 if veh.stage == "stage2" else 90.0])
        ang_err = np.array([roll_d - roll, pitch_d - pitch, wrap_pi(yaw_d - yaw)])
        rate_t = np.clip(np.array([4.5, 4.5, 3.0]) * ang_err, -rate_max, rate_max)
        rate_err = rate_t - self.w
        d_err = (rate_err - self.prev_rate_err) / DT
        self.prev_rate_err = rate_err
        kp, ki, kd = np.array([18.0, 18.0, 6.0]), np.array([8.0, 8.0, 1.5]), np.array([0.25, 0.25, 0.0])
        # ATC_RAT_*_IMAX = 0.5: integrator may hold up to 50 % of each axis' torque authority. It must be
        # sized in torque, not angular acceleration, because it trims steady moments (duct-lip moment in wind).
        tau_auth = np.array([2 * ARM_M * tmax, 2 * ARM_M * tmax, 2 * K_YAW_M * tmax])
        i_lim = 0.5 * tau_auth / (ki * self.I)
        self.i_rate = np.clip(self.i_rate + rate_err * DT, -i_lim, i_lim)
        alpha = kp * rate_err + ki * self.i_rate + kd * d_err
        tau = self.I * alpha

        # --- mixer with yaw-priority desaturation
        base = self.Binv @ np.array([t_sum, tau[0], tau[1], 0.0])
        yaw_part = self.Binv @ np.array([0.0, 0.0, 0.0, tau[2]])
        scale = 1.0
        for b, y in zip(base, yaw_part):
            if y > 1e-9:
                scale = min(scale, max(0.0, (tmax - b) / y))
            elif y < -1e-9:
                scale = min(scale, max(0.0, (0.0 - b) / y))
        cmd = np.clip(base + scale * yaw_part, 0.02 * tmax, tmax)
        self.clip = int(np.any(cmd >= tmax - 1e-6))
        return cmd

    # ----------------------------------------------------------------- physics
    def physics(self, cmd: np.ndarray) -> None:
        veh = self.veh
        self.T += (cmd - self.T) * min(1.0, DT / veh.motor_tau_s)
        if not self.armed:
            self.T[:] = 0.0
        R = quat_to_rot(self.q)
        t_sum = float(self.T.sum())

        # wind
        self.wind = self.wind_mean + self.turb.step(DT)
        alt = -self.p[2]
        shear = (max(alt, 0.3) / 10.0) ** 0.14 if alt < 10 else 1.0  # power-law boundary layer
        wind = self.wind.copy()
        wind[:2] *= shear
        v_rel = self.v - wind

        # momentum (ram) drag: each fan swallows m_dot = rho A 2 v_i and turns it
        vi = np.sqrt(np.maximum(self.T, 0.0) / (2.0 * RHO_HYD_KGM3 * DISC_AREA_M2))
        mdot = float(np.sum(RHO_HYD_KGM3 * DISC_AREA_M2 * 2.0 * vi))
        f_ram = np.array([-veh.ram_factor * mdot * v_rel[0], -veh.ram_factor * mdot * v_rel[1], 0.0])
        # bluff-body form drag
        areas = np.array([veh.side_area_m2, veh.side_area_m2, veh.top_area_m2])
        f_form = -0.5 * RHO_HYD_KGM3 * veh.cd_side * areas * np.abs(v_rel) * v_rel

        f = R @ np.array([0.0, 0.0, -t_sum]) + np.array([0.0, 0.0, veh.mass_kg * GRAVITY_MS2]) + f_ram + f_form
        acc = f / veh.mass_kg

        # torques: rotor thrust & reaction, duct lip moment in crossflow, damping
        tau = np.array([float(-self.ry @ self.T), float(self.rx @ self.T), float(K_YAW_M * (self.spin @ self.T))])
        if veh.lip_height_m > 0:
            r_lip_world = R @ np.array([0.0, 0.0, -veh.lip_height_m])
            tau += R.T @ np.cross(r_lip_world, f_ram)
        tau -= 0.002 * self.w
        w_dot = (tau - np.cross(self.w, self.I * self.w)) / self.I

        # integrate
        self.v += acc * DT
        self.p += self.v * DT
        self.w += w_dot * DT
        wx, wy, wz = self.w
        omega = np.array([[0, -wx, -wy, -wz], [wx, 0, wz, -wy], [wy, -wz, 0, wx], [wz, wy, -wx, 0]])
        self.q += 0.5 * (omega @ self.q) * DT
        self.q /= np.linalg.norm(self.q)

        # ground contact
        if self.p[2] >= 0.0:
            self.p[2] = 0.0
            if self.v[2] > 0:
                self.v[2] = 0.0
            if t_sum < 0.95 * veh.mass_kg * GRAVITY_MS2:
                self.v[:2] *= 0.0
                self.w *= 0.0
                r, p_, y = quat_to_euler(self.q)
                cy, sy = math.cos(y / 2), math.sin(y / 2)
                self.q = np.array([cy, 0.0, 0.0, sy])
                if not self.landed:
                    self.landed = True
                    self.ground_t = self.t
            else:
                self.landed = False
        elif self.p[2] < -0.05:
            self.landed = False

        # electrical
        p_ind = np.maximum(self.T, 0.0) ** 1.5 / math.sqrt(2.0 * RHO_HYD_KGM3 * DISC_AREA_M2)
        p_elec = float(np.sum(p_ind)) / (FIGURE_OF_MERIT * MOTOR_ESC_EFF) * (1.0 + veh.power_loss)
        idle = 4 * 3.0 if self.armed else 0.0
        self.batt.draw(p_elec + idle + AVIONICS_POWER_W + (4.0 if abs(self.gear_pos - (1.0 if self.gear_cmd_down else 0.0)) > 1e-6 else 0.0), DT)

    # ----------------------------------------------------------------- logging
    def log(self, tmax: float) -> None:
        roll, pitch, yaw = quat_to_euler(self.q)
        lat, lon = ne_to_latlon(self.est_p[0], self.est_p[1])
        frac = self.T / tmax if tmax > 0 else np.zeros(4)
        thr = float(self.T.sum() / (4 * tmax)) if self.armed else 0.0
        it = self.active_wp
        xtrack, dist_wp, wp_seq = "", "", ""
        if it is not None and it.kind == "wp" and self.mode == "AUTO":
            a, b = self.seg_a[:2], np.array([it.n, it.e])
            d = b - a
            L = float(np.linalg.norm(d))
            rel = self.est_p[:2] - a
            if L > 1e-6:
                u = d / L
                xtrack = f"{abs(u[0] * rel[1] - u[1] * rel[0]):.3f}"
            dist_wp = f"{float(np.linalg.norm(b - self.est_p[:2])):.3f}"
            wp_seq = it.seq
        thrust_load = thr
        vib = self.veh.vibe_base + 9.0 * thrust_load
        vibe = [max(0.0, vib * k + self.rng.normal(0, 0.6)) for k in (0.85, 0.90, 1.25)] if self.armed and not self.landed else [0.3, 0.3, 0.4]
        des = getattr(self, "des_att", (0.0, 0.0, 0.0))
        self.rows.append({
            "t_s": round(self.t, 2), "run_id": self.cfg.run_id, "stage": self.veh.stage,
            "mode": self.mode, "armed": int(self.armed), "landed": int(self.landed),
            "lat": f"{lat:.8f}", "lon": f"{lon:.8f}",
            "alt_m": round(self.est_alt, 3), "alt_true_m": round(-self.p[2], 3),
            "alt_target_m": round(float(self.tgt[2]), 3),
            "pos_n_m": round(self.est_p[0], 3), "pos_e_m": round(self.est_p[1], 3),
            "true_n_m": round(self.p[0], 3), "true_e_m": round(self.p[1], 3),
            "tgt_n_m": round(float(self.tgt[0]), 3), "tgt_e_m": round(float(self.tgt[1]), 3),
            "vn_ms": round(self.v[0], 3), "ve_ms": round(self.v[1], 3), "vd_ms": round(self.v[2], 3),
            "gnd_speed_ms": round(float(np.linalg.norm(self.v[:2])), 3),
            "roll_deg": round(math.degrees(roll), 2), "pitch_deg": round(math.degrees(pitch), 2),
            "yaw_deg": round(math.degrees(yaw), 2),
            "des_roll_deg": round(math.degrees(des[0]), 2), "des_pitch_deg": round(math.degrees(des[1]), 2),
            "wind_n_ms": round(self.wind[0], 3), "wind_e_ms": round(self.wind[1], 3),
            "thr_out": round(thr, 4), "thr_hover_learned": round(self.thr_hover, 4),
            "m1_pwm": round(thrust_frac_to_pwm(frac[0]) if self.armed else 1000, 0),
            "m2_pwm": round(thrust_frac_to_pwm(frac[1]) if self.armed else 1000, 0),
            "m3_pwm": round(thrust_frac_to_pwm(frac[2]) if self.armed else 1000, 0),
            "m4_pwm": round(thrust_frac_to_pwm(frac[3]) if self.armed else 1000, 0),
            "current_a": round(self.batt.current, 3), "voltage_v": round(self.batt.voltage, 3),
            "mah_used": round(self.batt.used_mah, 2),
            "batt_remaining_pct": round(100.0 * self.batt.soc, 2),
            "wp_seq": wp_seq, "xtrack_m": xtrack, "dist_to_wp_m": dist_wp,
            "gear_pos": round(self.gear_pos, 3), "gear_cmd_down": int(self.gear_cmd_down),
            "vibe_x": round(vibe[0], 2), "vibe_y": round(vibe[1], 2), "vibe_z": round(vibe[2], 2),
            "clip": getattr(self, "clip", 0) if self.armed else 0,
            "event": self.pending_event,
        })
        self.pending_event = ""

    # ----------------------------------------------------------------- main loop
    def run(self) -> "Sim":
        self.update_estimate()
        self.tgt = np.array([self.est_p[0], self.est_p[1], 0.0])
        self.start_item()
        log_every = int(round(1.0 / (LOG_HZ * DT)))
        step = 0
        while self.t < self.cfg.max_time_s:
            self.update_estimate()
            self.check_failsafes()
            self.guidance()
            self.update_gear()
            tmax = self.veh.tmax_n(self.batt.voltage)
            cmd = self.control(tmax)
            self.physics(cmd)
            # MOT_HOVER_LEARN analogue: slow filter of thrust fraction while hovering steady
            if self.armed and not self.landed and abs(self.est_v[2]) < 0.3 and np.linalg.norm(self.est_v[:2]) < 1.0:
                frac = float(self.T.sum() / (4 * tmax))
                self.thr_hover += (frac - self.thr_hover) * DT / 10.0
            dist = float(np.linalg.norm(self.p[:2]))
            self.max_dist_home = max(self.max_dist_home, dist)
            if step % log_every == 0:
                self.log(tmax)
            # touchdown -> disarm
            if self.armed and self.landed and self.mode in ("LAND", "RTL", "AUTO") and self.t - self.item_t0 > 2.0 \
                    and self.z_mode == "rate" and self.t - self.ground_t > 1.0:
                self.armed = False
                self.event(f"LANDED + DISARMED ({100 * self.batt.soc:.1f} % left)")
                self.log(tmax)
                break
            if self.cfg.stop_at_used_frac is not None and self.batt.used_mah >= self.cfg.stop_at_used_frac * BATT_CAPACITY_MAH:
                self.event(f"Endurance stop at {100 * self.cfg.stop_at_used_frac:.0f} % used")
                self.log(tmax)
                break
            if self.done:
                break
            self.t += DT
            step += 1
        return self

    # ----------------------------------------------------------------- summary
    def summary(self) -> Dict:
        rows = self.rows
        airborne = [r for r in rows if r["armed"] and not r["landed"]]
        xt = [float(r["xtrack_m"]) for r in rows if r["xtrack_m"] != ""]
        hover = [r for r in airborne if r["gnd_speed_ms"] < 0.6 and abs(r["vd_ms"]) < 0.3]
        t_air = (airborne[-1]["t_s"] - airborne[0]["t_s"]) if airborne else 0.0
        return {
            "run_id": self.cfg.run_id,
            "title": self.cfg.title,
            "vehicle": self.veh.name,
            "mass_kg": round(self.veh.mass_kg, 4),
            "inertia_kgm2": [round(x, 5) for x in self.veh.inertia],
            "inertia_source": getattr(self.veh, "inertia_source", ""),
            "thrust_multiplier": self.veh.thrust_mult,
            "wind_mean_ms": self.cfg.wind_mean_ms,
            "flight_time_s": round(t_air, 1),
            "mah_used": round(self.batt.used_mah, 1),
            "batt_remaining_pct_end": round(100 * self.batt.soc, 1),
            "min_voltage_v": round(min(r["voltage_v"] for r in rows), 2),
            "max_current_a": round(max(r["current_a"] for r in rows), 1),
            "hover_current_mean_a": round(float(np.mean([r["current_a"] for r in hover])), 2) if hover else None,
            "hover_thr_out_mean": round(float(np.mean([r["thr_out"] for r in hover])), 4) if hover else None,
            "thr_hover_learned_end": round(self.thr_hover, 4),
            "max_alt_m": round(max(r["alt_true_m"] for r in rows), 2),
            "max_dist_home_m": round(self.max_dist_home, 1),
            "max_lean_deg": round(max(max(abs(r["roll_deg"]), abs(r["pitch_deg"])) for r in airborne), 1) if airborne else 0.0,
            "xtrack_max_m": round(max(xt), 3) if xt else None,
            "xtrack_p95_m": round(float(np.percentile(xt, 95)), 3) if xt else None,
            "landing_pos_err_m": round(float(np.hypot(rows[-1]["true_n_m"], rows[-1]["true_e_m"])), 2),
            "events": self.events,
        }


# ============================================================================
# 7. Run definitions
# ============================================================================
def scripted_hover_loiter(alt_hold_alt: float, loiter_alt: float, hold_s: float, loiter_s: float) -> List[Item]:
    return [
        Item("takeoff", 1, 0.0, 0.0, alt_hold_alt, mode="ALT_HOLD"),
        Item("hold", 2, 0.0, 0.0, alt_hold_alt, hold_s=hold_s, mode="ALT_HOLD", pilot=True),
        Item("hold", 3, 0.0, 0.0, loiter_alt, hold_s=5.0, mode="LOITER"),
        Item("hold", 4, 0.0, 0.0, loiter_alt, hold_s=loiter_s, mode="LOITER"),
        Item("land", 5, 0.0, 0.0, 0.0, mode="LAND"),
    ]


def build_runs(mission: List[Item]) -> List[RunConfig]:
    rogue = list(mission)
    # Operator error: an extra waypoint 150 m north is inserted after the second box corner
    insert_at = next(i for i, it in enumerate(rogue) if it.kind == "wp" and it.seq == 4) + 1
    rogue.insert(insert_at, Item("wp", 99, 150.0, 0.0, 12.0, hold_s=0.0))
    return [
        RunConfig("A", "Stage 1 bare quad: ALT_HOLD 30 s, LOITER 60 s, LAND (2 m/s breeze)", "stage1",
                  scripted_hover_loiter(3.0, 5.0, 30.0, 62.0), wind_mean_ms=2.0, turb_sigma=0.4, seed=1),
        RunConfig("B0", "Stage 2 enclosed: ALT_HOLD 30 s, LOITER 60 s in 15 km/h wind", "stage2",
                  scripted_hover_loiter(3.0, 5.0, 30.0, 62.0), wind_mean_ms=WIND_MISSION_MS, turb_sigma=0.9, seed=2),
        RunConfig("B", "Stage 2 enclosed: full AUTO mission, 15 km/h wind + turbulence", "stage2",
                  mission, wind_mean_ms=WIND_MISSION_MS, turb_sigma=0.9, seed=3),
        RunConfig("B25", "Stage 2 enclosed: same mission at WPNAV_SPEED 2.5 m/s (wind mitigation)", "stage2",
                  mission, wind_mean_ms=WIND_MISSION_MS, turb_sigma=0.9, wp_speed=2.5, seed=3),
        RunConfig("B10", "Stage 2 enclosed: same mission in 10 km/h wind (operating-limit study)", "stage2",
                  mission, wind_mean_ms=10.0 / 3.6, turb_sigma=0.6, seed=3),
        RunConfig("C1", "Failsafe: RC loss at t = 45 s -> RTL", "stage2",
                  mission, wind_mean_ms=WIND_MISSION_MS, turb_sigma=0.9, rc_loss_t=45.0, seed=4),
        RunConfig("C2", "Failsafe: low battery (pack starts at 42 %) -> RTL -> LAND", "stage2",
                  mission, wind_mean_ms=WIND_MISSION_MS, turb_sigma=0.9, soc0=0.42, seed=5),
        RunConfig("C3", "Failsafe: geofence (rogue WP 150 m out, FENCE_RADIUS 120 m) -> RTL", "stage2",
                  rogue, wind_mean_ms=WIND_MISSION_MS, turb_sigma=0.9, seed=6),
        RunConfig("D", "Stage 2 still-air endurance hover to 80 % of pack used", "stage2",
                  [Item("takeoff", 1, 0.0, 0.0, 8.0, mode="LOITER"),
                   Item("hold", 2, 0.0, 0.0, 8.0, hold_s=900.0, mode="LOITER")],
                  stop_at_used_frac=0.80, max_time_s=900.0, failsafes=False, seed=7),
    ]


# ============================================================================
# 8. Plot
# ============================================================================
def make_plot(sims: Dict[str, Sim], mission: List[Item], out_png: Path) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, axs = plt.subplots(2, 2, figsize=(15, 11))
    fig.suptitle("Fig 6 - Autonomous mission & failsafe verification (6-DOF closed-loop sim, 1.905 kg enclosed 5\")",
                 fontsize=13, fontweight="bold")

    # (a) ground track
    ax = axs[0, 0]
    b = sims["B"].rows
    ax.plot([r["true_e_m"] for r in b], [r["true_n_m"] for r in b], color="#1f4e79", lw=1.6, label="Run B mission (true)")
    for key, col in (("C1", "#d62728"), ("C2", "#ff7f0e")):
        rr = sims[key].rows
        ax.plot([r["true_e_m"] for r in rr], [r["true_n_m"] for r in rr], color=col, lw=1.0, ls="--",
                label=f"Run {key} ({sims[key].cfg.title.split(':')[1].split('->')[0].strip()})")
    wps = [it for it in mission if it.kind == "wp"]
    ax.scatter([w.e for w in wps], [w.n for w in wps], s=90, facecolors="none", edgecolors="k", zorder=5, label="Waypoints")
    for w in wps:
        circ = plt.Circle((w.e, w.n), WPNAV_RADIUS, color="green", fill=False, lw=0.8, alpha=0.6)
        ax.add_patch(circ)
        ax.annotate(str(w.seq), (w.e + 1.2, w.n + 1.2), fontsize=8)
    ax.plot(0, 0, marker="*", ms=14, color="gold", mec="k", label="Home / land")
    ax.arrow(-22, 42, 4 * sims["B"].wind_mean[1], 4 * sims["B"].wind_mean[0], width=0.4, color="gray")
    ax.text(-26, 44, "wind 15 km/h", fontsize=8, color="gray")
    ax.set_xlabel("East (m)"); ax.set_ylabel("North (m)")
    ax.set_title(f"(a) Ground track - 6 WPs, 35 x 25 m box, WPNAV_RADIUS {WPNAV_RADIUS:.0f} m circles")
    ax.set_aspect("equal"); ax.grid(alpha=0.3); ax.legend(fontsize=7, loc="lower right")

    # (b) altitude + gear
    ax = axs[0, 1]
    t = [r["t_s"] for r in b]
    ax.plot(t, [r["alt_true_m"] for r in b], color="#1f4e79", label="Altitude (true)")
    ax.plot(t, [r["alt_target_m"] for r in b], color="k", lw=0.8, ls=":", label="Target")
    ax.axhline(LGR_RETRACT_ALT, color="purple", ls="--", lw=0.8, label="LGR_RETRACT_ALT 5 m")
    ax.axhline(LGR_DEPLOY_ALT, color="brown", ls="--", lw=0.8, label="LGR_DEPLOY_ALT 3 m")
    ax2 = ax.twinx()
    ax2.fill_between(t, [r["gear_pos"] for r in b], step="post", alpha=0.15, color="orange", label="Gear down")
    ax2.set_ylim(0, 4); ax2.set_yticks([0, 1]); ax2.set_yticklabels(["up", "down"])
    ax.set_xlabel("Time (s)"); ax.set_ylabel("Altitude AGL (m)")
    ax.set_title("(b) Run B altitude profile & retractable-gear state")
    ax.grid(alpha=0.3); ax.legend(fontsize=7, loc="upper right")

    # (c) battery
    ax = axs[1, 0]
    for key, col in (("B", "#1f4e79"), ("C2", "#ff7f0e"), ("D", "#2ca02c")):
        rr = sims[key].rows
        ax.plot([r["t_s"] for r in rr], [r["batt_remaining_pct"] for r in rr], color=col, label=f"Run {key} remaining %")
    ax.axhline(25, color="red", ls="--", lw=1, label="Land reserve 25 %")
    ax.axhline(100 * BATT_LOW_MAH / BATT_CAPACITY_MAH, color="orange", ls=":", lw=1, label="BATT_LOW_MAH (RTL)")
    ax.axhline(100 * BATT_CRT_MAH / BATT_CAPACITY_MAH, color="darkred", ls=":", lw=1, label="BATT_CRT_MAH (LAND)")
    ax3 = ax.twinx()
    ax3.plot([r["t_s"] for r in b], [r["current_a"] for r in b], color="gray", lw=0.6, alpha=0.7)
    ax3.set_ylabel("Run B current (A)", color="gray")
    ax.set_xlabel("Time (s)"); ax.set_ylabel("Battery remaining (%)")
    ax.set_title("(c) Battery: mission reserve, low-batt failsafe, endurance hover")
    ax.grid(alpha=0.3); ax.legend(fontsize=7, loc="lower left")

    # (d) failsafes: distance from home
    ax = axs[1, 1]
    for key, col in (("C1", "#d62728"), ("C2", "#ff7f0e"), ("C3", "#9467bd")):
        s = sims[key]
        rr = s.rows
        ax.plot([r["t_s"] for r in rr], [math.hypot(r["true_n_m"], r["true_e_m"]) for r in rr], color=col,
                label=s.cfg.title.replace("Failsafe: ", ""))
        for te, msg in s.events:
            if msg.startswith(("FAILSAFE", "FENCE", "MODE")):
                ax.axvline(te, color=col, lw=0.6, ls=":")
                ax.annotate(msg.split("(")[0].strip(), (te, 5 + 10 * list(sims).index(key) % 60), rotation=90,
                            fontsize=6, color=col)
    ax.axhline(FENCE_RADIUS, color="k", ls="--", lw=0.8, label="FENCE_RADIUS 120 m")
    ax.set_xlabel("Time (s)"); ax.set_ylabel("Distance from home (m)")
    ax.set_title("(d) Failsafe responses: RC loss, low battery, geofence")
    ax.grid(alpha=0.3); ax.legend(fontsize=7, loc="upper left")

    fig.tight_layout(rect=[0, 0, 1, 0.96])
    out_png.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_png, dpi=140)
    plt.close(fig)


# ============================================================================
# 9. Main
# ============================================================================
def write_csv(rows: List[Dict], path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as fh:
        w = csv.DictWriter(fh, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[1])
    ap.add_argument("--mission", default=str(ROOT / "missions" / "iiith_5in_enclosed_mission.waypoints"))
    ap.add_argument("--runs", default="A,B0,B,B25,B10,C1,C2,C3,D", help="comma-separated subset of runs")
    args = ap.parse_args()

    mission = load_waypoints(Path(args.mission))
    vehicles = build_vehicles()
    wanted = set(args.runs.split(","))
    log_dir = ROOT / "outputs" / "logs"
    file_names = {
        "A": "stage1_flight_telemetry.csv",
        "B0": "stage2_hover_loiter_telemetry.csv",
        "B": "stage2_mission_telemetry.csv",
        "B25": "stage2_mission_slow_2p5ms_telemetry.csv",
        "B10": "stage2_mission_wind10kmh_telemetry.csv",
        "C1": "stage2_failsafe_rc_loss_telemetry.csv",
        "C2": "stage2_failsafe_low_battery_telemetry.csv",
        "C3": "stage2_failsafe_geofence_telemetry.csv",
        "D": "stage2_endurance_hover_telemetry.csv",
    }

    for v in vehicles.values():
        print(f"Vehicle: {v.name}: m = {v.mass_kg:.4f} kg, I = {v.inertia} kg m^2 "
              f"[{getattr(v, 'inertia_source', '')}], thrust mult = {v.thrust_mult}, "
              f"Tmax @21.5 V = {v.tmax_n(21.5) / GRAVITY_MS2:.3f} kgf")

    sims: Dict[str, Sim] = {}
    summaries = []
    for cfg in build_runs(mission):
        if cfg.run_id not in wanted:
            continue
        sim = Sim(cfg, vehicles[cfg.vehicle]).run()
        sims[cfg.run_id] = sim
        write_csv(sim.rows, log_dir / file_names[cfg.run_id])
        s = sim.summary()
        s["log_file"] = f"outputs/logs/{file_names[cfg.run_id]}"
        summaries.append(s)
        print(f"\n=== Run {cfg.run_id}: {cfg.title} ===")
        for k in ("flight_time_s", "mah_used", "batt_remaining_pct_end", "hover_current_mean_a", "hover_thr_out_mean",
                  "thr_hover_learned_end", "max_current_a", "min_voltage_v", "max_lean_deg", "xtrack_max_m",
                  "xtrack_p95_m", "max_dist_home_m", "landing_pos_err_m"):
            print(f"  {k:24s}: {s[k]}")
        for te, msg in sim.events:
            print(f"    t={te:7.2f}s  {msg}")

    (log_dir / "mission_sim_summary.json").write_text(json.dumps(summaries, indent=2), encoding="utf-8")
    if all(k in sims for k in ("B", "C1", "C2", "C3", "D")):
        make_plot(sims, mission, ROOT / "outputs" / "plots" / "fig6_autonomous_mission_and_failsafes.png")
        print("\nSaved outputs/plots/fig6_autonomous_mission_and_failsafes.png")
    print(f"Saved telemetry CSVs + mission_sim_summary.json in {log_dir}")


if __name__ == "__main__":
    main()
