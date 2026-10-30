"""
Flight-Log Gate Verifier (Gates 4-11) for Project 8: 5-Inch Enclosed Ducted-Fan Quadrotor.

Hardware Sync Gate D: after each Stage 1 / Stage 2 flight pull the DataFlash log off the
SpeedyBee F405 V4 SD card and run

    python src/analysis/flight_log_analyzer.py --log 00000012.BIN [--log 00000013.BIN] [--stage stage2]

Real .BIN logs are read with pymavlink's DFReader (pip install pymavlink). The CSV telemetry
written by src/sitl/mission_simulator.py is read natively, so with no --log the analyzer
verifies the simulated Stage 1 / Stage 2 / failsafe flights.

Both sources are normalised to one table (t_s, mode, armed, alt_m, alt_target_m, n_m, e_m,
current_a, voltage_v, batt_pct, thr_out, thr_hover, gear_down, vibe_x/y/z, clip ...) and the
same gate logic is applied:

  G4  Vibration          1-s mean VIBE X/Y/Z < 15 m/s^2 and no accelerometer clipping
  G5  First hover        hover current & throttle vs sizing prediction (Stage 1 / Stage 2),
  G6  Trim / hover learn learned MOT_THST_HOVER (CTUN.ThH), measured enclosure power loss kappa
  G7  ALT_HOLD           |alt - target| <= 0.5 m after a 5 s settle
  G8  AUTOTUNE           not observable in telemetry; reported if AUTOTUNE mode / ATUN present
  G9  LOITER             >= 60 s hold, radial deviation <= 2.0 m (also reports the 1 m strict reading)
  G10 AUTO + gear        every waypoint passed within 2.0 m; gear up above 5 m in cruise,
                         fully down >= 1 s before touchdown (deploy altitude reported)
  G11 Mission reserve    battery remaining at disarm >= 25 %
  FS  Failsafes          RC loss / low battery / geofence -> RTL or LAND within 1 s, lands < 5 m from home

Outputs: outputs/logs/gate_compliance_report.json, outputs/plots/fig8_flight_log_gate_verification.png
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path
from typing import Dict, List, Optional

import numpy as np
import pandas as pd

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src" / "sizing"))

from sizing_model import (  # noqa: E402
    AVIONICS_POWER_W,
    evaluate_propulsion_point,
    get_locked_5in_bom,
    summarize_bom,
)

M_PER_DEG_LAT = 110674.14
M_PER_DEG_LON = 106231.00
COPTER_MODES = {0: "STABILIZE", 1: "ACRO", 2: "ALT_HOLD", 3: "AUTO", 4: "GUIDED", 5: "LOITER", 6: "RTL",
                7: "CIRCLE", 9: "LAND", 15: "AUTOTUNE", 16: "POSHOLD", 17: "BRAKE", 21: "SMART_RTL"}

VIBE_LIMIT = 15.0
ALT_HOLD_TOL = 0.5
LOITER_RADIUS = 2.0
LOITER_MIN_S = 60.0
WP_TOL = 2.0
GEAR_RETRACT_ALT = 5.0
GEAR_DEPLOY_ALT = 3.0
RESERVE_PCT = 25.0
SETTLE_S = 5.0


# ============================================================================
# 1. Loading
# ============================================================================
def load_sim_csv(path: Path) -> pd.DataFrame:
    d = pd.read_csv(path)
    out = pd.DataFrame({
        "t_s": d["t_s"], "mode": d["mode"], "armed": d["armed"].astype(int),
        "landed": d.get("landed", pd.Series(0, index=d.index)).astype(int),
        "alt_m": d["alt_m"], "alt_target_m": d["alt_target_m"],
        "n_m": d["pos_n_m"], "e_m": d["pos_e_m"],
        "current_a": d["current_a"], "voltage_v": d["voltage_v"], "batt_pct": d["batt_remaining_pct"],
        "thr_out": d["thr_out"], "thr_hover": d["thr_hover_learned"],
        "gear_down": d["gear_pos"], "vibe_x": d["vibe_x"], "vibe_y": d["vibe_y"], "vibe_z": d["vibe_z"],
        "clip": d["clip"], "gnd_speed": d["gnd_speed_ms"], "vd": d["vd_ms"],
        "wp_seq": d["wp_seq"], "xtrack_m": d["xtrack_m"], "event": d["event"].fillna(""),
        "roll_deg": d["roll_deg"], "pitch_deg": d["pitch_deg"],
    })
    meta = {"source": "sim_csv", "run_id": str(d["run_id"].iloc[0]), "stage": str(d["stage"].iloc[0])}
    out.attrs.update(meta)
    return out


def load_bin(path: Path, gear_deployed_pwm: str = "max") -> pd.DataFrame:
    try:
        from pymavlink import DFReader  # type: ignore
    except ImportError as exc:  # pragma: no cover
        raise SystemExit("pymavlink is required for .BIN logs: pip install pymavlink "
                         "(or run setup_external_tools.ps1)") from exc

    log = DFReader.DFReader_binary(str(path), zero_time_base=True)
    want = ["POS", "CTUN", "BAT", "VIBE", "MODE", "RCOU", "LGR", "PARM", "MSG", "GPS", "EV", "ERR", "ATUN", "ATT"]
    streams: Dict[str, List[dict]] = {k: [] for k in want}
    params: Dict[str, float] = {}
    while True:
        m = log.recv_match(type=want)
        if m is None:
            break
        t = m.get_type()
        d = m.to_dict()
        d["t_s"] = getattr(m, "_timestamp", 0.0)
        if t == "PARM":
            params[d.get("Name", "")] = float(d.get("Value", float("nan")))
        elif t == "VIBE" and d.get("IMU", 0) not in (0, None):
            continue
        else:
            streams[t].append(d)
    if not streams["POS"] or not streams["CTUN"]:
        raise SystemExit(f"{path}: no POS/CTUN messages - is LOG_BITMASK set?")

    pos = pd.DataFrame(streams["POS"])
    base = pd.DataFrame({"t_s": pos["t_s"].values})
    lat0, lon0 = pos["Lat"].iloc[0], pos["Lng"].iloc[0]
    base["n_m"] = (pos["Lat"].values - lat0) * M_PER_DEG_LAT
    base["e_m"] = (pos["Lng"].values - lon0) * M_PER_DEG_LON
    sp = np.hypot(np.gradient(base["n_m"]), np.gradient(base["e_m"])) / np.maximum(np.gradient(base["t_s"]), 1e-3)
    base["gnd_speed"] = sp

    def interp(stream: str, col: str, default=np.nan):
        rows = streams[stream]
        if not rows or col not in rows[0]:
            return np.full(len(base), default, dtype=float)
        df = pd.DataFrame(rows)
        return np.interp(base["t_s"], df["t_s"], df[col].astype(float))

    def step(stream: str, col: str, default):
        rows = streams[stream]
        if not rows:
            return [default] * len(base)
        df = pd.DataFrame(rows)
        idx = np.searchsorted(df["t_s"].values, base["t_s"].values, side="right") - 1
        return [df[col].iloc[i] if i >= 0 else default for i in idx]

    base["alt_m"] = interp("CTUN", "Alt")
    base["alt_target_m"] = interp("CTUN", "DAlt")
    base["vd"] = -np.gradient(base["alt_m"]) / np.maximum(np.gradient(base["t_s"]), 1e-3)
    base["thr_out"] = interp("CTUN", "ThO")
    base["thr_hover"] = interp("CTUN", "ThH")
    base["roll_deg"] = interp("ATT", "Roll", 0.0)
    base["pitch_deg"] = interp("ATT", "Pitch", 0.0)
    base["current_a"] = interp("BAT", "Curr")
    base["voltage_v"] = interp("BAT", "Volt")
    cap = params.get("BATT_CAPACITY", 2600.0)
    used = interp("BAT", "CurrTot")
    base["batt_pct"] = interp("BAT", "RemPct") if streams["BAT"] and "RemPct" in streams["BAT"][0] else 100 * (1 - used / cap)
    for ax in ("X", "Y", "Z"):
        base[f"vibe_{ax.lower()}"] = interp("VIBE", f"Vibe{ax}")
    clip_col = "Clip" if streams["VIBE"] and "Clip" in streams["VIBE"][0] else "Clip0"
    base["clip"] = np.gradient(interp("VIBE", clip_col, 0.0)) > 0
    base["mode"] = [COPTER_MODES.get(int(m), str(m)) for m in step("MODE", "ModeNum", 0)]
    # Armed: EV id 10 = ARMED, 11 = DISARMED
    ev = pd.DataFrame(streams["EV"]) if streams["EV"] else pd.DataFrame(columns=["t_s", "Id"])
    armed = np.zeros(len(base), dtype=int)
    for _, r in ev.iterrows():
        if r["Id"] == 10:
            armed[base["t_s"] >= r["t_s"]] = 1
        elif r["Id"] == 11:
            armed[base["t_s"] >= r["t_s"]] = 0
    if not armed.any():
        armed[:] = 1
    base["armed"] = armed
    base["landed"] = ((base["alt_m"] < 0.3) | (armed == 0)).astype(int)

    # Landing gear: LGR log message if present, else the RCOU channel whose SERVOn_FUNCTION = 29
    if streams["LGR"]:
        lg = pd.DataFrame(streams["LGR"])
        col = "LandingGear" if "LandingGear" in lg.columns else lg.columns[1]
        # AP_LandingGear::LG_LandingGear_State: 0 = deployed, 1 = retracted (verify on first log)
        base["gear_down"] = 1.0 - np.interp(base["t_s"], lg["t_s"], (lg[col].astype(float) >= 1).astype(float))
    else:
        ch = next((n for n in range(1, 17) if params.get(f"SERVO{n}_FUNCTION") == 29), None)
        if ch is not None:
            pwm = interp("RCOU", f"C{ch}")
            lo, hi = params.get(f"SERVO{ch}_MIN", 1100), params.get(f"SERVO{ch}_MAX", 1900)
            frac = np.clip((pwm - lo) / max(hi - lo, 1), 0, 1)
            base["gear_down"] = frac if gear_deployed_pwm == "max" else 1 - frac
        else:
            base["gear_down"] = np.nan
    base["wp_seq"] = np.nan
    base["xtrack_m"] = np.nan
    msgs = pd.DataFrame(streams["MSG"]) if streams["MSG"] else pd.DataFrame(columns=["t_s", "Message"])
    base["event"] = ""
    for _, r in msgs.iterrows():
        i = int(np.searchsorted(base["t_s"].values, r["t_s"]))
        if i < len(base):
            base.at[i, "event"] = (base.at[i, "event"] + "; " if base.at[i, "event"] else "") + str(r["Message"])
    base.attrs.update({"source": "dataflash_bin", "run_id": path.stem, "stage": "unknown",
                       "params": {k: v for k, v in params.items() if k.startswith(("MOT_", "BATT_", "FENCE_", "LGR_", "WPNAV_"))},
                       "autotune_seen": bool(streams["ATUN"]) or "AUTOTUNE" in set(base["mode"])})
    return base


def load_mission_wps(path: Path) -> List[Dict]:
    if not path.exists():
        return []
    wps = []
    for ln in path.read_text(encoding="utf-8").strip().splitlines()[1:]:
        f = ln.split("\t")
        seq, cmd, lat, lon = int(f[0]), int(f[3]), float(f[8]), float(f[9])
        if cmd in (16, 19) and seq > 0:
            wps.append({"seq": seq, "n": (lat - 17.4455) * M_PER_DEG_LAT, "e": (lon - 78.3489) * M_PER_DEG_LON})
    return wps


# ============================================================================
# 2. Segment helpers
# ============================================================================
def airborne(d: pd.DataFrame) -> pd.DataFrame:
    return d[(d["armed"] == 1) & (d["landed"] == 0) & (d["alt_m"] > 0.5)]


def mode_segments(d: pd.DataFrame, mode: str) -> List[pd.DataFrame]:
    m = (d["mode"] == mode).astype(int).values
    edges = np.flatnonzero(np.diff(np.r_[0, m, 0]))
    return [d.iloc[a:b] for a, b in zip(edges[::2], edges[1::2])]


def verdict(ok: Optional[bool]) -> str:
    return "N/A" if ok is None else ("PASS" if ok else "FAIL")


# ============================================================================
# 3. Gates
# ============================================================================
def gate4_vibration(d: pd.DataFrame) -> Dict:
    a = airborne(d)
    if a.empty:
        return {"verdict": "N/A"}
    dt = float(np.median(np.diff(a["t_s"]))) or 0.1
    win = max(1, int(round(1.0 / dt)))
    peaks = {ax: float(a[f"vibe_{ax}"].rolling(win, min_periods=1).mean().max()) for ax in "xyz"}
    clips = int(a["clip"].astype(int).sum())
    return {"verdict": verdict(max(peaks.values()) < VIBE_LIMIT and clips == 0),
            "max_1s_mean_vibe_ms2": {k: round(v, 2) for k, v in peaks.items()},
            "median_vibe_ms2": {ax: round(float(a[f"vibe_{ax}"].median()), 2) for ax in "xyz"},
            "clip_samples": clips, "limit_ms2": VIBE_LIMIT}


def hover_samples(d: pd.DataFrame) -> pd.DataFrame:
    a = airborne(d)
    return a[(a["gnd_speed"] < 0.6) & (a["vd"].abs() < 0.3) & a["mode"].isin(["LOITER", "ALT_HOLD", "AUTO", "POSHOLD"])]


def gate56_hover(d: pd.DataFrame, stage: str) -> Dict:
    h = hover_samples(d)
    if h.empty:
        return {"verdict": "N/A"}
    i_raw = float(h["current_a"].mean())
    v_meas = float(h["voltage_v"].mean())
    # Holding station in wind needs T/cos(tilt) per rotor; momentum theory P ~ T^1.5, so
    # normalise to an equivalent level hover (conservative: ignores translational lift).
    tilt = np.radians(np.hypot(h["roll_deg"].fillna(0), h["pitch_deg"].fillna(0)))
    lean_corr = float(np.mean(np.cos(tilt) ** 1.5))
    i_meas = i_raw * lean_corr
    legs = summarize_bom(get_locked_5in_bom(True), stage1_only=(stage == "stage1"))["all_up_mass_g"]
    pred = evaluate_propulsion_point("gate", legs)
    p_rotor_meas = i_meas * v_meas - AVIONICS_POWER_W
    p_rotor_pred0 = 4 * pred.elec_power_per_motor_0pct_w
    kappa = p_rotor_meas / p_rotor_pred0 - 1.0
    hi = pred.hover_current_20pct_a if stage == "stage2" else pred.hover_current_0pct_a
    ok = i_meas <= 1.10 * hi * (22.2 / v_meas)
    return {"verdict": verdict(ok),
            "mass_g_assumed": legs,
            "hover_current_measured_raw_a": round(i_raw, 2),
            "mean_hover_tilt_deg": round(float(np.degrees(tilt).mean()), 1),
            "hover_current_measured_a": round(i_meas, 2),
            "hover_voltage_measured_v": round(v_meas, 2),
            "hover_current_pred_0pct_a": pred.hover_current_0pct_a,
            "hover_current_pred_20pct_a": pred.hover_current_20pct_a,
            "hover_power_vs_ideal_duct_model_kappa_pct": round(100 * kappa, 1),
            "thr_out_hover_mean": round(float(h["thr_out"].mean()), 3),
            "mot_thst_hover_learned_end": round(float(d["thr_hover"].dropna().iloc[-1]), 3),
            "criterion": "lean-corrected hover current <= 1.10 x model (+20 % box loss for Stage 2, 0 % for Stage 1), "
                         "scaled to 22.2 V"}


def gate7_alt_hold(d: pd.DataFrame) -> Dict:
    segs = [s for s in mode_segments(d, "ALT_HOLD") if s["t_s"].iloc[-1] - s["t_s"].iloc[0] > SETTLE_S + 5]
    source = "ALT_HOLD"
    if not segs:
        segs = [s for s in mode_segments(d, "LOITER") if s["t_s"].iloc[-1] - s["t_s"].iloc[0] > SETTLE_S + 5]
        source = "LOITER (no ALT_HOLD segment)"
    if not segs:
        return {"verdict": "N/A"}
    errs = []
    for s in segs:
        # Only grade samples whose altitude target has been steady for SETTLE_S (excludes climbs/steps)
        tgt = s["alt_target_m"].values
        t = s["t_s"].values
        last_change = np.maximum.accumulate(np.where(np.r_[True, np.abs(np.diff(tgt)) > 0.02], t, -np.inf))
        s = s[(t - last_change >= SETTLE_S) & (t > t[0] + SETTLE_S)]
        if not s.empty:
            errs.append((s["alt_m"] - s["alt_target_m"]).values)
    if not errs:
        return {"verdict": "N/A", "note": "no steady altitude-hold portion"}
    e = np.concatenate(errs)
    return {"verdict": verdict(float(np.abs(e).max()) <= ALT_HOLD_TOL), "segment_source": source,
            "max_abs_err_m": round(float(np.abs(e).max()), 3), "rms_err_m": round(float(np.sqrt(np.mean(e ** 2))), 3),
            "tolerance_m": ALT_HOLD_TOL, "_err": e}


def gate9_loiter(d: pd.DataFrame) -> Dict:
    segs = [s for s in mode_segments(d, "LOITER") if s["t_s"].iloc[-1] - s["t_s"].iloc[0] >= LOITER_MIN_S + SETTLE_S - 1]
    if not segs:
        return {"verdict": "N/A", "note": f"no LOITER segment >= {LOITER_MIN_S:.0f} s"}
    s = max(segs, key=len)
    s = s[s["t_s"] > s["t_s"].iloc[0] + SETTLE_S]
    s = s[s["alt_m"] > 1.0]
    cn, ce = float(s["n_m"].median()), float(s["e_m"].median())
    r = np.hypot(s["n_m"] - cn, s["e_m"] - ce)
    dur = float(s["t_s"].iloc[-1] - s["t_s"].iloc[0])
    return {"verdict": verdict(dur >= LOITER_MIN_S - 1 and float(r.max()) <= LOITER_RADIUS),
            "duration_s": round(dur, 1), "max_radius_m": round(float(r.max()), 3),
            "p95_radius_m": round(float(np.percentile(r, 95)), 3),
            "within_1m_radius_strict": bool(float(r.max()) <= 1.0),
            "radius_limit_m": LOITER_RADIUS, "_n": (s["n_m"] - cn).values, "_e": (s["e_m"] - ce).values}


def gate10_auto_gear(d: pd.DataFrame, wps: List[Dict]) -> Dict:
    auto = d[(d["mode"] == "AUTO") & (d["armed"] == 1)]
    if auto.empty or not wps:
        return {"verdict": "N/A"}
    arrivals = []
    for w in wps:
        dist = np.hypot(auto["n_m"] - w["n"], auto["e_m"] - w["e"])
        arrivals.append({"seq": w["seq"], "min_dist_m": round(float(dist.min()), 3)})
    wp_ok = all(a["min_dist_m"] <= WP_TOL for a in arrivals)
    xt = pd.to_numeric(auto["xtrack_m"], errors="coerce").dropna()
    res = {"waypoint_arrivals": arrivals, "waypoints_ok": wp_ok, "wp_tolerance_m": WP_TOL}
    if not xt.empty:
        res.update({"xtrack_p95_m": round(float(np.percentile(xt, 95)), 3), "xtrack_max_m": round(float(xt.max()), 3),
                    "xtrack_note": "cross-track is informational; the graded criterion is waypoint passage <= 2 m"})
    gear_ok = None
    if d["gear_down"].notna().any():
        a = airborne(d)
        cruise = a[a["alt_m"] > GEAR_RETRACT_ALT + 1.0]
        cruise = cruise[cruise["t_s"] > cruise["t_s"].iloc[0] + 3.0] if not cruise.empty else cruise
        up_in_cruise = float((cruise["gear_down"] < 0.05).mean()) if not cruise.empty else float("nan")
        td = d[(d["armed"] == 1) & (d["landed"] == 1)]
        last_air = a["t_s"].iloc[-1] if not a.empty else d["t_s"].iloc[-1]
        before_td = d[(d["t_s"] >= last_air - 1.0) & (d["t_s"] <= last_air)]
        down_at_td = bool((before_td["gear_down"] > 0.98).all()) if not before_td.empty else False
        # deploy start: last time gear left 'up' while descending
        g = d["gear_down"].values
        starts = np.flatnonzero((g[1:] > 0.02) & (g[:-1] <= 0.02))
        deploy_alt = round(float(d["alt_m"].iloc[starts[-1] + 1]), 2) if len(starts) else None
        retract_rows = np.flatnonzero((g[1:] < 0.98) & (g[:-1] >= 0.98))
        retract_alt = round(float(d["alt_m"].iloc[retract_rows[0] + 1]), 2) if len(retract_rows) else None
        gear_ok = bool(up_in_cruise > 0.95 and down_at_td)
        res.update({"gear_up_fraction_in_cruise": round(up_in_cruise, 3), "gear_down_1s_before_touchdown": down_at_td,
                    "gear_deploy_start_alt_m": deploy_alt, "gear_retract_start_alt_m": retract_alt,
                    "gear_rule": f"retract > {GEAR_RETRACT_ALT} m, deploy < {GEAR_DEPLOY_ALT} m (LGR_*), "
                                 "or earlier by mission MAV_CMD_AIRFRAME_CONFIGURATION"})
    res["verdict"] = verdict(wp_ok and (gear_ok is not False))
    return res


def gate11_reserve(d: pd.DataFrame) -> Dict:
    armed = d[d["armed"] == 1]
    if armed.empty:
        return {"verdict": "N/A"}
    end = float(armed["batt_pct"].iloc[-1])
    return {"verdict": verdict(end >= RESERVE_PCT), "battery_at_disarm_pct": round(end, 1), "limit_pct": RESERVE_PCT,
            "min_voltage_v": round(float(armed["voltage_v"].min()), 2),
            "flight_time_s": round(float(armed["t_s"].iloc[-1] - armed["t_s"].iloc[0]), 1)}


def gate_failsafe(d: pd.DataFrame) -> Dict:
    ev = d[d["event"].str.contains("FAILSAFE|FENCE", case=False, regex=True)]
    if ev.empty:
        return {"verdict": "N/A"}
    t0 = float(ev["t_s"].iloc[0])
    after = d[(d["t_s"] >= t0) & (d["t_s"] <= t0 + 1.0)]
    reacted = bool(after["mode"].isin(["RTL", "LAND", "SMART_RTL"]).any())
    last = d.iloc[-1]
    land_err = float(math.hypot(last["n_m"], last["e_m"]))
    return {"verdict": verdict(reacted and land_err < 5.0 and int(last["armed"]) == 0),
            "trigger": ev["event"].iloc[0], "trigger_t_s": t0, "mode_after_1s": list(dict.fromkeys(after["mode"]))[-1],
            "landing_dist_from_home_m": round(land_err, 2), "disarmed": int(last["armed"]) == 0,
            "battery_at_disarm_pct": round(float(d[d["armed"] == 1]["batt_pct"].iloc[-1]), 1),
            "all_events": [e for e in d["event"] if e]}


def autotune_gate(d: pd.DataFrame) -> Dict:
    seen = d.attrs.get("autotune_seen", "AUTOTUNE" in set(d["mode"]))
    if seen:
        return {"verdict": "INFO", "note": "AUTOTUNE flown in this log; check ATUN messages and that gains were saved"}
    return {"verdict": "N/A", "note": "Gate 8 (AUTOTUNE) is flown by the pilot; not observable in this log"}


# ============================================================================
# 4. Driver
# ============================================================================
def evaluate(d: pd.DataFrame, stage: str, wps: List[Dict]) -> Dict:
    fs = gate_failsafe(d)
    aborted = {"verdict": "N/A", "note": "mission intentionally aborted by an injected failsafe; see FS gate"}
    gates = {
        "G4_vibration": gate4_vibration(d),
        "G5_G6_hover_and_trim": gate56_hover(d, stage),
        "G7_alt_hold": gate7_alt_hold(d),
        "G8_autotune": autotune_gate(d),
        "G9_loiter": gate9_loiter(d),
        "G10_auto_waypoints_and_gear": aborted if fs["verdict"] != "N/A" else gate10_auto_gear(d, wps),
        "G11_mission_battery_reserve": aborted if fs["verdict"] != "N/A" else (
            gate11_reserve(d) if (d["mode"] == "AUTO").any() else {"verdict": "N/A"}),
        "FS_failsafe_response": fs,
    }
    return gates


def strip_private(o):
    if isinstance(o, dict):
        return {k: strip_private(v) for k, v in o.items() if not k.startswith("_")}
    if isinstance(o, list):
        return [strip_private(v) for v in o]
    if isinstance(o, (np.bool_,)):
        return bool(o)
    if isinstance(o, (np.integer,)):
        return int(o)
    if isinstance(o, (np.floating,)):
        return float(o)
    return o


def make_plot(results: Dict[str, Dict], frames: Dict[str, pd.DataFrame], out_png: Path) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    def first(stage: str, pred=lambda k: True):
        return next((k for k, r in results.items() if r["stage"] == stage and pred(k)), None)

    s1 = first("stage1")
    s2h = next((k for k, r in results.items() if r["stage"] == "stage2" and r["gates"]["G9_loiter"]["verdict"] != "N/A"), None)
    s2m = next((k for k, r in results.items() if r["stage"] == "stage2" and r["gates"]["G10_auto_waypoints_and_gear"]["verdict"] != "N/A"
                and r["gates"]["FS_failsafe_response"]["verdict"] == "N/A"), None)

    fig, axs = plt.subplots(2, 3, figsize=(18, 10.5))
    fig.suptitle("Fig 8 - Flight-log gate verification (Gates 4-11)  [source: " +
                 ", ".join(sorted({frames[k].attrs.get('source', '?') for k in frames})) + "]",
                 fontsize=13, fontweight="bold")

    ax = axs[0, 0]
    if s2m:
        d = airborne(frames[s2m])
        for ax_, c in zip("xyz", ("#1f77b4", "#2ca02c", "#d62728")):
            ax.plot(d["t_s"], d[f"vibe_{ax_}"].rolling(10, min_periods=1).mean(), color=c, lw=1, label=f"VIBE {ax_.upper()}")
        ax.axhline(VIBE_LIMIT, color="k", ls="--", label="15 m/s^2 limit")
        ax.set_title(f"(a) G4 vibration ({s2m}): {results[s2m]['gates']['G4_vibration']['verdict']}")
        ax.legend(fontsize=7)
    ax.set(xlabel="Time (s)", ylabel="Vibration (m/s^2)")

    ax = axs[0, 1]
    labels, meas, p0, p20 = [], [], [], []
    for k in (s1, s2h):
        if k:
            g = results[k]["gates"]["G5_G6_hover_and_trim"]
            if g["verdict"] != "N/A":
                labels.append(f"{results[k]['stage']}\n({k})")
                meas.append(g["hover_current_measured_a"])
                p0.append(g["hover_current_pred_0pct_a"])
                p20.append(g["hover_current_pred_20pct_a"])
    x = np.arange(len(labels))
    ax.bar(x - 0.27, p0, 0.27, color="#c6dbef", label="model, 0 % loss")
    ax.bar(x, p20, 0.27, color="#6baed6", label="model, +20 % loss")
    ax.bar(x + 0.27, meas, 0.27, color="#08519c", label="log / sim")
    for xi, v in zip(x, meas):
        ax.text(xi + 0.27, v + 0.3, f"{v:.1f} A", ha="center", fontsize=8)
    ax.set_xticks(x)
    ax.set_xticklabels(labels, fontsize=8)
    ax.set(ylabel="Hover current (A)", title="(b) G5/G6 hover current: Stage 1 vs Stage 2 (enclosure cost)")
    ax.legend(fontsize=7)

    ax = axs[0, 2]
    for k, c in ((s1, "#2ca02c"), (s2h, "#1f4e79")):
        if k and "_err" in results[k]["gates"]["G7_alt_hold"]:
            e = results[k]["gates"]["G7_alt_hold"]["_err"]
            ax.plot(np.arange(len(e)) / 10.0, e, color=c, lw=1,
                    label=f"{results[k]['stage']}: max {np.abs(e).max():.2f} m ({results[k]['gates']['G7_alt_hold']['verdict']})")
    ax.axhspan(-ALT_HOLD_TOL, ALT_HOLD_TOL, color="green", alpha=0.08)
    ax.axhline(ALT_HOLD_TOL, color="green", ls="--", lw=0.8)
    ax.axhline(-ALT_HOLD_TOL, color="green", ls="--", lw=0.8)
    ax.set(xlabel="Time in hold (s)", ylabel="Altitude error (m)", title="(c) G7 ALT_HOLD +/- 0.5 m")
    ax.legend(fontsize=7)

    ax = axs[1, 0]
    for k, c in ((s1, "#2ca02c"), (s2h, "#1f4e79")):
        if k and "_n" in results[k]["gates"]["G9_loiter"]:
            g = results[k]["gates"]["G9_loiter"]
            ax.plot(g["_e"], g["_n"], ".", ms=2, color=c,
                    label=f"{results[k]['stage']}: max r {g['max_radius_m']:.2f} m ({g['verdict']})")
    for r, ls in ((1.0, ":"), (LOITER_RADIUS, "--")):
        ax.add_patch(plt.Circle((0, 0), r, fill=False, ls=ls, color="k"))
    ax.set_xlim(-2.6, 2.6); ax.set_ylim(-2.6, 2.6); ax.set_aspect("equal")
    ax.set(xlabel="East dev (m)", ylabel="North dev (m)", title="(d) G9 LOITER 60 s: 1 m and 2 m circles")
    ax.legend(fontsize=7, loc="upper right")

    ax = axs[1, 1]
    if s2m:
        g = results[s2m]["gates"]["G10_auto_waypoints_and_gear"]
        seqs = [a["seq"] for a in g["waypoint_arrivals"]]
        dist = [a["min_dist_m"] for a in g["waypoint_arrivals"]]
        ax.bar([str(s) for s in seqs], dist, color=["#2ca02c" if v <= WP_TOL else "#d62728" for v in dist])
        ax.axhline(WP_TOL, color="k", ls="--", label="2 m tolerance")
        ax.set_title(f"(e) G10 waypoint passage ({s2m}): {g['verdict']}; xtrack p95 {g.get('xtrack_p95_m', float('nan')):.2f} m")
        ax.legend(fontsize=7)
    ax.set(xlabel="Mission seq", ylabel="Closest approach (m)")

    ax = axs[1, 2]
    if s2m:
        d = frames[s2m]
        ax.plot(d["t_s"], d["batt_pct"], color="#1f4e79", label="battery %")
        ax.axhline(RESERVE_PCT, color="r", ls="--", label="25 % reserve")
        ax2 = ax.twinx()
        ax2.plot(d["t_s"], d["alt_m"], color="gray", lw=0.8, label="alt (m)")
        ax2.fill_between(d["t_s"], 0, d["gear_down"] * 3, color="orange", alpha=0.25, label="gear down (x3)")
        ax2.axhline(GEAR_RETRACT_ALT, color="purple", ls=":", lw=0.8)
        ax2.axhline(GEAR_DEPLOY_ALT, color="brown", ls=":", lw=0.8)
        ax2.set_ylabel("Altitude (m)")
        g11 = results[s2m]["gates"]["G11_mission_battery_reserve"]
        ax.set_title(f"(f) G10 gear + G11 reserve: {g11['battery_at_disarm_pct']} % at disarm ({g11['verdict']})")
        ax.legend(fontsize=7, loc="lower left")
        ax2.legend(fontsize=7, loc="upper right")
    ax.set(xlabel="Time (s)", ylabel="Battery remaining (%)")

    for a in axs.flat:
        a.grid(alpha=0.3)
    fig.tight_layout(rect=[0, 0, 1, 0.95])
    out_png.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_png, dpi=140)
    plt.close(fig)


def main() -> None:
    ap = argparse.ArgumentParser(description="Gate 4-11 flight-log verifier")
    ap.add_argument("--log", action="append", help=".BIN DataFlash or simulator .csv (repeatable)")
    ap.add_argument("--stage", default=None, help="stage1 | stage2 for .BIN logs (default: guess from hover throttle)")
    ap.add_argument("--mission", default=str(ROOT / "missions" / "iiith_5in_enclosed_mission.waypoints"))
    ap.add_argument("--gear-deployed-pwm", choices=["max", "min"], default="max",
                    help="which SERVOn end-point is 'deployed' when no LGR message is logged")
    args = ap.parse_args()

    logs = [Path(p) for p in args.log] if args.log else [
        ROOT / "outputs" / "logs" / n for n in (
            "stage1_flight_telemetry.csv", "stage2_hover_loiter_telemetry.csv", "stage2_mission_telemetry.csv",
            "stage2_failsafe_rc_loss_telemetry.csv", "stage2_failsafe_low_battery_telemetry.csv",
            "stage2_failsafe_geofence_telemetry.csv", "stage2_endurance_hover_telemetry.csv")]
    wps = load_mission_wps(Path(args.mission))

    results: Dict[str, Dict] = {}
    frames: Dict[str, pd.DataFrame] = {}
    for p in logs:
        if not p.exists():
            print(f"  skip {p} (not found)")
            continue
        d = load_bin(p, args.gear_deployed_pwm) if p.suffix.lower() == ".bin" else load_sim_csv(p)
        stage = args.stage or d.attrs.get("stage", "unknown")
        if stage == "unknown":
            th = hover_samples(d)["thr_out"].mean()
            stage = "stage2" if th > 0.27 else "stage1"
        key = d.attrs.get("run_id", p.stem)
        frames[key] = d
        results[key] = {"log": str(p.relative_to(ROOT) if p.is_relative_to(ROOT) else p), "stage": stage,
                        "source": d.attrs.get("source"), "gates": evaluate(d, stage, wps)}

    print("\n=== Gate compliance ===")
    names = ["G4_vibration", "G5_G6_hover_and_trim", "G7_alt_hold", "G8_autotune", "G9_loiter",
             "G10_auto_waypoints_and_gear", "G11_mission_battery_reserve", "FS_failsafe_response"]
    print(f"{'log':6s} {'stage':7s} " + " ".join(f"{n.split('_')[0]:>5s}" for n in names))
    for k, r in results.items():
        print(f"{k:6s} {r['stage']:7s} " + " ".join(f"{r['gates'][n]['verdict']:>5s}" for n in names))

    overall = {}
    for n in names:
        vs = [r["gates"][n]["verdict"] for r in results.values() if r["gates"][n]["verdict"] in ("PASS", "FAIL")]
        overall[n] = "N/A" if not vs else ("PASS" if all(v == "PASS" for v in vs) else "FAIL")
    print("\nOverall:", overall)

    out_json = ROOT / "outputs" / "logs" / "gate_compliance_report.json"
    out_json.write_text(json.dumps(strip_private({"overall": overall, "logs": results}), indent=2), encoding="utf-8")
    make_plot(results, frames, ROOT / "outputs" / "plots" / "fig8_flight_log_gate_verification.png")
    print(f"Saved {out_json} and outputs/plots/fig8_flight_log_gate_verification.png")


if __name__ == "__main__":
    main()
