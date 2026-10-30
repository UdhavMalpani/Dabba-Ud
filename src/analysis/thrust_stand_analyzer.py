"""
Stage 0 Thrust-Stand Analyzer for Project 8: 5-Inch Enclosed Ducted-Fan Quadrotor.

Hardware Sync Gate B: run one Emax ECO II 2207 1700KV + 5x4.3 tri-blade on the lab stand
(firmware/thrust_stand_logger/thrust_stand_logger.ino) in five configurations, then

    python src/analysis/thrust_stand_analyzer.py --csv <serial_log.csv> [--csv <more.csv>]

With no --csv it first writes a model-based synthetic dataset
(outputs/logs/stage0_thrust_stand_sweep.csv) built from the same physics as the sizing
model (momentum theory + duct_aero.py tip-gap/grille model) so the whole pipeline can be
exercised before hardware arrives. Synthetic results are predictions, not measurements.

Per configuration it computes
  C_T = T / (rho n^2 D^4),  C_P = P_shaft / (rho n^3 D^5),  FM = C_T^1.5 / (sqrt(2) C_P)
  with P_shaft = eta_motor * P_elec (eta_motor = 0.80 assumed, same as the sizing model)
  usable max thrust  T_use = T_max,meas * (rho_site / rho_test) * (21.5 V / V_at_max)^2
  duct gain at equal electrical power vs the open 5x4.3 prop (at the hover power point)
  hover point at T_h = m g / 4 for 1,825 g (skids) and 1,905 g (legs):
      electrical power, current, throttle, predicted MOT_THST_HOVER, T/W, hover time
and applies the Slide 6 decision gate (>= 1.19 kgf legs, >= 1.14 kgf skids, ...).

Outputs: outputs/logs/stage0_thrust_stand_sweep.csv (synthetic only),
         outputs/sizing/stage0_thrust_stand_results.json,
         outputs/plots/fig7_stage0_thrust_stand_results.png
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import sys
from pathlib import Path
from typing import Dict, List

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src" / "sizing"))

from duct_aero import evaluate_duct_aero, get_locked_5in_geometry  # noqa: E402
from sizing_model import (  # noqa: E402
    AVIONICS_POWER_W,
    GRAVITY_MS2,
    RHO_HYD_KGM3,
    RHO_SL_KGM3,
    USABLE_PACK_FRAC,
    get_locked_5in_bom,
    summarize_bom,
)

PROP_D_M = 0.127
DISC_AREA_M2 = math.pi * PROP_D_M ** 2 / 4.0
ETA_MOTOR = 0.80
V_SAG_FLIGHT = 21.5
V_NOM = 22.2
PACK_AH = 2.6
CSV_COLUMNS = ["config", "tip_gap_mm", "pwm_us", "throttle_pct", "thrust_g", "voltage_v", "current_a", "power_w", "rpm"]

GATE_LEGS_KGF = 1.19
GATE_SKIDS_KGF = 1.14
GATE_TRIM_KGF = 1.10
GATE_MIN_KGF = 0.95


# ============================================================================
# 1. Synthetic dataset (model prediction standing in for the real stand)
# ============================================================================
def _synthetic_config(label: str, gap: float, t_ref_kgf: float, ct: float, fm_hi: float,
                      duct_mult: float, rng: np.random.Generator) -> List[Dict]:
    """t_ref_kgf: open-prop max thrust at 24.0 V in Hyderabad air. duct_mult: thrust at equal power."""
    rows = []
    v_ocv, r_pack = 24.9, 0.022
    for pwm in range(1000, 2001, 50):
        u = (pwm - 1000) / 1000.0
        f = 0.35 * u + 0.65 * u * u          # thrust curve shape (MOT_THST_EXPO 0.65)
        if u < 0.04:
            rows.append({"config": label, "tip_gap_mm": gap, "pwm_us": pwm, "throttle_pct": round(100 * u, 1),
                         "thrust_g": round(rng.normal(0, 1.5), 1), "voltage_v": v_ocv, "current_a": 0.25,
                         "power_w": round(0.25 * v_ocv, 2), "rpm": 0})
            continue
        v = v_ocv
        for _ in range(6):                    # thrust depends on sag, sag on current
            t_open_n = t_ref_kgf * GRAVITY_MS2 * f * (v / 24.0) ** 2
            fm = 0.47 + (fm_hi - 0.47) * (min(f, 1.0)) ** 0.3
            p_open = t_open_n ** 1.5 / math.sqrt(2 * RHO_HYD_KGM3 * DISC_AREA_M2) / (fm * ETA_MOTOR)
            p_elec = p_open + 2.0              # ESC / bearing tare
            i = p_elec / v
            v = v_ocv - i * r_pack
        t_n = t_open_n * duct_mult            # same electrical power, duct changes thrust
        n_rps = math.sqrt(t_open_n / (ct * RHO_HYD_KGM3 * PROP_D_M ** 4)) * (0.985 if duct_mult != 1.0 else 1.0)
        rows.append({
            "config": label, "tip_gap_mm": gap, "pwm_us": pwm, "throttle_pct": round(100 * u, 1),
            "thrust_g": round(t_n / GRAVITY_MS2 * 1000 * (1 + rng.normal(0, 0.006)) + rng.normal(0, 2.0), 1),
            "voltage_v": round(v + rng.normal(0, 0.02), 3),
            "current_a": round(i + rng.normal(0, 0.12), 3),
            "power_w": round(v * i, 2),
            "rpm": round(n_rps * 60 * (1 + rng.normal(0, 0.004))),
        })
    return rows


def generate_synthetic_csv(path: Path, seed: int = 0) -> Path:
    rng = np.random.default_rng(seed)
    t_open_24v = 1.98 * RHO_HYD_KGM3 / RHO_SL_KGM3  # 1.762 kgf: datasheet scaled to 540 m / 30 C
    configs = [
        ("open_5x4.3_3blade", float("nan"), t_open_24v, 0.165, 0.63, 1.0),
        ("open_5x4.5_2blade", float("nan"), 1.60 * RHO_HYD_KGM3 / RHO_SL_KGM3, 0.120, 0.66, 1.0),
    ]
    for gap in (1.0, 1.5, 2.0):
        perf = evaluate_duct_aero(get_locked_5in_geometry(gap), include_grilles=True, include_enclosurebox=False)
        configs.append((f"duct_grilles_gap{gap:.1f}", gap, t_open_24v, 0.165, 0.63, perf.net_thrust_multiplier))
    rows: List[Dict] = []
    for label, gap, tref, ct, fm_hi, mult in configs:
        rows += _synthetic_config(label, gap, tref, ct, fm_hi, mult, rng)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as fh:
        fh.write("# SYNTHETIC Stage 0 dataset generated by thrust_stand_analyzer.py (model prediction, not measured)\n")
        w = csv.DictWriter(fh, fieldnames=CSV_COLUMNS)
        w.writeheader()
        w.writerows(rows)
    return path


# ============================================================================
# 2. Parsing
# ============================================================================
def read_stand_csv(paths: List[Path]) -> Dict[str, Dict[str, np.ndarray]]:
    data: Dict[str, Dict[str, list]] = {}
    for p in paths:
        lines = [ln for ln in p.read_text(encoding="utf-8", errors="replace").splitlines()
                 if ln.strip() and not ln.lstrip().startswith("#")]
        reader = csv.DictReader(lines)
        missing = set(CSV_COLUMNS) - set(reader.fieldnames or [])
        if missing:
            raise ValueError(f"{p}: missing columns {sorted(missing)}")
        for r in reader:
            if r["config"] == "config":     # repeated header from a re-run
                continue
            d = data.setdefault(r["config"], {k: [] for k in CSV_COLUMNS[1:]})
            for k in CSV_COLUMNS[1:]:
                try:
                    d[k].append(float(r[k]))
                except (TypeError, ValueError):
                    d[k].append(float("nan"))
    return {c: {k: np.array(v) for k, v in d.items()} for c, d in data.items()}


# ============================================================================
# 3. Analysis
# ============================================================================
def analyze_config(name: str, d: Dict[str, np.ndarray], rho_test: float, hover_n: Dict[str, float]) -> Dict:
    order = np.argsort(d["pwm_us"])
    d = {k: v[order] for k, v in d.items()}
    t_n = d["thrust_g"] / 1000.0 * GRAVITY_MS2
    p_e = np.where(np.isfinite(d["power_w"]), d["power_w"], d["voltage_v"] * d["current_a"])
    n = d["rpm"] / 60.0
    ok = (n > 50) & (t_n > 0.2)
    ct = np.full_like(t_n, np.nan)
    cp = np.full_like(t_n, np.nan)
    ct[ok] = t_n[ok] / (rho_test * n[ok] ** 2 * PROP_D_M ** 4)
    cp[ok] = ETA_MOTOR * p_e[ok] / (rho_test * n[ok] ** 3 * PROP_D_M ** 5)
    fm = np.full_like(t_n, np.nan)
    fm[ok] = ct[ok] ** 1.5 / (math.sqrt(2.0) * cp[ok])
    # Same FM from the disc-loading form (does not need RPM): FM = T^1.5 / sqrt(2 rho A) / P_shaft
    fm_disc = np.where(t_n > 0.2, np.maximum(t_n, 0.0) ** 1.5 / math.sqrt(2 * rho_test * DISC_AREA_M2) / (ETA_MOTOR * np.maximum(p_e, 1e-6)), np.nan)

    i_max = int(np.nanargmax(t_n))
    t_max_kgf = float(t_n[i_max] / GRAVITY_MS2)
    v_at_max = float(d["voltage_v"][i_max])
    usable_kgf = t_max_kgf * (RHO_HYD_KGM3 / rho_test) * (V_SAG_FLIGHT / v_at_max) ** 2

    def interp(x, xp, fp):
        m = np.isfinite(xp) & np.isfinite(fp)
        idx = np.argsort(xp[m])
        return float(np.interp(x, xp[m][idx], fp[m][idx]))

    hover = {}
    for key, th in hover_n.items():
        # Thrust at hover must be scaled back to test voltage conditions: compare at equal thrust
        p_h = interp(th, t_n, p_e)
        i_h = interp(th, t_n, d["current_a"])
        thr = interp(th, t_n, d["throttle_pct"]) / 100.0
        rpm_h = interp(th, t_n, d["rpm"])
        mass_kg = 4 * th / GRAVITY_MS2
        i_total = (4 * p_h + AVIONICS_POWER_W) / V_NOM
        hover[key] = {
            "hover_thrust_gf": round(th / GRAVITY_MS2 * 1000, 1),
            "elec_power_per_motor_w": round(p_h, 1),
            "current_per_motor_a_at_test_v": round(i_h, 2),
            "throttle_frac": round(thr, 3),
            "rpm": round(rpm_h),
            "efficiency_g_per_w": round(th / GRAVITY_MS2 * 1000 / p_h, 2),
            "mot_thst_hover_pred": round(th / (usable_kgf * GRAVITY_MS2), 3),
            "thrust_to_weight": round(4 * usable_kgf / mass_kg, 2),
            "total_hover_current_a_no_box_loss": round(i_total, 1),
            "hover_time_min_no_box_loss": round(USABLE_PACK_FRAC * PACK_AH / i_total * 60, 2),
            "hover_time_min_with_box_20pct": round(USABLE_PACK_FRAC * PACK_AH / ((4 * p_h * 1.2 + AVIONICS_POWER_W) / V_NOM) * 60, 2),
            "fm_at_hover": round(interp(th, t_n, fm_disc), 3),
        }
    gap = d["tip_gap_mm"][0]
    return {
        "config": name,
        "tip_gap_mm": None if not np.isfinite(gap) else float(gap),
        "is_ducted": bool(name.lower().startswith("duct") or np.isfinite(gap)),
        "max_thrust_kgf_measured": round(t_max_kgf, 3),
        "voltage_at_max_v": round(v_at_max, 2),
        "current_at_max_a": round(float(d["current_a"][i_max]), 1),
        "power_at_max_w": round(float(p_e[i_max]), 1),
        "rpm_at_max": round(float(d["rpm"][i_max])),
        "usable_max_thrust_kgf": round(usable_kgf, 3),
        "ct_mean_upper_half": round(float(np.nanmean(ct[len(ct) // 2:])), 4),
        "cp_mean_upper_half": round(float(np.nanmean(cp[len(cp) // 2:])), 4),
        "fm_max": round(float(np.nanmax(fm_disc)), 3),
        "hover": hover,
        "_curves": {"pwm": d["pwm_us"], "thr": d["throttle_pct"], "t_g": d["thrust_g"], "p": p_e,
                    "i": d["current_a"], "rpm": d["rpm"], "ct": ct, "cp": cp, "fm": fm_disc},
    }


def gate_decision(usable_kgf: float) -> Dict:
    m_skids = summarize_bom(get_locked_5in_bom(False))["all_up_mass_kg"]
    m_legs = summarize_bom(get_locked_5in_bom(True))["all_up_mass_kg"]
    if usable_kgf >= GATE_LEGS_KGF:
        decision, code = "PASS (legs): keep 5-inch as designed and fit the retractable legs", "LEGS"
    elif usable_kgf >= GATE_SKIDS_KGF:
        decision, code = "PASS (skids): keep 5-inch with flush TPU skids + cradle (or cut ~80 g to fit legs)", "SKIDS"
    elif usable_kgf >= GATE_TRIM_KGF:
        decision, code = "MARGINAL: keep 5-inch, trim ~140 g (3 mm grilles, LW-PLA panels, 1.2 mm duct wall)", "TRIM"
    elif usable_kgf >= GATE_MIN_KGF:
        decision, code = "LOW: accept T/W 2.0-2.3 for a slow demo, or switch to the 7-inch fallback", "ACCEPT_LOW_TW"
    else:
        decision, code = "FAIL: switch to the 7-inch fallback set (F1-F6)", "SWITCH_7IN"
    return {
        "usable_thrust_kgf": round(usable_kgf, 3),
        "tw_skids": round(4 * usable_kgf / m_skids, 2),
        "tw_legs": round(4 * usable_kgf / m_legs, 2),
        "decision_code": code,
        "decision": decision,
    }


# ============================================================================
# 4. Plot
# ============================================================================
def make_plot(results: List[Dict], baseline: Dict, out_png: Path, synthetic: bool) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    colors = ["#7f7f7f", "#bcbd22", "#2ca02c", "#1f4e79", "#d62728"]
    fig, axs = plt.subplots(2, 3, figsize=(17, 10))
    tag = " [SYNTHETIC - model prediction until real stand data is loaded]" if synthetic else " [MEASURED]"
    fig.suptitle("Fig 7 - Stage 0 thrust stand: ECO II 2207 1700KV, 5\" props, 6S" + tag, fontsize=13, fontweight="bold")

    for r, c in zip(results, colors):
        cv = r["_curves"]
        lw = 2.4 if r is baseline else 1.3
        axs[0, 0].plot(cv["thr"], cv["t_g"], color=c, lw=lw, marker="o", ms=3, label=r["config"])
        axs[0, 1].plot(cv["p"], cv["t_g"], color=c, lw=lw, label=r["config"])
        m = np.isfinite(cv["fm"])
        axs[0, 2].plot(cv["t_g"][m], cv["fm"][m], color=c, lw=lw, label=r["config"])
        m2 = np.isfinite(cv["ct"])
        axs[1, 0].plot(cv["rpm"][m2], cv["ct"][m2], color=c, lw=lw, label=f"C_T {r['config']}")
        axs[1, 0].plot(cv["rpm"][m2], cv["cp"][m2] * 5, color=c, lw=0.8, ls="--")

    th_legs = results[0]["hover"]["legs_1905g"]["hover_thrust_gf"]
    th_skids = results[0]["hover"]["skids_1825g"]["hover_thrust_gf"]
    for ax in (axs[0, 0], axs[0, 1]):
        ax.axhline(th_legs, color="k", ls=":", lw=0.8)
        ax.text(ax.get_xlim()[0], th_legs + 15, f" hover {th_legs:.0f} gf (1,905 g)", fontsize=7)
    axs[0, 0].set(xlabel="Throttle (%)", ylabel="Thrust (g)", title="(a) Thrust vs throttle")
    axs[0, 1].set(xlabel="Electrical power (W)", ylabel="Thrust (g)", title="(b) Thrust vs power: duct gain at equal power")
    axs[0, 2].axhline(0.60, color="k", ls="--", lw=0.8)
    axs[0, 2].text(50, 0.605, "sizing FM 0.60", fontsize=7)
    axs[0, 2].axvline(th_legs, color="k", ls=":", lw=0.8)
    axs[0, 2].set(xlabel="Thrust (g)", ylabel="Figure of merit (eta_m = 0.8)", title="(c) FM vs thrust (disc area of prop)")
    axs[1, 0].set(xlabel="RPM", ylabel="C_T (solid), 5 x C_P (dashed)", title="(d) Non-dimensional coefficients")

    ax = axs[1, 1]
    names = [r["config"].replace("_", "\n") for r in results]
    vals = [r["usable_max_thrust_kgf"] for r in results]
    bars = ax.bar(range(len(results)), vals, color=colors[: len(results)])
    for b, v in zip(bars, vals):
        ax.text(b.get_x() + b.get_width() / 2, v + 0.01, f"{v:.3f}", ha="center", fontsize=8)
    for y, lab, col in ((GATE_LEGS_KGF, "1.19 legs", "green"), (GATE_SKIDS_KGF, "1.14 skids", "orange"),
                        (GATE_TRIM_KGF, "1.10 trim", "red"), (GATE_MIN_KGF, "0.95 -> 7-inch", "darkred")):
        ax.axhline(y, color=col, ls="--", lw=1)
        ax.text(len(results) - 0.5, y + 0.005, lab, fontsize=7, color=col, ha="right")
    ax.set_xticks(range(len(results)))
    ax.set_xticklabels(names, fontsize=7)
    ax.set_ylim(0.8, max(vals) * 1.12)
    ax.set(ylabel="Usable thrust per motor (kgf @ 21.5 V, rho 1.09)", title="(e) Slide 6 decision gate")

    ax = axs[1, 2]
    x = np.arange(len(results))
    tw = [r["hover"]["legs_1905g"]["thrust_to_weight"] for r in results]
    ht0 = [r["hover"]["legs_1905g"]["hover_time_min_no_box_loss"] for r in results]
    ht20 = [r["hover"]["legs_1905g"]["hover_time_min_with_box_20pct"] for r in results]
    ax.bar(x - 0.2, ht0, 0.4, color="#9ecae1", label="hover time, no box loss (min)")
    ax.bar(x + 0.2, ht20, 0.4, color="#3182bd", label="hover time, +20 % box loss (min)")
    ax.axhline(4.5, color="#3182bd", ls=":", lw=1)
    ax2 = ax.twinx()
    ax2.plot(x, tw, "k-o", label="T/W (1,905 g)")
    ax2.axhline(2.5, color="k", ls="--", lw=0.8)
    ax2.set_ylabel("Thrust-to-weight")
    ax.set_xticks(x)
    ax.set_xticklabels(names, fontsize=7)
    ax.set(ylabel="Hover time (min)", title="(f) Endurance & T/W from stand data (legs, 1,905 g)")
    ax.legend(fontsize=7, loc="lower left")
    ax2.legend(fontsize=7, loc="lower right")

    for a in axs.flat:
        a.grid(alpha=0.3)
    axs[0, 0].legend(fontsize=7)
    fig.tight_layout(rect=[0, 0, 1, 0.95])
    out_png.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_png, dpi=140)
    plt.close(fig)


# ============================================================================
# 5. Main
# ============================================================================
def main() -> None:
    ap = argparse.ArgumentParser(description="Stage 0 thrust-stand analyzer (Hardware Sync Gate B)")
    ap.add_argument("--csv", action="append", help="serial CSV(s) from thrust_stand_logger.ino; omit for synthetic")
    ap.add_argument("--rho-test", type=float, default=RHO_HYD_KGM3, help="air density during the test (kg/m^3)")
    ap.add_argument("--baseline", default=None, help="config label of the flight baseline (default: 1.5 mm duct)")
    args = ap.parse_args()

    synthetic = not args.csv
    if synthetic:
        csv_paths = [generate_synthetic_csv(ROOT / "outputs" / "logs" / "stage0_thrust_stand_sweep.csv")]
        print(f"No --csv given: wrote SYNTHETIC model dataset {csv_paths[0]}")
    else:
        csv_paths = [Path(p) for p in args.csv]

    data = read_stand_csv(csv_paths)
    m_skids = summarize_bom(get_locked_5in_bom(False))["all_up_mass_kg"]
    m_legs = summarize_bom(get_locked_5in_bom(True))["all_up_mass_kg"]
    hover_n = {"skids_1825g": m_skids * GRAVITY_MS2 / 4, "legs_1905g": m_legs * GRAVITY_MS2 / 4}

    results = [analyze_config(name, d, args.rho_test, hover_n) for name, d in data.items()]
    open_ref = next((r for r in results if not r["is_ducted"] and "3blade" in r["config"]),
                    next((r for r in results if not r["is_ducted"]), None))
    ducted = [r for r in results if r["is_ducted"]]
    if args.baseline:
        baseline = next(r for r in results if r["config"] == args.baseline)
    else:
        baseline = next((r for r in ducted if r["tip_gap_mm"] == 1.5), ducted[0] if ducted else results[0])

    # Duct gain at equal electrical power (hover power of the open prop at 1,905 g)
    if open_ref is not None:
        p_ref = open_ref["hover"]["legs_1905g"]["elec_power_per_motor_w"]
        cv0 = open_ref["_curves"]
        t_open = float(np.interp(p_ref, cv0["p"], cv0["t_g"]))
        for r in results:
            cv = r["_curves"]
            r["thrust_at_open_hover_power_g"] = round(float(np.interp(p_ref, cv["p"], cv["t_g"])), 1)
            r["duct_gain_at_equal_power_pct"] = round(100 * (r["thrust_at_open_hover_power_g"] / t_open - 1), 2)

    for r in results:
        r["gate"] = gate_decision(r["usable_max_thrust_kgf"])
    best = max(ducted, key=lambda r: r["usable_max_thrust_kgf"]) if ducted else None

    print("\n=== Stage 0 results (usable = scaled to 21.5 V loaded, rho 1.09) ===")
    print(f"{'config':24s} {'Tmax':>6s} {'usable':>7s} {'FMmax':>6s} {'vs3bl%':>6s} {'P_hov':>6s} {'g/W':>5s} {'T/W L':>6s} {'hov0/20':>9s}  gate")
    for r in results:
        h = r["hover"]["legs_1905g"]
        print(f"{r['config']:24s} {r['max_thrust_kgf_measured']:6.3f} {r['usable_max_thrust_kgf']:7.3f} {r['fm_max']:6.3f} "
              f"{r.get('duct_gain_at_equal_power_pct', 0):6.2f} {h['elec_power_per_motor_w']:6.1f} {h['efficiency_g_per_w']:5.2f} "
              f"{h['thrust_to_weight']:6.2f} {h['hover_time_min_no_box_loss']:4.2f}/{h['hover_time_min_with_box_20pct']:4.2f}  {r['gate']['decision_code']}")
    print(f"\nFlight baseline: {baseline['config']} -> {baseline['gate']['decision']}")
    if best:
        print(f"Best tip gap on the stand: {best['tip_gap_mm']} mm ({best['usable_max_thrust_kgf']:.3f} kgf usable). "
              "Arms flex ~0.5 mm in flight, so 1.0 mm is stand-only.")

    out_json = ROOT / "outputs" / "sizing" / "stage0_thrust_stand_results.json"
    out_json.parent.mkdir(parents=True, exist_ok=True)
    clean = [{k: v for k, v in r.items() if k != "_curves"} for r in results]
    out_json.write_text(json.dumps({
        "data_source": "SYNTHETIC model prediction" if synthetic else [str(p) for p in csv_paths],
        "rho_test": args.rho_test,
        "flight_baseline_config": baseline["config"],
        "flight_baseline_gate": baseline["gate"],
        "best_ducted_config": best["config"] if best else None,
        "configs": clean,
    }, indent=2), encoding="utf-8")
    make_plot(results, baseline, ROOT / "outputs" / "plots" / "fig7_stage0_thrust_stand_results.png", synthetic)
    print(f"Saved {out_json} and outputs/plots/fig7_stage0_thrust_stand_results.png")


if __name__ == "__main__":
    main()
