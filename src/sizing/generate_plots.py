"""
Publication-Grade Trade-Study & Aerodynamic Plot Generator
for Project 8: 5-Inch Enclosed Ducted-Fan Quadrotor.

Generates 6 engineering figures in outputs/plots/:
1. fig1_sizing_convergence_and_bom.png  - Iterative sizing convergence & BOM mass breakdown
2. fig2_prop_and_battery_trades.png     - 5" vs 6" vs 7" trade study & 2200/2600/3300 mAh sweep
3. fig3_duct_tip_gap_and_stage0_gate.png - Bellmouth tip gap sweep & Stage 0 decision gate thresholds
4. fig4_stage1_vs_stage2_and_payload.png - Bare vs Enclosed hover current/endurance & payload sensitivity
5. fig5_crosswind_ram_drag_polar.png    - Crosswind momentum ram drag vs bluff-body drag & lean angle
"""

from __future__ import annotations
from pathlib import Path
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from duct_aero import (
    get_locked_5in_geometry,
    evaluate_duct_aero,
    compute_crosswind_lean,
)
from sizing_model import (
    get_locked_5in_bom,
    run_all_trade_studies,
)


plt.rcParams.update({
    "font.family": "sans-serif",
    "font.size": 10,
    "axes.titlesize": 11,
    "axes.titleweight": "bold",
    "axes.labelsize": 10,
    "axes.grid": True,
    "grid.alpha": 0.3,
    "figure.dpi": 180,
})


def generate_all_plots() -> Path:
    data = run_all_trade_studies()
    out_dir = Path(__file__).resolve().parents[2] / "outputs" / "plots"
    out_dir.mkdir(parents=True, exist_ok=True)

    # ------------------------------------------------------------------------
    # Figure 1: Sizing Convergence Loop (Slide 8) & BOM Group Mass Breakdown (Slide 9)
    # ------------------------------------------------------------------------
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12.5, 4.8))

    rounds_legs = data["convergence_loop_legs"]
    rounds_skids = data["convergence_loop_skids"]
    r_idx = [r["round"] for r in rounds_legs]
    m_legs = [r["assumed_or_bom_mass_g"] for r in rounds_legs]
    m_skids = [r["assumed_or_bom_mass_g"] for r in rounds_skids]

    ax1.plot(r_idx, m_legs, "o-", color="#b45309", lw=2.4, ms=8, label="5-in w/ Retractable Legs (1,905 g)")
    ax1.plot(r_idx, m_skids, "s--", color="#0f766e", lw=2.4, ms=8, label="5-in w/ Flush TPU Skids (1,825 g)")
    for x, yl, ys in zip(r_idx, m_legs, m_skids):
        ax1.annotate(f"{yl:.0f} g", (x, yl), textcoords="offset points", xytext=(0, 8), ha="center", fontweight="bold", color="#b45309")
        ax1.annotate(f"{ys:.0f} g", (x, ys), textcoords="offset points", xytext=(0, -15), ha="center", fontweight="bold", color="#0f766e")
    ax1.set_xticks([1, 2, 3])
    ax1.set_xticklabels(["Round 1\n(Worst-Case Guess)", "Round 2\n(BOM Re-Weigh)", "Round 3\n(Converged <2%)"])
    ax1.set_ylabel("All-Up Mass (g)")
    ax1.set_ylim(1700, 2100)
    ax1.set_title("Iterative Sizing Loop Convergence (5-Inch Locked)")
    ax1.legend(loc="upper right")

    # Slide 9 Group mass breakdown (5-inch Skids vs Legs)
    groups = ["Battery\n(6S 2600)", "Box\nStructure", "Ducts x4\n(PETG)", "Payload\n(Cam+Ballast)", "Propulsion\n(Motors+ESC)", "Grilles x8\n(PLA)", "Frame +\nWiring", "Avionics +\nRadios", "Landing\nGear"]
    masses_skids = [400.0, 387.6, 230.0, 180.0, 196.2, 156.4, 184.0, 63.5, 27.6]
    masses_legs  = [400.0, 387.6, 230.0, 180.0, 196.2, 156.4, 184.0, 63.5, 107.3]
    y_pos = np.arange(len(groups))
    h = 0.38
    ax2.barh(y_pos - h/2, masses_skids, height=h, color="#0f766e", label="Skids Baseline (1,825 g)")
    ax2.barh(y_pos + h/2, masses_legs, height=h, color="#b45309", label="Retractable Legs (1,905 g)")
    ax2.set_yticks(y_pos)
    ax2.set_yticklabels(groups, fontsize=8.5)
    ax2.invert_yaxis()
    ax2.set_xlabel("Counted Mass incl. +15% Contingency on Estimates (g)")
    ax2.set_title("5-Inch Mass Budget Breakdown by Subsystem")
    ax2.legend(loc="lower right")

    fig.tight_layout()
    fig.savefig(out_dir / "fig1_sizing_convergence_and_bom.png")
    plt.close(fig)

    # ------------------------------------------------------------------------
    # Figure 2: Prop Size Trade Study (5" vs 6" vs 7") & Battery Capacity Sweep
    # ------------------------------------------------------------------------
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12.5, 4.8))

    props = data["prop_size_trade_study"]
    p_labels = ["5-inch\n(Locked Baseline)", "6-inch\n(Interpolated)", "7-inch\n(Fallback)"]
    tw_vals = [p["thrust_to_weight"] for p in props]
    t0_vals = [p["hover_time_0pct_min"] for p in props]
    t20_vals = [p["hover_time_20pct_min"] for p in props]
    mass_vals = [p["all_up_mass_g"] / 1000.0 for p in props]

    x = np.arange(len(p_labels))
    w = 0.25
    ax1.bar(x - w, tw_vals, width=w, color="#1d4ed8", label="Thrust-to-Weight (T/W)")
    ax1.bar(x, t0_vals, width=w, color="#15803d", label="Hover Time (0% Encl. Loss) [min]")
    ax1.bar(x + w, t20_vals, width=w, color="#d97706", label="Hover Time (+20% Encl. Loss) [min]")
    ax1_twin = ax1.twinx()
    ax1_twin.plot(x, mass_vals, "D--", color="#be123c", lw=2, ms=7, label="All-Up Mass (kg)")
    ax1_twin.set_ylabel("All-Up Mass (kg)", color="#be123c")
    ax1_twin.set_ylim(1.0, 3.2)
    ax1_twin.grid(False)

    ax1.axhline(2.5, color="#1d4ed8", ls=":", lw=1.2, label="Min T/W Target (2.5)")
    ax1.set_xticks(x)
    ax1.set_xticklabels(p_labels)
    ax1.set_ylabel("T/W Ratio  |  Hover Time (min)")
    ax1.set_ylim(0, 6.8)
    ax1.set_title("Propeller Size Trade Study (Why 5-Inch Wins)")
    ax1.legend(loc="upper left", fontsize=8)

    # Battery sweep on 5-inch
    batts = data["battery_capacity_sweep"]
    b_labels = ["6S 2200 mAh\n(340 g)", "6S 2600 mAh\n(400 g - Locked)", "6S 3300 mAh\n(520 g)"]
    bx = np.arange(len(b_labels))
    b_h0 = [b["hover_time_0pct_min"] for b in batts]
    b_h20 = [b["hover_time_20pct_min"] for b in batts]
    b_rem = [b["mission_3min2_batt_left_20pct"] / 10.0 for b in batts]

    ax2.bar(bx - w/2, b_h0, width=w, color="#0f766e", label="Hover Time (0% Loss) [min]")
    ax2.bar(bx + w/2, b_h20, width=w, color="#ea580c", label="Hover Time (+20% Loss) [min]")
    for i, (v0, v20) in enumerate(zip(b_h0, b_h20)):
        ax2.text(i - w/2, v0 + 0.1, f"{v0:.1f}m", ha="center", fontsize=8.5, fontweight="bold")
        ax2.text(i + w/2, v20 + 0.1, f"{v20:.1f}m", ha="center", fontsize=8.5, fontweight="bold")
    ax2.axhline(4.5, color="#dc2626", ls="--", lw=1.2, label="Committed Hover Target (4.5 min)")
    ax2.set_xticks(bx)
    ax2.set_xticklabels(b_labels)
    ax2.set_ylabel("Hover Endurance (minutes)")
    ax2.set_ylim(0, 7.5)
    ax2.set_title("5-Inch Battery Capacity Sweep (6S Pack Selection)")
    ax2.legend(loc="upper left", fontsize=8.5)

    fig.tight_layout()
    fig.savefig(out_dir / "fig2_prop_and_battery_trades.png")
    plt.close(fig)

    # ------------------------------------------------------------------------
    # Figure 3: Duct Tip Clearance Sweep & Stage 0 Thrust-Stand Decision Gate
    # ------------------------------------------------------------------------
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12.5, 4.8))

    gaps = np.linspace(0.75, 2.5, 36)
    clean_gains = []
    net_mults = []
    for g in gaps:
        perf = evaluate_duct_aero(get_locked_5in_geometry(float(g)))
        clean_gains.append(perf.clean_duct_thrust_gain_pct)
        net_mults.append((perf.net_thrust_multiplier - 1.0) * 100.0)

    ax1.plot(gaps, clean_gains, "-", color="#0284c7", lw=2.4, label="Clean Bellmouth Duct Gain (%)")
    ax1.plot(gaps, net_mults, "--", color="#dc2626", lw=2.4, label="Net Enclosed (Duct - 2 Grilles - Box) (%)")
    for test_g, lbl in [(1.0, "1.0 mm\n(Stand Only)"), (1.5, "1.5 mm\n(Flight Baseline)"), (2.0, "2.0 mm\n(Wide Test)")]:
        p_pt = evaluate_duct_aero(get_locked_5in_geometry(test_g))
        ax1.plot(test_g, p_pt.clean_duct_thrust_gain_pct, "o", color="#0284c7", ms=8)
        ax1.annotate(lbl, (test_g, p_pt.clean_duct_thrust_gain_pct), textcoords="offset points", xytext=(0, 10), ha="center", fontsize=8)
    ax1.axvline(0.5, color="#7c3aed", ls=":", lw=1.5, label="Carbon Arm Flex at Full Thrust (~0.5 mm)")
    ax1.set_xlabel("Radial Tip Clearance δ (mm)")
    ax1.set_ylabel("Thrust Modification at Constant Power (%)")
    ax1.set_title("5-Inch Bellmouth Duct Tip-Gap Sensitivity (Stage 0 Sweep)")
    ax1.legend(loc="upper right", fontsize=8)

    # Stage 0 Thrust Gate Chart (Slide 6 & 11)
    t_motor = np.linspace(0.85, 1.55, 100)
    tw_skids_curve = 4.0 * t_motor / 1.8252
    tw_legs_curve = 4.0 * t_motor / 1.9049
    ax2.plot(t_motor, tw_skids_curve, "-", color="#0f766e", lw=2.4, label="5-in w/ Skids (1,825 g)")
    ax2.plot(t_motor, tw_legs_curve, "-", color="#b45309", lw=2.4, label="5-in w/ Retractable Legs (1,905 g)")
    ax2.axvspan(1.19, 1.55, color="#dcfce7", alpha=0.6, label=">=1.19 kgf: Fit Retractable Legs (T/W>=2.5)")
    ax2.axvspan(1.10, 1.19, color="#fef9c3", alpha=0.6, label="1.10-1.19 kgf: Keep 5-in (Skids / Trim 140g)")
    ax2.axvspan(0.85, 1.10, color="#fee2e2", alpha=0.5, label="<1.10 kgf: Lower T/W or 7-in Fallback")
    ax2.axvline(1.41, color="#1d4ed8", ls="--", lw=2.0, label="Predicted Usable Thrust (1.41 kgf)")
    ax2.axhline(2.50, color="#111827", ls=":", lw=1.4, label="Target T/W = 2.50")
    ax2.set_xlabel("Stage 0 Measured Usable Thrust per Motor (kgf)")
    ax2.set_ylabel("Aircraft Thrust-to-Weight Ratio (T/W)")
    ax2.set_xlim(0.85, 1.55)
    ax2.set_ylim(1.7, 3.5)
    ax2.set_title("Stage 0 Thrust-Stand Decision Gate (Slide 6 Rule)")
    ax2.legend(loc="upper left", fontsize=7.8)

    fig.tight_layout()
    fig.savefig(out_dir / "fig3_duct_tip_gap_and_stage0_gate.png")
    plt.close(fig)

    # ------------------------------------------------------------------------
    # Figure 4: Stage 1 (Bare) vs Stage 2 (Enclosed) & Payload Sensitivity
    # ------------------------------------------------------------------------
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12.5, 4.8))

    prim = data["primary_configurations"][:4]
    c_labels = ["Stage 1 Bare\n(No Payload\n981 g)", "Stage 1 Bare\n(180g Payload\n1,161 g)", "Stage 2 Enclosed\n(Skids Baseline\n1,825 g)", "Stage 2 Enclosed\n(Legs Upgrade\n1,905 g)"]
    c_curr = [c["hover_current_20pct_a"] if "Stage 2" in c["config_name"] else c["hover_current_0pct_a"] for c in prim]
    c_time = [c["hover_time_20pct_min"] if "Stage 2" in c["config_name"] else c["hover_time_0pct_min"] for c in prim]
    cx = np.arange(len(c_labels))

    ax1.bar(cx - 0.18, c_curr, width=0.35, color="#be123c", label="Hover Current Draw (A)")
    ax1_t = ax1.twinx()
    ax1_t.bar(cx + 0.18, c_time, width=0.35, color="#0284c7", label="Hover Endurance (min)")
    ax1.set_xticks(cx)
    ax1.set_xticklabels(c_labels, fontsize=8.5)
    ax1.set_ylabel("Total Hover Current (A)", color="#be123c")
    ax1_t.set_ylabel("Hover Flight Time (min)", color="#0284c7")
    ax1.set_ylim(0, 32)
    ax1_t.set_ylim(0, 16)
    ax1_t.grid(False)
    ax1.set_title("Stage 1 (Bare Quad) vs. Stage 2 (Enclosed Box)")

    # Payload sensitivity sweep
    pl_sweep = data["payload_sensitivity_sweep"]
    pl_m = [p["payload_g"] for p in pl_sweep]
    pl_t0 = [p["hover_time_0pct_min"] for p in pl_sweep]
    pl_t20 = [p["hover_time_20pct_min"] for p in pl_sweep]
    pl_tw = [p["thrust_to_weight"] for p in pl_sweep]

    ax2.plot(pl_m, pl_t0, "o-", color="#0f766e", lw=2.2, label="Hover Time (0% Encl. Loss) [min]")
    ax2.plot(pl_m, pl_t20, "s-", color="#ea580c", lw=2.2, label="Hover Time (+20% Encl. Loss) [min]")
    ax2.axvline(180, color="#1d4ed8", ls="--", lw=1.8, label="Committed Payload = 180 g (~10% m)")
    ax2.set_xlabel("Payload Mass in Central Bay (g)")
    ax2.set_ylabel("Hover Endurance (minutes)")
    ax2.set_ylim(4.0, 6.8)
    ax2.set_title("Payload Mass Sensitivity (~0.32 min per 100 g)")
    ax2.legend(loc="upper right", fontsize=8.5)

    fig.tight_layout()
    fig.savefig(out_dir / "fig4_stage1_vs_stage2_and_payload.png")
    plt.close(fig)

    # ------------------------------------------------------------------------
    # Figure 5: Crosswind Momentum (Ram) Drag vs. Form Drag & Lean Angle
    # ------------------------------------------------------------------------
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12.5, 4.6))

    g5 = get_locked_5in_geometry(1.5)
    winds_ms = np.linspace(0.5, 7.0, 28)
    ram_forces = []
    form_forces = []
    lean_angles = []
    for w_ms in winds_ms:
        cw = compute_crosswind_lean(g5, all_up_mass_kg=1.8252, wind_speed_ms=float(w_ms))
        ram_forces.append(cw["ram_drag_n"])
        form_forces.append(cw["form_drag_n"])
        lean_angles.append(cw["lean_angle_deg"])

    ax1.stackplot(
        winds_ms * 3.6,
        ram_forces,
        form_forces,
        labels=["Inlet Momentum (Ram) Drag (~91% of total)", "Cuboid Box Side Form Drag (~9% of total)"],
        colors=["#0284c7", "#f59e0b"],
        alpha=0.8,
    )
    ax1.axvline(15.0, color="#dc2626", ls="--", lw=1.8, label="Max Demo Wind Target (15 km/h = 4.2 m/s)")
    ax1.set_xlabel("Crosswind Speed (km/h)")
    ax1.set_ylabel("Lateral Side Force on Aircraft (N)")
    ax1.set_title("Crosswind Force Breakdown (Duct Ram Drag Dominates)")
    ax1.legend(loc="upper left", fontsize=8.5)

    ax2.plot(winds_ms * 3.6, lean_angles, "-", color="#be123c", lw=2.5, label="Required Pitch/Roll Lean Angle (deg)")
    ax2.axvline(15.0, color="#dc2626", ls="--", lw=1.8, label="15 km/h Target Limit (~15.5 deg lean)")
    ax2.axhline(15.5, color="#be123c", ls=":", lw=1.2)
    ax2.set_xlabel("Crosswind Speed (km/h)")
    ax2.set_ylabel("Steady Hover Lean Angle into Wind (deg)")
    ax2.set_title("Required Tilt Angle vs. Crosswind Speed (5-Inch Box)")
    ax2.legend(loc="upper left", fontsize=8.5)

    fig.tight_layout()
    fig.savefig(out_dir / "fig5_crosswind_ram_drag_polar.png")
    plt.close(fig)

    print(f"Successfully generated 5 trade-study & aerodynamic figures in: {out_dir}")
    return out_dir


if __name__ == "__main__":
    generate_all_plots()
