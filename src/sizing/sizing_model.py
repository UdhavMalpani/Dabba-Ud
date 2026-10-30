"""
Complete Propulsion Sizing, Mass Budget & Iterative Convergence Engine
for Project 8: 5-Inch Enclosed Ducted-Fan Quadrotor (IIIT Hyderabad EC4.402).

Matches all equations and BOM figures from:
- Project 8 - Component List BOM final.pdf (5-inch 6S locked baseline + legs option + 7-inch & 4S comparisons)
- Project 8 — Design Review Presentation.pdf (Slides 6, 7, 8, 9)
- context.md (Momentum theory derivations, Stage 1 vs Stage 2, trade studies)
"""

from __future__ import annotations
from dataclasses import dataclass
import json
import math
from pathlib import Path
from typing import Dict, List, Tuple

from duct_aero import (
    DuctGrilleGeometry,
    get_locked_5in_geometry,
    get_fallback_7in_geometry,
    compute_crosswind_lean,
)


# ============================================================================
# 1. Environmental & System Constants (Hyderabad 540 m ASL, 30 C)
# ============================================================================
RHO_HYD_KGM3 = 1.09          # Air density at 540 m, 30 C, ~95 kPa (kg/m^3)
RHO_SL_KGM3 = 1.225          # Standard sea-level air density (kg/m^3)
GRAVITY_MS2 = 9.81           # Gravitational acceleration (m/s^2)
FIGURE_OF_MERIT = 0.60       # Rotor Figure of Merit (FM)
MOTOR_ESC_EFF = 0.80         # Combined motor + ESC efficiency (eta)
USABLE_PACK_FRAC = 0.80      # Usable LiPo capacity fraction (80% rules out deep discharge)
AVIONICS_POWER_W = 10.0      # FC + GPS + Receiver + Telemetry + Camera power (W)
CONTINGENCY_EST = 1.15       # +15% contingency applied strictly to estimated parts


@dataclass
class BOMItem:
    item_id: str
    name: str
    category: str             # "Propulsion", "Avionics", "Payload", "Structure", "Gear"
    basis: str                # "Spec" (1.0x), "Est" (1.15x), "Allow" (1.0x), "Defined" (1.0x)
    qty_on_aircraft: int
    mass_each_g: float
    in_stage1_bare: bool = True

    @property
    def raw_total_g(self) -> float:
        return self.qty_on_aircraft * self.mass_each_g

    @property
    def counted_g(self) -> float:
        mult = CONTINGENCY_EST if self.basis == "Est" else 1.0
        return round(self.raw_total_g * mult, 2)


def get_locked_5in_bom(use_retractable_legs: bool = False) -> List[BOMItem]:
    """
    Returns the exact BOM table from 'Project 8 - Component List BOM final.pdf'
    for the locked 5-inch, 6S 2600 mAh configuration.
    - use_retractable_legs=False -> Primary Flush TPU Skids + Ground Cradle (1,825.2 g)
    - use_retractable_legs=True  -> Optional Retractable MG90S Legs (1,904.9 g)
    """
    items = [
        # 1. Propulsion and power
        BOMItem("1", "Motors (Emax ECO II 2207 1700KV)", "Propulsion", "Spec", 4, 33.4, True),
        BOMItem("2", "Motor leads allowance", "Propulsion", "Est", 4, 4.0, True),
        BOMItem("3", "Propellers, 3-blade (5x4.3 HQProp/Gemfan)", "Propulsion", "Est", 4, 4.5, True),
        BOMItem("5", "ESC (SpeedyBee BLS 60A 4-in-1)", "Propulsion", "Spec", 1, 23.5, True),
        BOMItem("6", "Battery (6S 2600 mAh >=65C XT90)", "Propulsion", "Spec", 1, 400.0, True),
        # Avionics and radios
        BOMItem("10", "Flight controller (SpeedyBee F405 V4)", "Avionics", "Spec", 1, 10.5, True),
        BOMItem("11", "GPS + compass (Matek M10Q-5883)", "Avionics", "Spec", 1, 8.0, True),
        BOMItem("12", "GPS mast + cable (PA-CF base)", "Avionics", "Est", 1, 10.0, True),
        BOMItem("13", "RC receiver + telemetry air module", "Avionics", "Allow", 1, 30.0, True),
        BOMItem("16", "Active buzzer", "Avionics", "Est", 1, 3.0, True),
        # Payload (180 g total)
        BOMItem("17", "Camera (RunCam Thumb 1080p60)", "Payload", "Spec", 1, 9.8, True),
        BOMItem("18", "Payload bay ballast", "Payload", "Defined", 1, 170.2, True),
        # 2. Structure and printed parts
        BOMItem("19", "Frame (Carbon true-X 4mm plate, 276mm)", "Structure", "Est", 1, 80.0, True),
        BOMItem("20", "Duct rings x4 (PETG bellmouth, 130mm bore)", "Structure", "Est", 4, 50.0, False),
        BOMItem("21", "Side panels (PLA, 0.8 mm)", "Structure", "Est", 1, 135.0, False),
        BOMItem("22", "Grilles x8 (PLA square grid, 1mm bar, 12mm pitch)", "Structure", "Est", 8, 17.0, False),
        BOMItem("23", "Decks, ribs, quick-attach joints (4 quarters)", "Structure", "Est", 1, 142.0, False),
        BOMItem("24", "Spine and bay walls (PLA, 45 mm cross spine)", "Structure", "Est", 1, 30.0, False),
        BOMItem("25", "Payload bay door, hinge, latch (PLA)", "Structure", "Est", 1, 30.0, False),
        BOMItem("26", "Wiring, XT90, capacitor, inserts, fasteners", "Structure", "Est", 1, 80.0, True),
    ]

    if not use_retractable_legs:
        items.append(
            BOMItem("27", "Skid pads x4 (TPU, flush with bottom)", "Gear", "Est", 4, 6.0, True)
        )
    else:
        items.extend([
            BOMItem("A1", "Leg servos (MG90S metal gear)", "Gear", "Spec", 2, 13.4, True),
            BOMItem("A2", "Servo supply (UBEC 5V >=3A)", "Gear", "Est", 1, 10.0, True),
            BOMItem("A3", "Printed legs (2x ~240mm with PA-CF hinges)", "Gear", "Est", 2, 25.0, True),
            BOMItem("A4", "Servo mounts (PA-CF)", "Gear", "Est", 1, 10.0, True),
        ])

    return items


def summarize_bom(items: List[BOMItem], stage1_only: bool = False) -> Dict[str, float]:
    """Computes mass breakdown by basis (Spec, Est, Allow+Defined) matching Slide 8 & 9."""
    filtered = [it for it in items if (it.in_stage1_bare or not stage1_only)]
    spec_g = sum(it.raw_total_g for it in filtered if it.basis == "Spec")
    est_raw_g = sum(it.raw_total_g for it in filtered if it.basis == "Est")
    est_counted_g = round(est_raw_g * CONTINGENCY_EST, 2)
    allow_defined_g = sum(it.raw_total_g for it in filtered if it.basis in ("Allow", "Defined"))
    total_g = round(spec_g + est_counted_g + allow_defined_g, 2)
    return {
        "spec_g": round(spec_g, 2),
        "est_raw_g": round(est_raw_g, 2),
        "est_counted_g": round(est_counted_g, 2),
        "allow_defined_g": round(allow_defined_g, 2),
        "all_up_mass_g": total_g,
        "all_up_mass_kg": round(total_g / 1000.0, 4),
        "estimated_share_pct": round(100.0 * est_counted_g / total_g, 1) if total_g > 0 else 0.0,
    }


@dataclass
class SizingResult:
    config_name: str
    prop_in: float
    all_up_mass_g: float
    hover_thrust_per_motor_n: float
    hover_thrust_per_motor_gf: float
    ideal_power_per_motor_w: float
    elec_power_per_motor_0pct_w: float
    elec_power_per_motor_20pct_w: float
    total_hover_power_0pct_w: float
    total_hover_power_20pct_w: float
    hover_current_0pct_a: float
    hover_current_20pct_a: float
    usable_max_thrust_per_motor_kgf: float
    thrust_to_weight: float
    hover_throttle_est: float
    hover_time_0pct_min: float
    hover_time_20pct_min: float
    mission_3min2_batt_left_0pct: float
    mission_3min2_batt_left_20pct: float
    peak_current_total_a: float
    battery_max_cont_current_a: float
    esc_max_cont_current_a: float
    hover_rpm: float
    max_rpm: float
    tip_speed_max_ms: float


def evaluate_propulsion_point(
    config_name: str,
    all_up_mass_g: float,
    prop_d_mm: float = 127.0,
    motor_kv: float = 1700.0,
    motor_spec_max_thrust_kgf: float = 1.98,
    motor_peak_current_a: float = 40.0,
    cell_count_s: int = 6,
    batt_capacity_mah: float = 2600.0,
    batt_c_rating: float = 65.0,
    esc_rating_a: float = 60.0,
    ct_prop: float = 0.125,
    apply_voltage_sag_derate: bool = True,
) -> SizingResult:
    """
    Evaluates momentum-theory hover power, current, endurance, T/W, and RPM
    following the exact equations on Slide 8 of the Design Review Presentation:
      T_h = m * g / 4
      P_i = T_h^(3/2) / sqrt(2 * rho * A)
      P_e = P_i / (FM * eta)
      I_h = (4 * P_e + P_av) / V_nom
      T_max_usable = T_spec * (rho / rho_0) * (V_sag / V_test)^2
      t_hover = (0.8 * C_Ah / I_h) * 60
    """
    mass_kg = all_up_mass_g / 1000.0
    disc_area_m2 = math.pi * ((prop_d_mm * 1e-3) ** 2) / 4.0

    v_nom = 3.7 * cell_count_s
    v_sag = (21.5 / 6.0) * cell_count_s
    v_test = 4.0 * cell_count_s

    # 1. Hover thrust per motor
    th_n = (mass_kg * GRAVITY_MS2) / 4.0
    th_gf = all_up_mass_g / 4.0

    # 2. Ideal & electrical hover power per motor (0% loss vs +20% enclosure loss)
    pi_w = (th_n ** 1.5) / math.sqrt(2.0 * RHO_HYD_KGM3 * disc_area_m2)
    pe_0pct_w = pi_w / (FIGURE_OF_MERIT * MOTOR_ESC_EFF)
    pe_20pct_w = pe_0pct_w * 1.20

    total_p_0pct_w = 4.0 * pe_0pct_w + AVIONICS_POWER_W
    total_p_20pct_w = 4.0 * pe_20pct_w + AVIONICS_POWER_W

    ih_0pct_a = total_p_0pct_w / v_nom
    ih_20pct_a = total_p_20pct_w / v_nom

    # 3. Usable max thrust per motor in Hyderabad air density + voltage sag
    density_factor = RHO_HYD_KGM3 / RHO_SL_KGM3  # 1.09 / 1.225 = 0.8898
    sag_factor = ((v_sag / v_test) ** 2) if apply_voltage_sag_derate else 1.0
    tmax_usable_kgf = motor_spec_max_thrust_kgf * density_factor * sag_factor

    tw_ratio = (4.0 * tmax_usable_kgf) / mass_kg

    # Hover throttle estimate (MOT_THST_HOVER ~ sqrt(1 / TWR) or thrust share)
    thrust_share = (mass_kg / 4.0) / tmax_usable_kgf
    hover_throttle = round(0.5 * thrust_share + 0.5 * math.sqrt(thrust_share), 3)

    # 4. Hover endurance (minutes)
    usable_ah = USABLE_PACK_FRAC * (batt_capacity_mah / 1000.0)
    th_0pct_min = (usable_ah / ih_0pct_a) * 60.0
    th_20pct_min = (usable_ah / ih_20pct_a) * 60.0

    # 5. Mission battery remaining after a 3.2-min autonomous waypoint mission
    # Using context.md Section 6 formula: left = 1 - 1.15 * t_mission / (t_hover / 0.8)
    t_mission_min = 3.2
    batt_left_0pct = max(0.0, 100.0 * (1.0 - 1.15 * t_mission_min / (th_0pct_min / USABLE_PACK_FRAC)))
    batt_left_20pct = max(0.0, 100.0 * (1.0 - 1.15 * t_mission_min / (th_20pct_min / USABLE_PACK_FRAC)))

    # 6. Peak current & C-rating check
    peak_total_a = 4.0 * motor_peak_current_a
    batt_max_a = (batt_capacity_mah / 1000.0) * batt_c_rating

    # 7. Rotational speed & blade tip speed
    # T = C_T * rho * n^2 * D^4  => n (rev/s) = sqrt(T / (C_T * rho * D^4))
    d_m = prop_d_mm * 1e-3
    n_hover_rps = math.sqrt(th_n / (ct_prop * RHO_HYD_KGM3 * (d_m ** 4)))
    tmax_n = tmax_usable_kgf * GRAVITY_MS2
    n_max_rps = math.sqrt(tmax_n / (ct_prop * RHO_HYD_KGM3 * (d_m ** 4)))
    hover_rpm = n_hover_rps * 60.0
    max_rpm = min(n_max_rps * 60.0, motor_kv * v_sag * 0.92)
    tip_speed_max_ms = (max_rpm / 60.0) * math.pi * d_m

    return SizingResult(
        config_name=config_name,
        prop_in=round(prop_d_mm / 25.4, 1),
        all_up_mass_g=round(all_up_mass_g, 1),
        hover_thrust_per_motor_n=round(th_n, 2),
        hover_thrust_per_motor_gf=round(th_gf, 1),
        ideal_power_per_motor_w=round(pi_w, 1),
        elec_power_per_motor_0pct_w=round(pe_0pct_w, 1),
        elec_power_per_motor_20pct_w=round(pe_20pct_w, 1),
        total_hover_power_0pct_w=round(total_p_0pct_w, 1),
        total_hover_power_20pct_w=round(total_p_20pct_w, 1),
        hover_current_0pct_a=round(ih_0pct_a, 1),
        hover_current_20pct_a=round(ih_20pct_a, 1),
        usable_max_thrust_per_motor_kgf=round(tmax_usable_kgf, 2),
        thrust_to_weight=round(tw_ratio, 2),
        hover_throttle_est=round(hover_throttle, 2),
        hover_time_0pct_min=round(th_0pct_min, 2),
        hover_time_20pct_min=round(th_20pct_min, 2),
        mission_3min2_batt_left_0pct=round(batt_left_0pct, 1),
        mission_3min2_batt_left_20pct=round(batt_left_20pct, 1),
        peak_current_total_a=round(peak_total_a, 1),
        battery_max_cont_current_a=round(batt_max_a, 1),
        esc_max_cont_current_a=round(esc_rating_a, 1),
        hover_rpm=round(hover_rpm, 0),
        max_rpm=round(max_rpm, 0),
        tip_speed_max_ms=round(tip_speed_max_ms, 1),
    )


def run_iterative_convergence_loop(use_retractable_legs: bool = True) -> List[Dict[str, float | str]]:
    """
    Executes the 3-round TA Sizing Convergence Loop shown on Slide 8 of the
    Design Review Presentation:
      Round 1: Worst-case initial guess m_0 = 2,000 g
      Round 2: Re-weighed BOM mass (1,904.9 g with legs or 1,825.2 g with skids)
      Round 3: Converged (Delta m / m = 0.0% < 2% threshold)
    """
    bom_summary = summarize_bom(get_locked_5in_bom(use_retractable_legs=use_retractable_legs))
    real_mass_g = bom_summary["all_up_mass_g"]

    masses = [2000.0, real_mass_g, real_mass_g]
    rounds = []
    for idx, m_g in enumerate(masses, start=1):
        prev_m = masses[idx - 2] if idx > 1 else m_g
        delta_pct = 0.0 if idx == 1 else 100.0 * (m_g - prev_m) / prev_m
        res = evaluate_propulsion_point(
            config_name=f"Round {idx}",
            all_up_mass_g=m_g,
            prop_d_mm=127.0,
            motor_kv=1700.0,
            motor_spec_max_thrust_kgf=1.98,
            motor_peak_current_a=40.0,
            cell_count_s=6,
            batt_capacity_mah=2600.0,
            batt_c_rating=65.0,
        )
        rounds.append({
            "round": idx,
            "assumed_or_bom_mass_g": round(m_g, 1),
            "delta_mass_pct": round(delta_pct, 2),
            "converged": idx > 1 and abs(delta_pct) < 2.0,
            "hover_thrust_per_motor_n": res.hover_thrust_per_motor_n,
            "ideal_power_per_motor_w": res.ideal_power_per_motor_w,
            "elec_power_per_motor_0pct_w": res.elec_power_per_motor_0pct_w,
            "total_hover_power_0pct_w": res.total_hover_power_0pct_w,
            "hover_current_0pct_a": res.hover_current_0pct_a,
            "hover_current_20pct_a": res.hover_current_20pct_a,
            "usable_thrust_per_motor_kgf": res.usable_max_thrust_per_motor_kgf,
            "thrust_to_weight": res.thrust_to_weight,
            "hover_time_0pct_min": res.hover_time_0pct_min,
            "hover_time_20pct_min": res.hover_time_20pct_min,
        })
    return rounds


def run_all_trade_studies() -> Dict[str, List[Dict]]:
    """
    Generates all comparative trade studies required for the grading report:
    1. Primary Configurations (Stage 1 Bare vs Stage 2 5" Skids vs Stage 2 5" Legs vs 4S Alt)
    2. Prop Size Trade Study (5-inch vs 6-inch vs 7-inch — Slide 6 table)
    3. Battery Capacity Sweep on 5-inch (2200 vs 2600 vs 3300 mAh)
    4. Payload Mass Sensitivity Sweep (0 g to 300 g in 50 g steps)
    5. 2-Blade vs 3-Blade 5-inch Prop Comparison
    6. Stage 0 Thrust-Stand Decision Gate Matrix (Slide 6 rules)
    """
    bom_skids = summarize_bom(get_locked_5in_bom(False))
    bom_legs = summarize_bom(get_locked_5in_bom(True))
    bom_stage1_with_payload = summarize_bom(get_locked_5in_bom(False), stage1_only=True)
    stage1_no_payload_g = bom_stage1_with_payload["all_up_mass_g"] - 180.0

    # 1. Primary configurations comparison
    configs = [
        evaluate_propulsion_point("Stage 1: Bare Quad (No Payload)", stage1_no_payload_g),
        evaluate_propulsion_point("Stage 1: Bare Quad (180g Payload)", bom_stage1_with_payload["all_up_mass_g"]),
        evaluate_propulsion_point("Stage 2: 5-inch Enclosed (Skids Baseline)", bom_skids["all_up_mass_g"]),
        evaluate_propulsion_point("Stage 2: 5-inch Enclosed (Retractable Legs)", bom_legs["all_up_mass_g"]),
        evaluate_propulsion_point(
            "Stage 2: 5-inch 4S 4000mAh Contingency",
            all_up_mass_g=1855.0,
            prop_d_mm=127.0,
            motor_kv=2400.0,
            motor_spec_max_thrust_kgf=1.64,
            motor_peak_current_a=45.0,
            cell_count_s=4,
            batt_capacity_mah=4000.0,
            batt_c_rating=60.0,
        ),
    ]

    # 2. Prop Size Trade Study (5" vs 6" vs 7" matching Slide 6 of Presentation)
    prop_sweep = [
        evaluate_propulsion_point(
            "5-inch (2207 1700KV, 6S 2600, Legs)",
            all_up_mass_g=1904.9,
            prop_d_mm=127.0,
            motor_kv=1700.0,
            motor_spec_max_thrust_kgf=1.98,
            motor_peak_current_a=40.0,
            cell_count_s=6,
            batt_capacity_mah=2600.0,
            batt_c_rating=65.0,
        ),
        evaluate_propulsion_point(
            "6-inch Interpolated (6S 2950, Legs)",
            all_up_mass_g=2320.0,
            prop_d_mm=152.4,
            motor_kv=1500.0,
            motor_spec_max_thrust_kgf=2.23,
            motor_peak_current_a=38.5,
            cell_count_s=6,
            batt_capacity_mah=2950.0,
            batt_c_rating=60.0,
        ),
        evaluate_propulsion_point(
            "7-inch Fallback (2807 1300KV, 6S 3300, Legs)",
            all_up_mass_g=2735.0,
            prop_d_mm=177.8,
            motor_kv=1300.0,
            motor_spec_max_thrust_kgf=2.48,
            motor_peak_current_a=36.8,
            cell_count_s=6,
            batt_capacity_mah=3300.0,
            batt_c_rating=60.0,
            ct_prop=0.14,
        ),
    ]

    # 3. Battery Capacity Sweep on 5-inch Skids Baseline (base without 400g battery = 1425.2 g)
    base_no_batt_g = bom_skids["all_up_mass_g"] - 400.0
    battery_sweep = []
    for cap_mah, batt_mass_g, c_rate in [(2200, 340.0, 65.0), (2600, 400.0, 65.0), (3300, 520.0, 60.0)]:
        res = evaluate_propulsion_point(
            f"5-inch on 6S {cap_mah} mAh ({int(batt_mass_g)}g)",
            all_up_mass_g=base_no_batt_g + batt_mass_g,
            batt_capacity_mah=cap_mah,
            batt_c_rating=c_rate,
        )
        battery_sweep.append(res.__dict__)

    # 4. Payload Mass Sensitivity Sweep (0 g to 300 g on 5-inch Skids Baseline)
    base_no_payload_g = bom_skids["all_up_mass_g"] - 180.0
    payload_sweep = []
    for p_g in [0, 50, 100, 150, 180, 200, 250, 300]:
        res = evaluate_propulsion_point(
            f"Payload {p_g} g",
            all_up_mass_g=base_no_payload_g + p_g,
        )
        d = res.__dict__.copy()
        d["payload_g"] = p_g
        payload_sweep.append(d)

    # 5. 2-Blade vs 3-Blade 5-inch Prop Comparison (Stage 0 test decision D3)
    blade_comparison = [
        {
            "blade_type": "3-Blade (5x4.3 Tri-Blade Baseline)",
            "ct": 0.125,
            "fm": 0.60,
            **evaluate_propulsion_point(
                "5-inch 3-Blade (5x4.3)", bom_skids["all_up_mass_g"], ct_prop=0.125
            ).__dict__,
        },
        {
            "blade_type": "2-Blade (5x4.5 Two-Blade Comparison)",
            "ct": 0.095,
            "fm": 0.63,
            **evaluate_propulsion_point(
                "5-inch 2-Blade (5x4.5)", bom_skids["all_up_mass_g"] - 2.3, ct_prop=0.095
            ).__dict__,
        },
    ]

    # 6. Stage 0 Thrust-Stand Decision Gate Matrix (Slide 6)
    stage0_gates = [
        {
            "measured_thrust_kgf_range": ">= 1.19 kgf",
            "tw_skids_1825g": round(4 * 1.19 / 1.8252, 2),
            "tw_legs_1905g": round(4 * 1.19 / 1.9049, 2),
            "decision": "Keep 5-inch as designed + Fit Retractable Legs (T/W >= 2.50 with legs)",
        },
        {
            "measured_thrust_kgf_range": "1.14 - 1.19 kgf",
            "tw_skids_1825g": round(4 * 1.14 / 1.8252, 2),
            "tw_legs_1905g": round(4 * 1.14 / 1.9049, 2),
            "decision": "Keep 5-inch with Flush TPU Skids + Cradle (or cut ~80g to fit legs)",
        },
        {
            "measured_thrust_kgf_range": "1.10 - 1.14 kgf",
            "tw_skids_1825g": round(4 * 1.10 / 1.8252, 2),
            "tw_legs_1905g": round(4 * 1.10 / 1.9049, 2),
            "decision": "Keep 5-inch, trim ~140 g (3mm grilles -39g, LW-PLA panels -75g, 1.2mm duct wall)",
        },
        {
            "measured_thrust_kgf_range": "0.95 - 1.10 kgf",
            "tw_skids_1825g": round(4 * 0.95 / 1.8252, 2),
            "tw_legs_1905g": round(4 * 0.95 / 1.9049, 2),
            "decision": "Accept T/W = 2.0-2.3 on 5-inch for slow hover/waypoint demo, or switch to 7-inch",
        },
    ]

    return {
        "bom_5in_skids_summary": bom_skids,
        "bom_5in_legs_summary": bom_legs,
        "bom_stage1_bare_summary": bom_stage1_with_payload,
        "convergence_loop_legs": run_iterative_convergence_loop(True),
        "convergence_loop_skids": run_iterative_convergence_loop(False),
        "primary_configurations": [c.__dict__ for c in configs],
        "prop_size_trade_study": [p.__dict__ for p in prop_sweep],
        "battery_capacity_sweep": battery_sweep,
        "payload_sensitivity_sweep": payload_sweep,
        "blade_count_comparison": blade_comparison,
        "stage0_gate_rules": stage0_gates,
        "crosswind_5in_skids": compute_crosswind_lean(
            get_locked_5in_geometry(1.5), bom_skids["all_up_mass_kg"], wind_speed_ms=4.17
        ),
    }


if __name__ == "__main__":
    results = run_all_trade_studies()
    out_dir = Path(__file__).resolve().parents[2] / "outputs" / "sizing"
    out_dir.mkdir(parents=True, exist_ok=True)
    out_file = out_dir / "sizing_and_trade_studies_5in.json"
    out_file.write_text(json.dumps(results, indent=2), encoding="utf-8")

    print("=== Locked 5-Inch BOM Summary ===")
    print("  Skids Baseline :", results["bom_5in_skids_summary"])
    print("  Legs Upgrade   :", results["bom_5in_legs_summary"])
    print("  Stage 1 Bare   :", results["bom_stage1_bare_summary"])
    print("\n=== Iterative Sizing Convergence (Slide 8 - Retractable Legs 1,905 g) ===")
    for r in results["convergence_loop_legs"]:
        print(f"  Round {r['round']}: m={r['assumed_or_bom_mass_g']}g (dm={r['delta_mass_pct']}%), "
              f"Th={r['hover_thrust_per_motor_n']}N, Pi={r['ideal_power_per_motor_w']}W, "
              f"Pe={r['elec_power_per_motor_0pct_w']}W, Ih={r['hover_current_0pct_a']}A, "
              f"T/W={r['thrust_to_weight']}, Hover={r['hover_time_0pct_min']}/{r['hover_time_20pct_min']} min")
    print(f"\nSaved full JSON results to: {out_file}")
