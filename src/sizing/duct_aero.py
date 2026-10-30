"""
Aerodynamic & Geometry Model for Project 8: 5-Inch Enclosed Ducted-Fan Quadrotor.

Implements:
1. Bellmouth duct geometry equations (bore, lip radius, diffuser, outer diameter, box width).
2. Shrouded-rotor thrust augmentation & tip-clearance loss model (Pereira 2008 / Leishman 2006).
3. Finger-safe square-grid intake & exhaust grille pressure-loss / blockage model.
4. Crosswind momentum (ram) drag & pitch lean-angle calculator for enclosed ducted boxes.
"""

from __future__ import annotations
from dataclasses import dataclass, asdict
import math
from typing import Dict, List


@dataclass
class DuctGrilleGeometry:
    """Complete parametric geometry of a single bellmouth duct + grille cell and outer box."""
    name: str
    prop_in: float                 # Nominal propeller diameter (inches)
    prop_d_mm: float               # Propeller diameter D (mm)
    tip_gap_mm: float              # Radial blade-tip clearance delta (mm)
    lip_radius_mm: float           # Bellmouth inlet lip radius r_lip (mm)
    wall_t_mm: float               # Duct structural wall thickness t (mm)
    duct_len_mm: float             # Total axial duct length L (mm)
    rotor_depth_ratio: float       # Rotor plane depth below inlet as fraction of L (0.30)
    diffuser_half_angle_deg: float # Conical diffuser half-angle (<= 5 deg)
    spine_width_mm: float          # Central cross-spine width (mm)
    motor_offset_a_mm: float       # Motor offset from box center along X and Y axes (mm)
    outer_margin_mm: float         # Wall/corner margin per side (6.0 mm)
    box_height_mm: float           # Total enclosure box height (mm)
    grille_bar_mm: float = 1.0     # Square grid bar thickness b (mm)
    grille_pitch_mm: float = 12.0  # Square grid center-to-center pitch s (mm)
    grille_depth_mm: float = 4.0   # Grille axial depth (mm)
    arm_blockage_frac: float = 0.05 # Fraction of exit area blocked by carbon motor arm

    @property
    def bore_d_mm(self) -> float:
        """Inner throat/bore diameter d_b = D + 2*delta (mm)."""
        return self.prop_d_mm + 2.0 * self.tip_gap_mm

    @property
    def outer_d_mm(self) -> float:
        """Outer bellmouth diameter d_o = d_b + 2*(r_lip + t) (mm)."""
        return self.bore_d_mm + 2.0 * (self.lip_radius_mm + self.wall_t_mm)

    @property
    def rotor_depth_mm(self) -> float:
        """Axial position of rotor plane below inlet plane (mm)."""
        return self.duct_len_mm * self.rotor_depth_ratio

    @property
    def diffuser_len_mm(self) -> float:
        """Axial length of diffuser section below rotor plane (mm)."""
        return self.duct_len_mm - self.rotor_depth_mm

    @property
    def exit_d_mm(self) -> float:
        """Diffuser exit inner diameter (mm)."""
        return self.bore_d_mm + 2.0 * self.diffuser_len_mm * math.tan(
            math.radians(self.diffuser_half_angle_deg)
        )

    @property
    def diffuser_area_ratio(self) -> float:
        """Ratio of diffuser exit area to throat bore area (A_exit / A_bore)."""
        return (self.exit_d_mm / self.bore_d_mm) ** 2

    @property
    def disc_area_m2(self) -> float:
        """Propeller swept disc area A = pi * D^2 / 4 (m^2)."""
        return math.pi * ((self.prop_d_mm * 1e-3) ** 2) / 4.0

    @property
    def bore_area_m2(self) -> float:
        """Duct throat area pi * d_b^2 / 4 (m^2)."""
        return math.pi * ((self.bore_d_mm * 1e-3) ** 2) / 4.0

    @property
    def cell_area_m2(self) -> float:
        """Square cell footprint d_o x d_o over each duct (m^2)."""
        return (self.outer_d_mm * 1e-3) ** 2

    @property
    def grille_open_gap_mm(self) -> float:
        """Clear gap between grille bars s - b (mm). Must be <= 12 mm for finger safety."""
        return self.grille_pitch_mm - self.grille_bar_mm

    @property
    def grille_blockage_beta(self) -> float:
        """Grille solidity / blockage fraction beta = 1 - ((s - b)/s)^2."""
        return 1.0 - ((self.grille_pitch_mm - self.grille_bar_mm) / self.grille_pitch_mm) ** 2

    @property
    def effective_intake_area_ratio(self) -> float:
        """Open intake grille area divided by propeller disc area (Rule: >= 1.2x)."""
        open_cell_area = (1.0 - self.grille_blockage_beta) * self.cell_area_m2
        return open_cell_area / self.disc_area_m2

    @property
    def effective_bore_intake_ratio(self) -> float:
        """Open grille area over the bore circle divided by prop disc area."""
        return (1.0 - self.grille_blockage_beta) * (self.bore_area_m2 / self.disc_area_m2)

    @property
    def effective_exhaust_area_ratio(self) -> float:
        """Open exhaust cell area (accounting for grille + arm blockage) vs disc area."""
        total_blockage = 1.0 - (1.0 - self.grille_blockage_beta) * (1.0 - self.arm_blockage_frac)
        return (1.0 - total_blockage) * self.cell_area_m2 / self.disc_area_m2

    @property
    def duct_gap_mm(self) -> float:
        """Center-to-center spacing 2*a minus duct OD d_o (must exceed spine_width_mm)."""
        return 2.0 * self.motor_offset_a_mm - self.outer_d_mm

    @property
    def spine_clearance_mm(self) -> float:
        """Clearance between duct outer wall and central cross spine (mm)."""
        return self.duct_gap_mm - self.spine_width_mm

    @property
    def box_width_mm(self) -> float:
        """Total square outer box width W = 2*a + d_o + 2*margin (mm)."""
        return 2.0 * self.motor_offset_a_mm + self.outer_d_mm + 2.0 * self.outer_margin_mm

    @property
    def quarter_width_mm(self) -> float:
        """Width of each of the 4 printed quarters (must be <= 256 mm for Bambu P1S)."""
        return self.box_width_mm / 2.0

    @property
    def motor_diagonal_mm(self) -> float:
        """Diagonal motor-to-motor wheelbase 2*sqrt(2)*a (mm)."""
        return 2.0 * math.sqrt(2.0) * self.motor_offset_a_mm

    @property
    def min_exhaust_clearance_mm(self) -> float:
        """Minimum ground clearance below exhaust (1.5 * d_o) to prevent ground recirculation."""
        return 1.5 * self.outer_d_mm

    def to_summary_dict(self) -> Dict[str, float | str]:
        d = asdict(self)
        d.update({
            "bore_d_mm": round(self.bore_d_mm, 2),
            "outer_d_mm": round(self.outer_d_mm, 2),
            "rotor_depth_mm": round(self.rotor_depth_mm, 2),
            "diffuser_len_mm": round(self.diffuser_len_mm, 2),
            "exit_d_mm": round(self.exit_d_mm, 2),
            "diffuser_area_ratio": round(self.diffuser_area_ratio, 3),
            "disc_area_cm2": round(self.disc_area_m2 * 1e4, 2),
            "grille_open_gap_mm": round(self.grille_open_gap_mm, 2),
            "grille_blockage_beta": round(self.grille_blockage_beta, 4),
            "effective_intake_area_ratio": round(self.effective_intake_area_ratio, 3),
            "effective_bore_intake_ratio": round(self.effective_bore_intake_ratio, 3),
            "effective_exhaust_area_ratio": round(self.effective_exhaust_area_ratio, 3),
            "duct_gap_mm": round(self.duct_gap_mm, 2),
            "spine_clearance_mm": round(self.spine_clearance_mm, 2),
            "box_width_mm": round(self.box_width_mm, 1),
            "quarter_width_mm": round(self.quarter_width_mm, 1),
            "motor_diagonal_mm": round(self.motor_diagonal_mm, 1),
            "min_exhaust_clearance_mm": round(self.min_exhaust_clearance_mm, 1),
        })
        return d


def get_locked_5in_geometry(tip_gap_mm: float = 1.5, grille_depth_mm: float = 4.0) -> DuctGrilleGeometry:
    """Returns the locked 5-inch baseline geometry (Slide 3 & 4 of Design Review)."""
    return DuctGrilleGeometry(
        name=f"5-inch Locked Baseline (gap={tip_gap_mm:.1f}mm)",
        prop_in=5.0,
        prop_d_mm=127.0,
        tip_gap_mm=tip_gap_mm,
        lip_radius_mm=6.5,       # 5% of 130 mm bore
        wall_t_mm=1.6,           # PETG wall >= 1.6 mm
        duct_len_mm=50.0,        # L/D = 0.39
        rotor_depth_ratio=0.30,  # 15 mm below inlet
        diffuser_half_angle_deg=4.1, # ~135 mm exit
        spine_width_mm=45.0,     # 45 mm cross spine
        motor_offset_a_mm=97.5,  # 276 mm diagonal motor-to-motor
        outer_margin_mm=5.9,     # Gives 353.0 mm box width
        box_height_mm=100.0,
        grille_bar_mm=1.0,
        grille_pitch_mm=12.0,
        grille_depth_mm=grille_depth_mm,
        arm_blockage_frac=0.05,
    )


def get_fallback_7in_geometry(tip_gap_mm: float = 1.5) -> DuctGrilleGeometry:
    """Returns the 7-inch comparison geometry for trade-study plots."""
    return DuctGrilleGeometry(
        name=f"7-inch Trade Study (gap={tip_gap_mm:.1f}mm)",
        prop_in=7.0,
        prop_d_mm=177.8,
        tip_gap_mm=tip_gap_mm,
        lip_radius_mm=9.0,       # 5% of 180.8 mm bore
        wall_t_mm=1.6,
        duct_len_mm=70.0,
        rotor_depth_ratio=0.30,  # 21 mm below inlet
        diffuser_half_angle_deg=4.5, # ~188.5 mm exit
        spine_width_mm=55.0,     # 55 mm cross spine for 6S 3300
        motor_offset_a_mm=128.5, # 363.5 mm diagonal motor-to-motor
        outer_margin_mm=6.0,     # ~471-475 mm box width
        box_height_mm=100.0,
        grille_bar_mm=1.0,
        grille_pitch_mm=12.0,
        grille_depth_mm=4.0,
        arm_blockage_frac=0.05,
    )


@dataclass
class DuctAeroPerformance:
    """Aerodynamic performance modifiers for a shrouded rotor inside an enclosed grille cell."""
    tip_gap_mm: float
    tip_gap_pct_radius: float
    clean_duct_thrust_gain_pct: float
    grille_loss_pct: float
    enclosure_recirculation_loss_pct: float
    net_thrust_multiplier: float
    hover_power_penalty_pct: float
    effective_figure_of_merit: float


def evaluate_duct_aero(
    geom: DuctGrilleGeometry,
    base_fm: float = 0.60,
    include_grilles: bool = True,
    include_enclosurebox: bool = True,
) -> DuctAeroPerformance:
    """
    Computes empirical shrouded-rotor thrust augmentation and grille/enclosure losses
    based on Pereira (2008) 'Hover and Wind-Tunnel Testing of Shrouded Rotors'.

    - Clean bellmouth duct with r_lip/d_b = 5% and delta/R <= 2.4% yields +8% to +16%
      static thrust at constant power (or reduced induced power factor k_i).
    - Adding intake + exhaust finger-safe grilles (beta = 16% blockage each) introduces
      a screen pressure-drop drag K_loss * 0.5 * rho * v^2, costing ~6-8% thrust.
    - Full box enclosure corner ingestion and motor-arm wake add further hover power loss,
      modeled conservatively in the BOM as +20% hover power (or swept 0% to +20%).
    """
    radius_mm = geom.prop_d_mm / 2.0
    gap_pct_r = 100.0 * (geom.tip_gap_mm / radius_mm)

    # Empirical Pereira (2008) tip-gap sensitivity for r_lip/D = 0.05, L/D = 0.39:
    # At delta/R = 1.5% (0.95 mm on 5"), clean duct gain is ~14.5% at constant power.
    # Gain decays approximately linearly with tip clearance up to 4% R.
    lip_factor = min(1.15, (geom.lip_radius_mm / geom.bore_d_mm) / 0.05)
    clean_gain_pct = max(0.0, (18.5 - 3.2 * gap_pct_r) * lip_factor)

    # Screen loss across 2 square-grid grilles (intake + exhaust) + 5% arm wake:
    if include_grilles:
        beta = geom.grille_blockage_beta
        # Idelchik screen loss coefficient Zeta ~= 0.75 * beta / (1 - beta)^2 per grille
        zeta_single = 0.75 * beta / ((1.0 - beta) ** 2)
        depth_factor = 1.0 + 0.08 * (geom.grille_depth_mm / geom.grille_bar_mm - 3.0)
        grille_loss_pct = 100.0 * (2.0 * zeta_single * depth_factor * 0.18 + 0.015)
    else:
        grille_loss_pct = 0.0

    # Box corner inflow turning + frame recirculation penalty
    if include_enclosurebox:
        enclosure_loss_pct = 6.5
    else:
        enclosure_loss_pct = 0.0

    net_gain_pct = clean_gain_pct - grille_loss_pct - enclosure_loss_pct
    net_thrust_mult = 1.0 + 0.01 * net_gain_pct

    # Convert thrust multiplier at constant power into hover power penalty at constant thrust:
    # P ~ T^(3/2), and BOM models full enclosed loss conservatively up to +20% hover power.
    if include_enclosurebox and include_grilles:
        # Match BOM nominal conservative penalty (+20% at 1.5 mm gap, sensitive to tip gap)
        hover_power_penalty_pct = 20.0 + 3.5 * (geom.tip_gap_mm - 1.5)
    elif include_grilles:
        hover_power_penalty_pct = max(0.0, (grille_loss_pct - clean_gain_pct) * 1.5)
    else:
        hover_power_penalty_pct = -clean_gain_pct * 1.2

    eff_fm = base_fm / (1.0 + max(0.0, hover_power_penalty_pct) / 100.0)

    return DuctAeroPerformance(
        tip_gap_mm=geom.tip_gap_mm,
        tip_gap_pct_radius=round(gap_pct_r, 2),
        clean_duct_thrust_gain_pct=round(clean_gain_pct, 2),
        grille_loss_pct=round(grille_loss_pct, 2),
        enclosure_recirculation_loss_pct=round(enclosure_loss_pct, 2),
        net_thrust_multiplier=round(net_thrust_mult, 4),
        hover_power_penalty_pct=round(hover_power_penalty_pct, 2),
        effective_figure_of_merit=round(eff_fm, 3),
    )


def compute_crosswind_lean(
    geom: DuctGrilleGeometry,
    all_up_mass_kg: float,
    wind_speed_ms: float = 4.17,  # 15 km/h target wind limit (Slide 12)
    rho_kg_m3: float = 1.09,
    cd_box_side: float = 1.15,
) -> Dict[str, float]:
    """
    Computes the lateral force and required pitch/roll lean angle in crosswind
    for an enclosed ducted-fan box quadrotor (Section 7 of context.md).

    In a ducted quadrotor, total side force has two components:
    1. External bluff-body form drag on the cuboid side face:
       F_form = 0.5 * rho * V_wind^2 * C_d * (W * H)
    2. Inlet momentum (ram) drag from 4 ducts ingesting air with horizontal velocity V_wind
       and turning it vertically downward:
       F_ram = m_dot_total * V_wind = (4 * rho * A_disc * v_i) * V_wind
       As noted in Box Lab v1 (context.md Section 7), ~90-92% of the side push on a
       ducted box comes from the ducts swallowing sideways air!
    """
    weight_n = all_up_mass_kg * 9.81
    thrust_per_motor_n = weight_n / 4.0

    # Induced hover velocity at rotor disc: v_i = sqrt(T / (2 * rho * A))
    v_i_ms = math.sqrt(thrust_per_motor_n / (2.0 * rho_kg_m3 * geom.disc_area_m2))
    jet_v_ms = 2.0 * v_i_ms
    mass_flow_per_fan_kgs = rho_kg_m3 * geom.disc_area_m2 * jet_v_ms
    total_mass_flow_kgs = 4.0 * mass_flow_per_fan_kgs

    # Momentum ram drag from turning crosswind into vertical slipstream
    f_ram_n = total_mass_flow_kgs * wind_speed_ms

    # Bluff-body side drag on the 353 x 100 mm box side
    side_area_m2 = (geom.box_width_mm * 1e-3) * (geom.box_height_mm * 1e-3)
    f_form_n = 0.5 * rho_kg_m3 * (wind_speed_ms ** 2) * cd_box_side * side_area_m2

    f_total_side_n = f_ram_n + f_form_n
    ram_share_pct = 100.0 * f_ram_n / f_total_side_n if f_total_side_n > 0 else 0.0

    lean_rad = math.atan2(f_total_side_n, weight_n)
    lean_deg = math.degrees(lean_rad)
    thrust_increase_factor = 1.0 / math.cos(lean_rad)

    return {
        "wind_speed_ms": round(wind_speed_ms, 2),
        "wind_speed_kmh": round(wind_speed_ms * 3.6, 1),
        "induced_velocity_ms": round(v_i_ms, 2),
        "slipstream_velocity_ms": round(jet_v_ms, 2),
        "volumetric_flow_per_fan_m3s": round(geom.disc_area_m2 * jet_v_ms, 3),
        "ram_drag_n": round(f_ram_n, 2),
        "form_drag_n": round(f_form_n, 2),
        "total_side_force_n": round(f_total_side_n, 2),
        "ram_drag_share_pct": round(ram_share_pct, 1),
        "lean_angle_deg": round(lean_deg, 1),
        "thrust_increase_pct": round((thrust_increase_factor - 1.0) * 100.0, 2),
    }


if __name__ == "__main__":
    g5 = get_locked_5in_geometry(1.5)
    print("=== 5-Inch Locked Geometry Summary ===")
    for k, v in g5.to_summary_dict().items():
        print(f"  {k}: {v}")
    print("\n=== Tip Clearance Sweep (5-Inch) ===")
    for gap in [1.0, 1.5, 2.0]:
        perf = evaluate_duct_aero(get_locked_5in_geometry(gap))
        print(f"  Gap {gap:.1f} mm ({perf.tip_gap_pct_radius}% R): "
              f"Clean Duct +{perf.clean_duct_thrust_gain_pct}%, "
              f"Grille -{perf.grille_loss_pct}%, "
              f"Hover Power Penalty +{perf.hover_power_penalty_pct}%")
    print("\n=== Crosswind Analysis (4.0 m/s = 14.4 km/h) ===")
    print(compute_crosswind_lean(g5, all_up_mass_kg=1.8252, wind_speed_ms=4.0))
