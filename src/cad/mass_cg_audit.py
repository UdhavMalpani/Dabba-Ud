"""
3-D Mass / CG / Inertia audit for Project 8 (5-inch enclosed ducted-fan quad).

Coordinate convention (ArduPilot body frame, FRD, right-handed):
    origin = geometric centre of the 353 x 353 x 100 mm box
    x = forward, y = right, z = DOWN       (positions below are typed as z_up and converted: z = -z_up)
Products of inertia are reported in tensor form:  I_xy = -sum(m x y)  (same for xz, yz).

Every BOM line of `get_locked_5in_bom()` is distributed over one or more bodies whose masses sum
exactly to the BOM *counted* mass (Est x 1.15; Spec/Allow/Defined x 1.0), so totals reproduce
1,825.2 g (skids) and 1,904.9 g (retractable legs).  Large bodies carry self-inertia
(boxes, thin cylindrical shells, rods, deck plates with the 4 duct holes subtracted as negative discs).

States evaluated:
    legs config  : Payload IN/OUT x Legs UP/DOWN (Stage 2 enclosed)
    skids config : Payload IN/OUT (Stage 2 enclosed)
    Stage 1 bare : no enclosure (items 20-25 removed), legs DOWN / skids
Ballast (item 18) x/y position is solved so the horizontal CG is exactly zero in the reference
flight state (Stage 2, payload IN, legs UP); every other state is then evaluated with the ballast fixed.

Outputs: outputs/sizing/mass_cg_inertia_audit.json, outputs/plots/fig9_mass_cg_inertia_audit.png
"""

from __future__ import annotations

import json
import math
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Tuple

import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
sys.path.insert(0, str(ROOT / "src" / "sizing"))
sys.path.insert(0, str(HERE))

from sizing_model import get_locked_5in_bom, summarize_bom, CONTINGENCY_EST  # noqa: E402
from duct_aero import get_locked_5in_geometry  # noqa: E402
import generate_cad_stl as cad  # noqa: E402

OUT_JSON = ROOT / "outputs" / "sizing" / "mass_cg_inertia_audit.json"
OUT_PNG = ROOT / "outputs" / "plots" / "fig9_mass_cg_inertia_audit.png"
CG_LIMIT_MM = 5.0

GEO = get_locked_5in_geometry(1.5)
A = GEO.motor_offset_a_mm                      # 97.5
BOX_W = GEO.box_width_mm                       # 353.0
HB = BOX_W / 2
BOX_H = GEO.box_height_mm                      # 100
SQ = [(1, 1), (1, -1), (-1, 1), (-1, -1)]

# ---- vertical stack-up (z_up, mm, box centre = 0) -------------------------------------------
Z_DUCT_EXIT = -25.0          # duct 50 mm tall centred in the 100 mm box
Z_ROTOR = Z_DUCT_EXIT + GEO.duct_len_mm - GEO.rotor_depth_mm   # +10
Z_FRAME = -15.0              # 4 mm carbon plate, top at -13 (= arm slot top, 12 mm above duct exit)
Z_BAY = -32.0                # 30 mm payload bay, -47 .. -17 (spine)
Z_BATT = +24.0               # 40 mm tall pack, +4 .. +44 on the X-arm above the FC/ESC stack
LEG_PIVOT = [(+125.0, +12.0, -54.0), (-125.0, -12.0, -62.0)]  # fwd leg folds aft, aft leg folds fwd (nested)
CAM_X = 30.0                 # RunCam Thumb at +30 mm (front bay wall window)
# Steel block 24 x 32 x 28 mm = 21.5 cm3 ~ 169 g (+1 g washers).  A 24 x 36 x 25 block was tried first but
# the legs-config null needs y = +3.0 mm (UBEC at y=-45 mm), which a 36 mm block cannot reach in the 40 mm bay.
BALLAST_DIMS = (24.0, 32.0, 28.0)
CAM_AFT_FACE_X = 23.5        # camera body ~13 mm deep behind the window


# ============================================================================
# Inertia helpers (g, mm -> g mm^2)
# ============================================================================
def I_box(m, dx, dy, dz):
    return m / 12.0 * np.diag([dy ** 2 + dz ** 2, dx ** 2 + dz ** 2, dx ** 2 + dy ** 2])


def I_cyl_shell_z(m, r, h):
    return np.diag([m * (r ** 2 / 2 + h ** 2 / 12), m * (r ** 2 / 2 + h ** 2 / 12), m * r ** 2])


def I_cyl_solid_z(m, r, h):
    return np.diag([m * (3 * r ** 2 + h ** 2) / 12, m * (3 * r ** 2 + h ** 2) / 12, m * r ** 2 / 2])


def I_disc_z(m, r):
    return np.diag([m * r ** 2 / 4, m * r ** 2 / 4, m * r ** 2 / 2])


def I_rod(m, L, u):
    u = np.asarray(u, float)
    u = u / np.linalg.norm(u)
    return m * L ** 2 / 12.0 * (np.eye(3) - np.outer(u, u))


def I_point(m):
    return np.zeros((3, 3))


@dataclass
class Body:
    name: str
    item: str
    mass_g: float
    pos_zup: Tuple[float, float, float]
    I_self: np.ndarray = field(default_factory=lambda: np.zeros((3, 3)))
    group: str = ""

    @property
    def r_frd(self) -> np.ndarray:
        x, y, zu = self.pos_zup
        return np.array([x, y, -zu])


# ============================================================================
# Component model
# ============================================================================
def bom_counted(items) -> Dict[str, float]:
    return {it.item_id: it.raw_total_g * (CONTINGENCY_EST if it.basis == "Est" else 1.0) for it in items}


def duct_centroid_and_radius() -> Tuple[float, float]:
    shells, _ = cad.build_duct_ring(cad.prop_d_mm, 1.5)
    _, cen = cad.centroid_of(shells)
    # mass-weighted radius of gyration about own axis (thin-shell equivalent radius)
    num, V = 0.0, 0.0
    for s in shells:
        a, b, c = (s.v[s.f[:, k]] for k in range(3))
        v6 = np.einsum("ij,ij->i", a, np.cross(b, c))
        cc = (a + b + c) / 4.0
        num += np.sum(v6 / 6 * (cc[:, 0] ** 2 + cc[:, 1] ** 2))
        V += v6.sum() / 6
    return float(cen[2]), math.sqrt(num / V)


def leg_centroid_from_pivot() -> float:
    _, cen = cad.centroid_of(cad.build_retractable_leg())
    return float(cen[0])


DUCT_ZC_LOCAL, DUCT_RG = duct_centroid_and_radius()
LEG_D = leg_centroid_from_pivot()


def build_bodies(config: str, stage: int, payload_in: bool, legs_down: bool,
                 ballast_xy: Tuple[float, float]) -> List[Body]:
    items = get_locked_5in_bom(use_retractable_legs=(config == "legs"))
    m = bom_counted(items)
    B: List[Body] = []

    def add(name, item, mass, pos, I=None, group=""):
        B.append(Body(name, item, mass, pos, I if I is not None else np.zeros((3, 3)), group))

    # --- propulsion ---------------------------------------------------------------------------
    for sx, sy in SQ:
        add(f"motor{sx:+d}{sy:+d}", "1", m["1"] / 4, (sx * A, sy * A, -3.0), I_cyl_solid_z(m["1"] / 4, 14, 20), "propulsion")
        add(f"leads{sx:+d}{sy:+d}", "2", m["2"] / 4, (sx * A / 2, sy * A / 2, -12.0),
            I_rod(m["2"] / 4, A * math.sqrt(2), (sx, sy, 0)), "propulsion")
        mp = m["3"] / 4
        add(f"prop{sx:+d}{sy:+d}", "3", mp, (sx * A, sy * A, Z_ROTOR), np.diag([mp * 63.5 ** 2 / 6] * 2 + [mp * 63.5 ** 2 / 3]), "propulsion")
    add("ESC 4-in-1", "5", m["5"], (0, 0, -10.0), I_box(m["5"], 42, 42, 6), "propulsion")
    add("6S 2600 battery", "6", m["6"], (0, 0, Z_BATT), I_box(m["6"], 135, 43, 40), "propulsion")
    # --- avionics -----------------------------------------------------------------------------
    add("FC F405 V4", "10", m["10"], (0, 0, -2.0), I_box(m["10"], 37, 37, 5), "avionics")
    add("GPS M10Q", "11", m["11"], (0, 0, BOX_H / 2 + 47.0), I_box(m["11"], 25, 25, 8), "avionics")
    add("GPS mast", "12", m["12"], (0, 0, BOX_H / 2 + 22.0), I_rod(m["12"], 45, (0, 0, 1)), "avionics")
    add("RC receiver", "13", m["13"] / 2, (0, +60.0, +20.0), I_box(m["13"] / 2, 30, 20, 10), "avionics")
    add("Telemetry air", "13", m["13"] / 2, (0, -60.0, +20.0), I_box(m["13"] / 2, 30, 20, 10), "avionics")
    add("Buzzer", "16", m["16"], (-60.0, 0, -10.0), group="avionics")
    # --- payload ------------------------------------------------------------------------------
    if payload_in:
        add("RunCam Thumb", "17", m["17"], (CAM_X, 0, Z_BAY), I_box(m["17"], 13, 20, 20), "payload")
        add("Ballast", "18", m["18"], (ballast_xy[0], ballast_xy[1], Z_BAY), I_box(m["18"], *BALLAST_DIMS), "payload")
    # --- frame (true-X carbon) ----------------------------------------------------------------
    diag = 2 * math.sqrt(2) * A
    add("Frame centre plate", "19", 0.30 * m["19"], (0, 0, Z_FRAME), I_box(0.30 * m["19"], 70, 70, 4), "structure")
    for u in ((1, 1, 0), (1, -1, 0)):
        add(f"Frame arm {u[1]:+d}", "19", 0.35 * m["19"], (0, 0, Z_FRAME), I_rod(0.35 * m["19"], diag, u), "structure")
    # --- wiring / XT90 / fasteners (item 26) --------------------------------------------------
    add("XT90 + cap + battery lead", "26", 20.0 * CONTINGENCY_EST, (-80.0, 0, Z_BATT), I_box(23.0, 30, 20, 15), "structure")
    add("Power/signal harness", "26", 37.0 * CONTINGENCY_EST, (0, 0, -8.0), I_box(37 * 1.15, 80, 80, 10), "structure")
    for sx, sy in SQ:
        add(f"Fasteners{sx:+d}{sy:+d}", "26", 23.0 * CONTINGENCY_EST / 4, (sx * A, sy * A, Z_FRAME), group="structure")

    # --- enclosure (Stage 2 only) -------------------------------------------------------------
    if stage == 2:
        for sx, sy in SQ:
            md = m["20"] / 4
            add(f"duct{sx:+d}{sy:+d}", "20", md, (sx * A, sy * A, Z_DUCT_EXIT + DUCT_ZC_LOCAL),
                I_cyl_shell_z(md, DUCT_RG, GEO.duct_len_mm), "enclosure")
            for zz in (+1, -1):
                mg = m["22"] / 8
                add(f"grille{sx:+d}{sy:+d}{'T' if zz > 0 else 'B'}", "22", mg, (sx * A, sy * A, zz * (BOX_H / 2 - 2)),
                    I_box(mg, GEO.outer_d_mm, GEO.outer_d_mm, 4), "enclosure")
        mw = m["21"] / 4
        for s in (1, -1):
            add(f"side panel x{s:+d}", "21", mw, (s * (HB - 0.4), 0, 0), I_box(mw, 0.8, BOX_W, BOX_H), "enclosure")
            add(f"side panel y{s:+d}", "21", mw, (0, s * (HB - 0.4), 0), I_box(mw, BOX_W, 0.8, BOX_H), "enclosure")
        # decks (70 %), flange rails (20 %), insert bosses (10 %) of item 23
        r_hole = GEO.outer_d_mm / 2 + cad.POCKET_CLEAR_MM
        a_sq, a_hole = BOX_W ** 2, math.pi * r_hole ** 2
        a_bay = cad.PAYLOAD_L_MM * cad.PAYLOAD_W_MM
        for zz, bay in ((+1, False), (-1, True)):
            m_deck = 0.35 * m["23"]
            sigma = m_deck / (a_sq - 4 * a_hole - (a_bay if bay else 0))
            zc = zz * (BOX_H / 2 - 0.5)
            tag = "top" if zz > 0 else "bottom"
            add(f"{tag} deck (full square)", "23", sigma * a_sq, (0, 0, zc), I_box(sigma * a_sq, BOX_W, BOX_W, 1), "enclosure")
            for sx, sy in SQ:
                mh = -sigma * a_hole
                add(f"{tag} deck hole{sx:+d}{sy:+d}", "23", mh, (sx * A, sy * A, zc), I_disc_z(mh, r_hole), "enclosure")
            if bay:
                mb = -sigma * a_bay
                add("bottom deck bay opening", "23", mb, (0, 0, zc), I_box(mb, 60, 40, 1), "enclosure")
        mr = 0.20 * m["23"] / 4
        for s in (1, -1):
            add(f"flange rails y{s:+d}", "23", mr, (0, s * 23.3, 0), I_box(mr, 2 * 153, 1.6, BOX_H), "enclosure")
            add(f"flange rails x{s:+d}", "23", mr, (s * 23.3, 0, 0), I_box(mr, 1.6, 2 * 153, BOX_H), "enclosure")
        bosses = [(px, py) for px in (40, 170) for py in (40, 170)]
        mbo = 0.10 * m["23"] / 16
        for sx, sy in SQ:
            for px, py in bosses:
                add("insert boss", "23", mbo, (sx * px, sy * py, 0), I_rod(mbo, 98, (0, 0, 1)), "enclosure")
        # spine (item 24) split by CAD wall volume fractions
        ms = m["24"]
        for s in (1, -1):
            add(f"spine X-wall{s:+d}", "24", 0.583 * ms / 2, (0, s * 21.7, Z_BAY), I_box(0.583 * ms / 2, 250, 1.6, 30), "enclosure")
            add(f"bay end wall{s:+d}", "24", 0.087 * ms / 2, (s * 30.8, 0, Z_BAY), I_box(0.087 * ms / 2, 1.6, 41.8, 30), "enclosure")
            add(f"servo bulkhead{s:+d}", "24", 0.104 * ms / 2, (s * 124.2, 0, Z_BAY), I_box(0.104 * ms / 2, 1.6, 41.8, 30), "enclosure")
            for t in (1, -1):
                add(f"spine Y-rail{s:+d}{t:+d}", "24", 0.226 * ms / 4, (s * 21.7, t * 73.75, -39.5),
                    I_box(0.226 * ms / 4, 1.6, 102.5, 15), "enclosure")
        md = m["25"]
        add("bay door", "25", md * (25.0 / 30.0), (0, 0, -49.0), I_box(md * 25 / 30, 60, 40, 2), "enclosure")
        add("door hinge (aft)", "25", md * (2.5 / 30.0), (-31.0, 0, -48.0), group="enclosure")
        add("door latch (fwd)", "25", md * (2.5 / 30.0), (+31.0, 0, -48.0), group="enclosure")

    # --- landing gear -------------------------------------------------------------------------
    if config == "skids":
        for sx, sy in SQ:
            add(f"TPU skid{sx:+d}{sy:+d}", "27", m["27"] / 4, (sx * 160.0, sy * 160.0, -52.0), I_box(m["27"] / 4, 20, 20, 4), "gear")
    else:
        for k, (px, py, pz) in enumerate(LEG_PIVOT):
            inward = -np.sign(px)
            add(f"MG90S servo {k}", "A1", m["A1"] / 2, (px + inward * 10.0, py, Z_BAY),
                I_box(m["A1"] / 2, 23, 12, 29), "gear")
            add(f"servo mount {k}", "A4", m["A4"] / 2, (px + inward * 10.0, py, Z_BAY + 5), group="gear")
            ml = m["A3"] / 2
            if legs_down:
                add(f"leg {k} (DOWN)", "A3", ml, (px, py, pz - LEG_D), I_rod(ml, cad.LEG_LEN_MM, (0, 0, 1)), "gear")
            else:
                add(f"leg {k} (UP)", "A3", ml, (px + inward * LEG_D, py, pz), I_rod(ml, cad.LEG_LEN_MM, (1, 0, 0)), "gear")
        add("UBEC 5V 3A", "A2", m["A2"], (0, -45.0, -40.0), I_box(m["A2"], 30, 18, 8), "gear")

    # Stage 1 bare keeps only in_stage1_bare items
    if stage == 1:
        keep = {it.item_id for it in items if it.in_stage1_bare}
        B = [b for b in B if b.item in keep]
    return B


# ============================================================================
# Mass properties
# ============================================================================
def mass_props(B: List[Body]) -> Dict:
    M = sum(b.mass_g for b in B)
    r = np.array([b.r_frd for b in B])
    mm = np.array([b.mass_g for b in B])
    cg = (mm[:, None] * r).sum(0) / M
    I = np.zeros((3, 3))
    for b, ri in zip(B, r):
        d = ri - cg
        I += b.I_self + b.mass_g * (np.dot(d, d) * np.eye(3) - np.outer(d, d))
    I_si = I * 1e-9          # g mm^2 -> kg m^2
    w, V = np.linalg.eigh(I_si)
    return {"mass_g": M, "cg_frd_mm": cg, "I_kgm2": I_si, "principal_kgm2": w, "principal_axes": V}


def solve_ballast(config: str) -> Tuple[float, float]:
    """Ballast (x,y) that zeroes the horizontal CG in Stage 2, payload IN, legs UP."""
    B = build_bodies(config, 2, True, False, (0.0, 0.0))
    mb = next(b.mass_g for b in B if b.item == "18")
    mx = sum(b.mass_g * b.r_frd[0] for b in B)
    my = sum(b.mass_g * b.r_frd[1] for b in B)
    return -mx / mb, -my / mb


def fmt_tensor(I: np.ndarray) -> Dict[str, float]:
    return {"Ixx": I[0, 0], "Iyy": I[1, 1], "Izz": I[2, 2], "Ixy": I[0, 1], "Ixz": I[0, 2], "Iyz": I[1, 2]}


def main() -> Dict:
    # ---- BOM reproduction check --------------------------------------------------------------
    print("=" * 96)
    print("Project 8 mass / CG / inertia audit   (FRD body axes: x fwd, y right, z down; origin = box centre)")
    print("=" * 96)
    bom_check = {}
    for config in ("skids", "legs"):
        items = get_locked_5in_bom(config == "legs")
        tgt = bom_counted(items)
        B = build_bodies(config, 2, True, False, (0, 0))
        per = {}
        for b in B:
            per[b.item] = per.get(b.item, 0.0) + b.mass_g
        worst = max(abs(per.get(k, 0) - v) for k, v in tgt.items())
        tot = sum(b.mass_g for b in B)
        s2 = summarize_bom(items)["all_up_mass_g"]
        s1 = summarize_bom(items, stage1_only=True)["all_up_mass_g"]
        B1 = build_bodies(config, 1, True, True, (0, 0))
        tot1 = sum(b.mass_g for b in B1)
        ok = worst < 1e-6 and abs(tot - s2) < 0.01 and abs(tot1 - s1) < 0.01
        bom_check[config] = {"bodies": len(B), "model_total_g": round(tot, 3), "bom_all_up_g": s2,
                             "stage1_model_g": round(tot1, 3), "stage1_bom_g": s1,
                             "max_per_item_error_g": worst, "reproduces_bom": bool(ok)}
        print(f"BOM check [{config:5s}]: {len(B)} bodies, model {tot:8.2f} g vs BOM {s2:8.2f} g | "
              f"Stage1 model {tot1:8.2f} g vs BOM {s1:8.2f} g | max item err {worst:.2e} g -> {'OK' if ok else 'FAIL'}")

    # ---- ballast ------------------------------------------------------------------------------
    cam_only = -(9.8 * CAM_X) / 170.2
    ballast = {c: solve_ballast(c) for c in ("legs", "skids")}
    half = BALLAST_DIMS[0] / 2
    x_lo, x_hi = -cad.PAYLOAD_L_MM / 2 + half, CAM_AFT_FACE_X - 0.5 - half
    print(f"\nBallast placement (170.2 g block {BALLAST_DIMS[0]:.0f} x {BALLAST_DIMS[1]:.0f} x {BALLAST_DIMS[2]:.0f} mm, bay z = {-Z_BAY:+.0f} mm FRD):")
    print(f"  camera-only counterbalance:  x = -(9.8 x 30)/170.2 = {cam_only:+.2f} mm  (plan text quoted ~ -1.0 mm)")
    for c, (bx, by) in ballast.items():
        feas = x_lo <= bx <= x_hi and abs(by) <= (cad.PAYLOAD_W_MM - BALLAST_DIMS[1]) / 2 + 1e-9
        print(f"  whole-aircraft null [{c:5s}]: x = {bx:+.2f} mm, y = {by:+.2f} mm  "
              f"(feasible x range {x_lo:+.1f}..{x_hi:+.1f}, |y| <= {(cad.PAYLOAD_W_MM-BALLAST_DIMS[1])/2:.1f}) -> {'fits' if feas else 'DOES NOT FIT'}")

    # ---- states -------------------------------------------------------------------------------
    states = []
    defs = [("legs", 2, True, False), ("legs", 2, True, True), ("legs", 2, False, False), ("legs", 2, False, True),
            ("skids", 2, True, None), ("skids", 2, False, None),
            ("legs", 1, True, True), ("legs", 1, True, False), ("skids", 1, True, None)]
    print(f"\n{'State':<44}{'Mass g':>9}{'x_cg':>8}{'y_cg':>8}{'z_cg':>8}{'|r_h|':>8}{'|r_3D|':>8}  CG<=5mm(h)")
    print("-" * 104)
    for config, stage, pin, ldown in defs:
        bxy = ballast[config]
        B = build_bodies(config, stage, pin, bool(ldown), bxy)
        mp = mass_props(B)
        cg = mp["cg_frd_mm"]
        rh = float(np.hypot(cg[0], cg[1]))
        gear = "skids" if config == "skids" else f"legs {'DOWN' if ldown else 'UP'}"
        label = f"Stage {stage} {'enclosed' if stage == 2 else 'bare'} | {gear} | payload {'IN' if pin else 'OUT'}"
        st = {"label": label, "config": config, "stage": stage, "payload_in": pin, "legs_down": ldown,
              "mass_g": round(mp["mass_g"], 2), "cg_frd_mm": [round(float(c), 3) for c in cg],
              "cg_horizontal_mm": round(rh, 3), "cg_3d_mm": round(float(np.linalg.norm(cg)), 3),
              "cg_below_box_centre_mm": round(float(cg[2]), 3),
              "pass_horizontal_5mm": rh <= CG_LIMIT_MM,
              "inertia_kgm2": {k: float(f"{v:.6g}") for k, v in fmt_tensor(mp["I_kgm2"]).items()},
              "principal_kgm2": [float(f"{v:.6g}") for v in mp["principal_kgm2"]],
              "max_offdiag_over_min_diag": float(np.max(np.abs(mp["I_kgm2"] - np.diag(np.diag(mp["I_kgm2"])))) / np.min(np.diag(mp["I_kgm2"]))),
              "_xy": [(b.r_frd[0], b.r_frd[1], b.mass_g, b.group) for b in B]}
        states.append(st)
        print(f"{label:<44}{mp['mass_g']:>9.1f}{cg[0]:>8.2f}{cg[1]:>8.2f}{cg[2]:>8.2f}{rh:>8.2f}"
              f"{np.linalg.norm(cg):>8.2f}  {'PASS' if rh <= CG_LIMIT_MM else 'FAIL'}")

    # ---- inertia headline ---------------------------------------------------------------------
    pick = {"Stage 1 Bare (legs DOWN, payload IN)": states[6], "Stage 2 Enclosed skids (payload IN)": states[4],
            "Stage 2 Enclosed legs UP (payload IN)": states[0], "Stage 2 Enclosed legs DOWN (payload IN)": states[1]}
    print(f"\n{'Inertia about CG [kg m^2]':<42}{'mass g':>8}{'Ixx':>10}{'Iyy':>10}{'Izz':>10}{'Ixy':>11}{'Ixz':>11}{'Iyz':>11}")
    for k, s in pick.items():
        I = s["inertia_kgm2"]
        print(f"{k:<42}{s['mass_g']:>8.1f}{I['Ixx']:>10.5f}{I['Iyy']:>10.5f}{I['Izz']:>10.5f}"
              f"{I['Ixy']:>11.2e}{I['Ixz']:>11.2e}{I['Iyz']:>11.2e}")
    s1 = states[6]["mass_g"]
    print(f"\nStage 1 Bare mass from BOM = {s1:.2f} g (legs) / {states[8]['mass_g']:.2f} g (skids); "
          f"plan quoted ~1,160 g (1.161 kg) -> difference {s1-1161:+.1f} g")

    all_pass = all(s["pass_horizontal_5mm"] for s in states)
    worst = max(states, key=lambda s: s["cg_horizontal_mm"])
    print(f"All {len(states)} states |r_CG,h| <= {CG_LIMIT_MM} mm: {all_pass} (worst {worst['cg_horizontal_mm']:.2f} mm: {worst['label']})")

    # ---- JSON ---------------------------------------------------------------------------------
    ref_bodies = build_bodies("legs", 2, True, False, ballast["legs"])
    out = {
        "convention": "FRD body axes (x forward, y right, z down), origin at geometric centre of 353x353x100 box; "
                      "inertia about the state CG in kg m^2, tensor form (I_xy = -sum m x y)",
        "bom_reproduction": bom_check,
        "ballast": {"mass_g": 170.2, "block_mm": BALLAST_DIMS,
                    "camera_only_null_x_mm": round(cam_only, 3), "plan_quoted_x_mm": -1.0,
                    "whole_aircraft_null_legs_xy_mm": [round(v, 3) for v in ballast["legs"]],
                    "whole_aircraft_null_skids_xy_mm": [round(v, 3) for v in ballast["skids"]],
                    "feasible_x_range_mm": [x_lo, x_hi]},
        "layout_assumptions": {
            "ducts_motors": f"(+/-{A}, +/-{A}); duct exit z_up={Z_DUCT_EXIT}, duct CG {DUCT_ZC_LOCAL:.1f} mm above exit (from STL), "
                            f"radius of gyration {DUCT_RG:.1f} mm; rotor plane z_up={Z_ROTOR}",
            "frame": f"carbon true-X at z_up={Z_FRAME}: 30 % centre plate, 2 x 35 % diagonal rods 275.8 mm",
            "battery": f"135x43x40 mm on X-spine at z_up={Z_BATT} above FC (z_up -2) / ESC (z_up -10)",
            "gps": "on 45 mm mast at box centre, GPS z_up=+97",
            "rc_telemetry": "15 g each at y = +/-60 mm (balanced across Y-spine)",
            "asymmetric_items": "XT90+cap+lead 23 g at x=-80 (aft battery lead), buzzer at x=-60, UBEC at y=-45, "
                                "door hinge aft / latch fwd",
            "payload": f"RunCam at x=+{CAM_X}, bay centre z_up={Z_BAY}; ballast solved for zero horizontal CG",
            "decks": "353x353 plates minus 4 duct holes (r=73.35, negative-mass discs) minus 60x40 bay opening",
            "legs": f"pivots {LEG_PIVOT} (z_up); UP = folded flat under X-spine pointing inboard (nested, 8 mm stagger), "
                    f"DOWN = vertical; leg CG {LEG_D:.1f} mm from pivot (from STL)",
            "stage1": "items 20-25 removed (ducts, panels, grilles, decks, spine, door); same component positions",
        },
        "states": [{k: v for k, v in s.items() if k != "_xy"} for s in states],
        "inertia_headline": {k: {"mass_g": s["mass_g"], **s["inertia_kgm2"]} for k, s in pick.items()},
        "stage1_bare_mass_g": {"legs": states[6]["mass_g"], "skids": states[8]["mass_g"], "plan_quoted_g": 1161.0,
                               "difference_legs_g": round(states[6]["mass_g"] - 1161.0, 2)},
        "all_states_cg_within_5mm": bool(all_pass),
        "reference_state_bodies": [{"name": b.name, "item": b.item, "mass_g": round(b.mass_g, 3),
                                    "pos_frd_mm": [round(float(v), 2) for v in b.r_frd]} for b in ref_bodies],
    }
    OUT_JSON.parent.mkdir(parents=True, exist_ok=True)
    OUT_JSON.write_text(json.dumps(out, indent=2, default=lambda o: bool(o) if isinstance(o, np.bool_) else float(o)))
    print(f"Saved {OUT_JSON}")
    make_plot(states, ballast["legs"], pick)
    print(f"Saved {OUT_PNG}")
    return out


# ============================================================================
# Figure 9
# ============================================================================
def make_plot(states, ballast_xy, pick):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Circle, Rectangle

    plt.rcParams.update({"font.family": "sans-serif", "font.size": 9, "axes.titlesize": 10.5,
                         "axes.titleweight": "bold", "figure.dpi": 180})
    fig, (ax1, ax2, ax3) = plt.subplots(1, 3, figsize=(16, 5.4), gridspec_kw={"width_ratios": [1.15, 1, 1.25]})
    colors = {"propulsion": "#b45309", "avionics": "#2563eb", "payload": "#dc2626", "structure": "#525252",
              "enclosure": "#0f766e", "gear": "#7c3aed"}

    # (a) top view: plot horizontal = y (right), vertical = x (forward)
    ax1.add_patch(Rectangle((-HB, -HB), BOX_W, BOX_W, fill=False, lw=1.5, ec="k"))
    for sx, sy in SQ:
        ax1.add_patch(Circle((sy * A, sx * A), GEO.outer_d_mm / 2, fill=False, ec="#0f766e", lw=1.2))
        ax1.add_patch(Circle((sy * A, sx * A), GEO.bore_d_mm / 2, fill=False, ec="#0f766e", lw=0.6, ls="--"))
    ax1.add_patch(Rectangle((-22.5, -125), 45, 250, fc="#e5e7eb", ec="#9ca3af", lw=0.8, zorder=0))
    ax1.add_patch(Rectangle((-125, -22.5), 250, 45, fc="#e5e7eb", ec="#9ca3af", lw=0.8, zorder=0))
    ax1.add_patch(Rectangle((-20, -30), 40, 60, fc="none", ec="#dc2626", lw=1.0, ls="--"))
    ax1.add_patch(Rectangle((-21.5, -67.5), 43, 135, fc="#fde68a", ec="#b45309", lw=0.8, alpha=0.6, zorder=0.5))
    seen = set()
    for x, y, m, g in states[0]["_xy"]:
        if m <= 0:
            continue
        ax1.scatter(y, x, s=max(6, m * 0.9), c=colors.get(g, "k"), alpha=0.75, edgecolors="white", lw=0.5,
                    label=g if g not in seen else None, zorder=3)
        seen.add(g)
    ax1.annotate("ballast", (ballast_xy[1], ballast_xy[0]), xytext=(45, -95), fontsize=8, arrowprops=dict(arrowstyle="->", lw=0.7))
    ax1.annotate("RunCam", (0, CAM_X), xytext=(40, 60), fontsize=8, arrowprops=dict(arrowstyle="->", lw=0.7))
    ax1.annotate("battery", (0, 0), xytext=(-150, -150), fontsize=8, arrowprops=dict(arrowstyle="->", lw=0.7))
    ax1.set_xlim(-200, 200); ax1.set_ylim(-200, 200); ax1.set_aspect("equal")
    ax1.set_xlabel("y (right) [mm]"); ax1.set_ylabel("x (forward) [mm]")
    ax1.set_title("(a) Top view - component masses (legs UP, payload IN)")
    lg = ax1.legend(loc="upper left", fontsize=7, framealpha=0.9)
    for h in lg.legend_handles:
        h.set_sizes([30])
    ax1.grid(alpha=0.25)

    # (b) CG zoom
    ax2.add_patch(Circle((0, 0), CG_LIMIT_MM, fc="#dcfce7", ec="#16a34a", lw=1.5, label="|r_CG| = 5 mm limit"))
    marks = ["o", "s", "^", "v", "D", "P", "X", "*", "h"]
    for k, s in enumerate(states):
        x, y, z = s["cg_frd_mm"]
        ax2.scatter(y, x, marker=marks[k % len(marks)], s=55, zorder=3,
                    label=f"{s['label']}  (z={z:+.1f})")
    ax2.set_xlim(-6, 6); ax2.set_ylim(-6, 6); ax2.set_aspect("equal")
    ax2.axhline(0, c="k", lw=0.5); ax2.axvline(0, c="k", lw=0.5)
    ax2.set_xlabel("y_CG (right) [mm]"); ax2.set_ylabel("x_CG (forward) [mm]")
    ax2.set_title("(b) Horizontal CG in every state (z_CG FRD, +down)")
    ax2.legend(fontsize=5.6, loc="lower center", bbox_to_anchor=(0.5, -0.62), ncol=1, framealpha=0.9)
    ax2.grid(alpha=0.25)

    # (c) inertia bars
    labels = ["Stage 1 Bare\n(legs DN)", "Stage 2\nskids", "Stage 2\nlegs UP", "Stage 2\nlegs DOWN"]
    vals = [[s["inertia_kgm2"][k] * 1e3 for k in ("Ixx", "Iyy", "Izz")] for s in pick.values()]
    xs = np.arange(len(labels))
    for j, (k, c) in enumerate(zip(("Ixx", "Iyy", "Izz"), ("#2563eb", "#0f766e", "#b45309"))):
        bars = ax3.bar(xs + (j - 1) * 0.26, [v[j] for v in vals], 0.26, label=k, color=c)
        ax3.bar_label(bars, fmt="%.1f", fontsize=6.5, padding=1)
    for i, s in enumerate(pick.values()):
        ax3.text(xs[i], -2.2, f"{s['mass_g']:.0f} g", ha="center", fontsize=7.5, color="#374151")
    ax3.set_xticks(xs); ax3.set_xticklabels(labels, fontsize=8)
    ax3.set_ylabel("Moment of inertia about CG [g m^2  = 1e-3 kg m^2]")
    ax3.set_ylim(-3.5, max(max(v) for v in vals) * 1.15)
    worst_po = max(s["max_offdiag_over_min_diag"] for s in pick.values())
    ax3.set_title(f"(c) Inertia about CG (|products| <= {100*worst_po:.1f} % of smallest diagonal)")
    ax3.legend(fontsize=8)
    ax3.grid(axis="y", alpha=0.25)
    fig.suptitle("Fig. 9 - Project 8 5-inch mass / CG / inertia audit (FRD axes, origin = box centre)", fontweight="bold")
    fig.tight_layout()
    OUT_PNG.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(OUT_PNG, bbox_inches="tight")
    plt.close(fig)


if __name__ == "__main__":
    main()
