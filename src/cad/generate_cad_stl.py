"""
Pure-Python + NumPy binary STL generator for every printed part of the
Project 8 5-inch enclosed ducted-fan quadrotor (EC4.402, IIIT Hyderabad).

No CAD kernel is required.  Every part is assembled from closed, consistently
oriented triangle shells (revolved 2-D profiles, extruded polygons, rectilinear
cell extrusions).  Shells inside one part never overlap -- they only touch on
shared faces -- so the part volume is the exact sum of the shell volumes and a
slicer (Bambu Studio) merges them into one solid.

Outputs (outputs/stl/):
    duct_ring_5in_gap1.0mm.stl / gap1.5mm / gap2.0mm   (PETG, Stage 0 + flight)
    grille_5in_146mm_4mm.stl                          (PLA)
    enclosure_quarter_5in_177mm.stl                   (PLA, print 2 + 2 mirrored)
    cross_spine_45mm_payload_bay.stl                  (PLA)
    retractable_leg_240mm.stl                         (PLA, print x2)
and outputs/sizing/cad_mass_comparison.json.

Hardware Sync Gate A: after measuring the real 5x4.3 tri-blade with calipers,
change `prop_d_mm` below (or run with --prop-d <mm>) and re-run.
"""

from __future__ import annotations

import argparse
import json
import math
import struct
import sys
from pathlib import Path
from typing import Dict, List, Sequence, Tuple

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
STL_DIR = ROOT / "outputs" / "stl"
SIZING_DIR = ROOT / "outputs" / "sizing"

# ============================================================================
# 1. Locked 5-inch parameters (mirror cad/fusion360_parameters_5in.csv)
# ============================================================================
prop_d_mm = 127.0              # <-- Hardware Sync Gate A: caliper-measured prop diameter
TIP_GAP_BASELINE_MM = 1.5      # flight ring radial tip gap
STAGE0_GAPS_MM = (1.0, 1.5, 2.0)

LIP_R_MM = 6.5                 # bellmouth lip radius (0.05 * 130 mm, held constant across rings)
WALL_T_MM = 1.6                # duct wall
DUCT_LEN_MM = 50.0             # axial length L
ROTOR_DEPTH_RATIO = 0.30       # rotor plane 15 mm below inlet
DIFF_HALF_ANGLE_DEG = 4.1      # diffuser half angle
ARM_SLOT_W_MM = 16.0           # carbon arm slot width (arm 14 mm + 1 mm/side)
ARM_SLOT_H_MM = 12.0           # slot height from duct exit plane (arm top at 12 mm)

MOTOR_OFFSET_A_MM = 97.5
OUTER_MARGIN_MM = 5.9
BOX_H_MM = 100.0
SPINE_W_MM = 45.0
PANEL_T_MM = 0.8               # side panel (BOM item 21: 0.8 mm PLA)
DECK_T_MM = 1.0                # top / bottom deck
FLANGE_T_MM = 1.6              # spine-joint flange rail
FLANGE_H_MM = 10.0
POCKET_CLEAR_MM = 0.25         # radial clearance of duct pocket in decks
TONGUE_D_MM, TONGUE_H_MM, TONGUE_Z0_MM = 0.7, 2.8, 3.2   # tongue (quarter) -> groove (spine)
BOSS_OD_MM, BOSS_ID_MM = 6.0, 4.0                         # M3 heat-set insert bosses

GRILLE_BAR_MM, GRILLE_PITCH_MM, GRILLE_DEPTH_MM, GRILLE_RIM_MM = 1.0, 12.0, 4.0, 3.0

PAYLOAD_L_MM, PAYLOAD_W_MM, PAYLOAD_H_MM = 60.0, 40.0, 30.0
SPINE_LEN_MM = 250.0           # printed central cross-spine section (fits 256 mm bed)
SPINE_WALL_MM = 1.6
SPINE_H_MM = 30.0              # X-arm / bay wall height (= payload bay height)
SPINE_Y_H_MM = 15.0            # Y-arm rail height
GROOVE_D_MM, GROOVE_Z0_MM, GROOVE_H_MM = 0.8, 0.9, 3.2
CAM_WIN_W_MM, CAM_WIN_H_MM = 20.0, 16.0

LEG_LEN_MM = 240.0             # pivot centre -> foot sole
LEG_W_ROOT_MM, LEG_W_TIP_MM, LEG_T_MM = 16.0, 10.0, 8.0
FOOT_LEN_MM, FOOT_W_MM = 100.0, 12.0
HORN_BOSS_OD_MM, HORN_BOSS_ID_MM, HORN_BOSS_H_MM = 16.0, 3.0, 4.0

BED_MM = (256.0, 256.0, 256.0)  # Bambu Lab P1S build volume

# Slicer model (Bambu Studio defaults, 0.4 mm nozzle): 2 walls x 0.42 mm line,
# top/bottom ~0.8-1.0 mm -> treat a 0.84 mm skin over the whole surface as solid,
# everything deeper is 15 % gyroid/grid infill.
SKIN_T_MM = 0.84
INFILL_FRAC = 0.15
MATERIALS = {"PETG": 1.27, "PLA": 1.24, "PA-CF": 1.20, "TPU": 1.21}

N_SEG = 192                    # angular segments for revolved bodies (1.875 deg)


# ============================================================================
# 2. Mesh primitives
# ============================================================================
class Shell:
    """Closed triangle shell: vertices (N,3) float64, faces (M,3) int."""

    def __init__(self, v, f, name: str = ""):
        self.v = np.asarray(v, dtype=float)
        self.f = np.asarray(f, dtype=np.int64).reshape(-1, 3)
        self.name = name

    def signed_volume(self) -> float:
        a, b, c = (self.v[self.f[:, k]] for k in range(3))
        return float(np.einsum("ij,ij->i", a, np.cross(b, c)).sum() / 6.0)

    def oriented(self) -> "Shell":
        if self.signed_volume() < 0:
            self.f = self.f[:, [0, 2, 1]]
        return self

    def moved(self, dx=0.0, dy=0.0, dz=0.0) -> "Shell":
        return Shell(self.v + np.array([dx, dy, dz]), self.f.copy(), self.name)

    def permuted(self, order: Sequence[int]) -> "Shell":
        """Re-map axes (e.g. (1,2,0) puts local x->y...).  Re-orients afterwards."""
        v = np.empty_like(self.v)
        for src, dst in enumerate(order):
            v[:, dst] = self.v[:, src]
        return Shell(v, self.f.copy(), self.name).oriented()

    def triangles(self) -> np.ndarray:
        return self.v[self.f]


def polygon_area(p: np.ndarray) -> float:
    x, y = p[:, 0], p[:, 1]
    return 0.5 * float(np.sum(x * np.roll(y, -1) - np.roll(x, -1) * y))


def _cross2(o, a, b) -> float:
    return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])


def triangulate_polygon(p: np.ndarray) -> List[Tuple[int, int, int]]:
    """Ear-clipping triangulation of a simple polygon given CCW.  Returns CCW triples."""
    p = np.asarray(p, float)
    idx = list(range(len(p)))
    if polygon_area(p) < 0:
        idx.reverse()
    tris: List[Tuple[int, int, int]] = []
    while len(idx) > 3:
        m = len(idx)
        for k in range(m):
            i0, i1, i2 = idx[k - 1], idx[k], idx[(k + 1) % m]
            a, b, c = p[i0], p[i1], p[i2]
            if _cross2(a, b, c) <= 1e-10:
                continue
            blocked = False
            for j in idx:
                if j in (i0, i1, i2):
                    continue
                q = p[j]
                if _cross2(a, b, q) >= -1e-12 and _cross2(b, c, q) >= -1e-12 and _cross2(c, a, q) >= -1e-12:
                    blocked = True
                    break
            if not blocked:
                tris.append((i0, i1, i2))
                idx.pop(k)
                break
        else:
            raise RuntimeError("ear clipping failed (non-simple polygon?)")
    tris.append((idx[0], idx[1], idx[2]))
    return tris


def extrude_polygon(poly2d, z0: float, z1: float, name: str = "") -> Shell:
    """Prism: simple polygon in XY extruded from z0 to z1."""
    p = np.asarray(poly2d, float)
    if polygon_area(p) < 0:
        p = p[::-1]
    n = len(p)
    v = np.vstack([np.c_[p, np.full(n, z0)], np.c_[p, np.full(n, z1)]])
    faces = []
    for a, b, c in triangulate_polygon(p):
        faces.append((a + n, b + n, c + n))
        faces.append((c, b, a))
    for i in range(n):
        j = (i + 1) % n
        faces += [(i, j, j + n), (i, j + n, i + n)]
    return Shell(v, faces, name).oriented()


def box(x0, x1, y0, y1, z0, z1, name: str = "") -> Shell:
    return extrude_polygon([(x0, y0), (x1, y0), (x1, y1), (x0, y1)], z0, z1, name)


def revolve(profiles, thetas, closed_loop: bool = True, name: str = "") -> Shell:
    """
    Revolve (r, z) profile(s) about the z axis.
    profiles: (m,2) or (K,m,2) -- one profile per angle (allows angle-dependent radii).
    closed_loop=True wraps 360 deg; otherwise a partial sector with planar end caps.
    """
    thetas = np.asarray(thetas, float)
    P = np.asarray(profiles, float)
    if P.ndim == 2:
        P = np.broadcast_to(P, (len(thetas),) + P.shape).copy()
    if polygon_area(P[0]) < 0:
        P = P[:, ::-1, :]
    K, m, _ = P.shape
    c, s = np.cos(thetas)[:, None], np.sin(thetas)[:, None]
    V = np.stack([P[:, :, 0] * c, P[:, :, 0] * s, P[:, :, 1]], axis=-1).reshape(-1, 3)
    kk = np.arange(K if closed_loop else K - 1)
    kn = (kk + 1) % K
    ii = np.arange(m)
    jj = (ii + 1) % m
    A = (kk[:, None] * m + ii[None, :]).ravel()
    B = (kn[:, None] * m + ii[None, :]).ravel()
    C = (kn[:, None] * m + jj[None, :]).ravel()
    D = (kk[:, None] * m + jj[None, :]).ravel()
    faces = np.vstack([np.c_[A, B, C], np.c_[A, C, D]])
    if not closed_loop:
        cap = np.array(triangulate_polygon(P[0]))
        faces = np.vstack([faces, cap, ((K - 1) * m + cap)[:, ::-1]])
    return Shell(V, faces, name).oriented()


def ray_polygon_distance(c, theta: float, poly: np.ndarray) -> float:
    d = np.array([math.cos(theta), math.sin(theta)])
    best = math.inf
    n = len(poly)
    for k in range(n):
        p, q = poly[k], poly[(k + 1) % n]
        e = q - p
        den = d[0] * e[1] - d[1] * e[0]
        if abs(den) < 1e-14:
            continue
        w = p - c
        t = (w[0] * e[1] - w[1] * e[0]) / den
        u = (w[0] * d[1] - w[1] * d[0]) / den
        if t > 1e-9 and -1e-9 <= u <= 1 + 1e-9:
            best = min(best, t)
    return best


def plate_with_round_hole(outer_poly, centre, r_hole, z0, z1, n=N_SEG, name="") -> Shell:
    """Plate = star-shaped outer polygon minus a circular hole, built as a radial sweep."""
    poly = np.asarray(outer_poly, float)
    c = np.asarray(centre, float)
    th = list(np.linspace(0, 2 * np.pi, n, endpoint=False))
    th += [math.atan2(p[1] - c[1], p[0] - c[0]) % (2 * np.pi) for p in poly]
    th = np.unique(np.round(np.sort(np.mod(th, 2 * np.pi)), 9))
    profs = []
    for t in th:
        R = ray_polygon_distance(c, t, poly)
        profs.append([(r_hole, z0), (R, z0), (R, z1), (r_hole, z1)])
    return revolve(profs, th, True, name).moved(c[0], c[1], 0.0)


def grid_extrude(xs, ys, solid: np.ndarray, z0: float, z1: float, name="") -> Shell:
    """Extrude the union of solid cells of a rectilinear grid (conforming, watertight)."""
    xs, ys = np.asarray(xs, float), np.asarray(ys, float)
    nx, ny = len(xs) - 1, len(ys) - 1
    NX, NY = np.meshgrid(xs, ys, indexing="ij")
    base = np.c_[NX.ravel(), NY.ravel()]
    N = len(base)
    v = np.vstack([np.c_[base, np.full(N, z0)], np.c_[base, np.full(N, z1)]])
    nid = lambda i, j: i * (ny + 1) + j
    faces = []
    for i in range(nx):
        for j in range(ny):
            if not solid[i, j]:
                continue
            ring = [nid(i, j), nid(i + 1, j), nid(i + 1, j + 1), nid(i, j + 1)]
            faces += [(ring[0] + N, ring[1] + N, ring[2] + N), (ring[0] + N, ring[2] + N, ring[3] + N)]
            faces += [(ring[2], ring[1], ring[0]), (ring[3], ring[2], ring[0])]
            nbrs = [(i, j - 1), (i + 1, j), (i, j + 1), (i - 1, j)]  # across edges 0-1,1-2,2-3,3-0
            for e, (a, b) in enumerate(nbrs):
                inside = 0 <= a < nx and 0 <= b < ny and solid[a, b]
                if not inside:
                    p, q = ring[e], ring[(e + 1) % 4]
                    faces += [(p, q, q + N), (p, q + N, p + N)]
    return Shell(v, faces, name).oriented()


# ============================================================================
# 3. Part builders
# ============================================================================
def duct_dims(prop_d: float, gap: float) -> Dict[str, float]:
    bore = prop_d + 2 * gap
    rotor_depth = DUCT_LEN_MM * ROTOR_DEPTH_RATIO
    diff_len = DUCT_LEN_MM - rotor_depth
    exit_d = bore + 2 * diff_len * math.tan(math.radians(DIFF_HALF_ANGLE_DEG))
    return {"bore_d": bore, "od": bore + 2 * (LIP_R_MM + WALL_T_MM), "exit_d": exit_d,
            "rotor_depth": rotor_depth, "diff_len": diff_len}


def build_duct_ring(prop_d: float, gap: float) -> Tuple[List[Shell], Dict[str, float]]:
    """Duct printed upright: exit plane on the bed (z=0), bellmouth inlet at z=L."""
    d = duct_dims(prop_d, gap)
    rb, re, ro, t, L = d["bore_d"] / 2, d["exit_d"] / 2, d["od"] / 2, WALL_T_MM, DUCT_LEN_MM
    z_rot = L - d["rotor_depth"]                     # rotor plane height above exit (35 mm)
    R = lambda z: re - (re - rb) * z / z_rot         # inner diffuser radius
    zs = ARM_SLOT_H_MM
    z_cb = z_rot - 0.5                               # chamfer base under the lip collar
    z_ct = z_cb + (ro - (R(z_cb) + t))               # 45 deg chamfer -> printable overhang
    arc = [(rb + LIP_R_MM + LIP_R_MM * math.cos(a), L - LIP_R_MM + LIP_R_MM * math.sin(a))
           for a in np.radians(np.linspace(90, 180, 17))]
    upper = [(R(zs), zs), (R(zs) + t, zs), (R(z_cb) + t, z_cb), (ro, z_ct), (ro, L)] + arc + [(rb, z_rot)]
    lower = [(re, 0.0), (re + t, 0.0), (R(zs) + t, zs), (R(zs), zs)]
    shells = [revolve(upper, np.linspace(0, 2 * np.pi, N_SEG, endpoint=False), True, "duct_upper")]
    # arm slot (centred on -X, i.e. facing the airframe centre once installed)
    half = math.asin((ARM_SLOT_W_MM / 2) / re)
    nlow = int(N_SEG * (2 * np.pi - 2 * half) / (2 * np.pi))
    shells.append(revolve(lower, np.linspace(np.pi + half, 3 * np.pi - half, nlow + 1), False, "duct_lower_slotted"))
    # Pappus check volume (exact solid of revolution, slot removed)
    pu, pl = np.array(upper), np.array(lower)
    vol = lambda p: 2 * np.pi * abs(sum((p[i, 0] * p[(i + 1) % len(p), 1] - p[(i + 1) % len(p), 0] * p[i, 1])
                                        * (p[i, 0] + p[(i + 1) % len(p), 0]) for i in range(len(p))) / 6.0)
    d["pappus_volume_mm3"] = vol(pu) + vol(pl) * (2 * np.pi - 2 * half) / (2 * np.pi)
    d["z_collar_top"] = z_ct
    return shells, d


def build_grille() -> List[Shell]:
    w = 2 * duct_dims(prop_d_mm, TIP_GAP_BASELINE_MM)["od"] / 2   # 146.2 mm square
    h = w / 2
    rim_in = h - GRILLE_RIM_MM
    nb = int(rim_in // GRILLE_PITCH_MM)
    bars = [k * GRILLE_PITCH_MM for k in range(-nb, nb + 1) if abs(k * GRILLE_PITCH_MM) + GRILLE_BAR_MM / 2 < rim_in]
    lines = {-h, h, -rim_in, rim_in}
    for b in bars:
        lines |= {b - GRILLE_BAR_MM / 2, b + GRILLE_BAR_MM / 2}
    g = np.array(sorted(lines))
    cen = 0.5 * (g[:-1] + g[1:])
    in_bar = np.array([any(abs(cc - b) < GRILLE_BAR_MM / 2 for b in bars) for cc in cen])
    in_rim = np.abs(cen) > rim_in
    solid = in_rim[:, None] | in_rim[None, :] | in_bar[:, None] | in_bar[None, :]
    return [grid_extrude(g, g, solid, 0.0, GRILLE_DEPTH_MM, "grille")]


def build_enclosure_quarter(prop_d: float = prop_d_mm) -> List[Shell]:
    """Quarter (+X,+Y) in its own frame: spans [0,qw]^2 x [0,box_h]; (0,0) is the box centre line."""
    od = duct_dims(prop_d, TIP_GAP_BASELINE_MM)["od"]
    qw = (2 * MOTOR_OFFSET_A_MM + duct_dims(127.0, 1.5)["od"] + 2 * OUTER_MARGIN_MM) / 2  # locked box
    a, s2 = MOTOR_OFFSET_A_MM, SPINE_W_MM / 2
    r_pocket = od / 2 + POCKET_CLEAR_MM
    zt, zb = BOX_H_MM - DECK_T_MM, DECK_T_MM
    sq = [(0, 0), (qw, 0), (qw, qw), (0, qw)]
    bx, by = PAYLOAD_L_MM / 2, PAYLOAD_W_MM / 2          # payload door cut-out corner
    notched = [(bx, 0), (qw, 0), (qw, qw), (0, qw), (0, by), (bx, by)]
    sh = [plate_with_round_hole(notched, (a, a), r_pocket, 0.0, zb, name="bottom_deck"),
          plate_with_round_hole(sq, (a, a), r_pocket, zt, BOX_H_MM, name="top_deck"),
          box(qw - PANEL_T_MM, qw, 0, qw, zb, zt, "side_panel_x"),
          box(0, qw - PANEL_T_MM, qw - PANEL_T_MM, qw, zb, zt, "side_panel_y")]
    xe = qw - PANEL_T_MM
    for z0 in (zb, zt - FLANGE_H_MM):
        sh.append(box(s2, xe, s2, s2 + FLANGE_T_MM, z0, z0 + FLANGE_H_MM, "flange_rail_y"))
        sh.append(box(s2, s2 + FLANGE_T_MM, s2 + FLANGE_T_MM, xe, z0, z0 + FLANGE_H_MM, "flange_rail_x"))
    t0, t1 = 30.0, SPINE_LEN_MM / 2 - 5.0
    tz0 = zb + TONGUE_Z0_MM
    sh.append(box(t0, t1, s2 - TONGUE_D_MM, s2, tz0, tz0 + TONGUE_H_MM, "tongue_y"))
    sh.append(box(s2 - TONGUE_D_MM, s2, t0, t1, tz0, tz0 + TONGUE_H_MM, "tongue_x"))
    ring = [(BOSS_ID_MM / 2, zb), (BOSS_OD_MM / 2, zb), (BOSS_OD_MM / 2, zt), (BOSS_ID_MM / 2, zt)]
    for (px, py) in [(40.0, 40.0), (40.0, 170.0), (170.0, 40.0), (170.0, 170.0)]:
        assert math.hypot(px - a, py - a) > r_pocket + BOSS_OD_MM / 2
        sh.append(revolve(ring, np.linspace(0, 2 * np.pi, 48, endpoint=False), True, "insert_boss").moved(px, py))
    return sh


def _groove_wall_profile(y_in: float, y_out: float, h: float) -> List[Tuple[float, float]]:
    """(u = wall-normal coordinate, z) profile of a spine wall with a groove on the outer face."""
    g0, g1, gd = GROOVE_Z0_MM, GROOVE_Z0_MM + GROOVE_H_MM, GROOVE_D_MM
    return [(y_in, 0), (y_out, 0), (y_out, g0), (y_out - gd, g0), (y_out - gd, g1), (y_out, g1), (y_out, h), (y_in, h)]


def build_cross_spine() -> List[Shell]:
    """Central cross spine (|x|,|y| <= 125) with 60x40x30 payload bay; z=0 at bay floor/door plane."""
    s2, L, t = SPINE_W_MM / 2, SPINE_LEN_MM / 2, SPINE_WALL_MM
    sh = []
    prof = _groove_wall_profile(s2 - t, s2, SPINE_H_MM)
    # X-arm side walls: profile in (y,z) extruded along x  -> local (u,z,x) -> permute to (y,z,x)
    for sign in (+1, -1):
        p = [(sign * u, z) for (u, z) in prof]
        sh.append(extrude_polygon(p, -L, L).permuted((1, 2, 0)))
    # Y-arm walls (lower rails): profile in (x,z) extruded along y
    prof_y = _groove_wall_profile(s2 - t, s2, SPINE_Y_H_MM)
    for sx in (+1, -1):
        p = [(sx * u, z) for (u, z) in prof_y]
        for (y0, y1) in ((s2, L), (-L, -s2)):
            sh.append(extrude_polygon(p, y0, y1).permuted((0, 2, 1)))
    yi = s2 - t
    # servo bulkheads at X-arm ends
    sh.append(box(L - t, L, -yi, yi, 0, SPINE_H_MM, "bulkhead_fwd"))
    sh.append(box(-L, -L + t, -yi, yi, 0, SPINE_H_MM, "bulkhead_aft"))
    # payload bay end walls; front wall carries the RunCam Thumb window
    bx = PAYLOAD_L_MM / 2
    sh.append(box(-bx - t, -bx, -yi, yi, 0, SPINE_H_MM, "bay_aft_wall"))
    w2, z0w, z1w = CAM_WIN_W_MM / 2, (SPINE_H_MM - CAM_WIN_H_MM) / 2, (SPINE_H_MM + CAM_WIN_H_MM) / 2
    u_shape = [(-yi, 0), (yi, 0), (yi, SPINE_H_MM), (w2, SPINE_H_MM), (w2, z0w), (-w2, z0w), (-w2, SPINE_H_MM), (-yi, SPINE_H_MM)]
    # bay front wall: U-shape below/beside the window + lintel above it
    sh.append(extrude_polygon(u_shape, bx, bx + t).permuted((1, 2, 0)))
    sh.append(box(bx, bx + t, -w2, w2, z1w, SPINE_H_MM, "bay_front_lintel"))
    return sh


def leg_outline() -> np.ndarray:
    r = LEG_W_ROOT_MM / 2
    xf = LEG_LEN_MM - FOOT_W_MM
    pts = [(0.0, -r), (xf, -LEG_W_TIP_MM / 2), (xf, -FOOT_LEN_MM / 2), (LEG_LEN_MM, -FOOT_LEN_MM / 2),
           (LEG_LEN_MM, FOOT_LEN_MM / 2), (xf, FOOT_LEN_MM / 2), (xf, LEG_W_TIP_MM / 2), (0.0, r)]
    pts += [(r * math.cos(a), r * math.sin(a)) for a in np.radians(np.linspace(90, 270, 25)[1:-1])]
    return np.array(pts)


def build_retractable_leg() -> List[Shell]:
    """Leg printed flat: pivot axis at (0,0) along z, leg along +x, lateral foot along y."""
    ring = [(HORN_BOSS_ID_MM / 2, LEG_T_MM), (HORN_BOSS_OD_MM / 2, LEG_T_MM),
            (HORN_BOSS_OD_MM / 2, LEG_T_MM + HORN_BOSS_H_MM), (HORN_BOSS_ID_MM / 2, LEG_T_MM + HORN_BOSS_H_MM)]
    return [extrude_polygon(leg_outline(), 0.0, LEG_T_MM, "leg_beam_foot"),
            revolve(ring, np.linspace(0, 2 * np.pi, 64, endpoint=False), True, "servo_horn_boss")]


# ============================================================================
# 4. STL I/O + verification
# ============================================================================
STL_DTYPE = np.dtype([("n", "<f4", (3,)), ("v", "<f4", (3, 3)), ("a", "<u2")])


def write_binary_stl(path: Path, shells: List[Shell], title: str) -> None:
    tris = np.concatenate([s.triangles() for s in shells]).astype(np.float32)
    nrm = np.cross(tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 0]).astype(np.float64)
    nrm /= np.maximum(np.linalg.norm(nrm, axis=1, keepdims=True), 1e-30)
    data = np.zeros(len(tris), dtype=STL_DTYPE)
    data["n"], data["v"] = nrm.astype(np.float32), tris
    header = f"Project8 5in binary STL | {title}".encode("ascii")[:80].ljust(80, b" ")
    with open(path, "wb") as fh:
        fh.write(header)
        fh.write(struct.pack("<I", len(tris)))
        fh.write(data.tobytes())


def read_binary_stl(path: Path) -> np.ndarray:
    raw = path.read_bytes()
    n = struct.unpack("<I", raw[80:84])[0]
    if len(raw) != 84 + 50 * n:
        raise ValueError(f"{path.name}: size {len(raw)} != 84 + 50*{n}")
    return np.frombuffer(raw, dtype=STL_DTYPE, count=n, offset=84)["v"].astype(np.float64)


def check_tris(tris: np.ndarray) -> Dict[str, float]:
    """Weld exact float32 vertices, then edge-manifold + orientation + volume checks."""
    pts = tris.reshape(-1, 3)
    _, inv = np.unique(pts, axis=0, return_inverse=True)
    f = np.asarray(inv).reshape(-1, 3)
    e = np.concatenate([f[:, [0, 1]], f[:, [1, 2]], f[:, [2, 0]]])
    _, cnt = np.unique(np.sort(e, axis=1), axis=0, return_counts=True)
    _, dcnt = np.unique(e, axis=0, return_counts=True)
    a, b, c = tris[:, 0], tris[:, 1], tris[:, 2]
    area = 0.5 * np.linalg.norm(np.cross(b - a, c - a), axis=1)
    vol = float(np.einsum("ij,ij->i", a, np.cross(b, c)).sum() / 6.0)
    return {"closed_2manifold": bool(np.all(cnt == 2)), "consistent_orientation": bool(np.all(dcnt == 1)),
            "bad_edges": int(np.sum(cnt != 2)), "degenerate_tris": int(np.sum(area < 1e-9)),
            "volume_mm3": vol, "area_mm2": float(area.sum())}


def verify_stl(path: Path, shell_tri_counts: List[int]) -> Dict:
    tris = read_binary_stl(path)
    assert len(tris) == sum(shell_tri_counts), "triangle count != header/shell sum"
    per, k = [], 0
    for n in shell_tri_counts:
        per.append(check_tris(tris[k:k + n]))
        k += n
    lo, hi = tris.reshape(-1, 3).min(0), tris.reshape(-1, 3).max(0)
    dims = hi - lo
    return {
        "triangles": int(len(tris)), "shells": len(per),
        "all_shells_closed": all(p["closed_2manifold"] and p["consistent_orientation"] for p in per),
        "all_shell_volumes_positive": all(p["volume_mm3"] > 0 for p in per),
        "degenerate_tris": int(sum(p["degenerate_tris"] for p in per)),
        "volume_cm3": sum(p["volume_mm3"] for p in per) / 1000.0,
        "surface_area_cm2": sum(p["area_mm2"] for p in per) / 100.0,
        "bbox_min_mm": [round(float(x), 3) for x in lo], "bbox_max_mm": [round(float(x), 3) for x in hi],
        "bbox_dims_mm": [round(float(x), 3) for x in dims],
        "fits_bambu_p1s": bool(np.all(dims <= np.array(BED_MM))),
    }


def centroid_of(shells: List[Shell]) -> Tuple[float, np.ndarray]:
    V, M = 0.0, np.zeros(3)
    for s in shells:
        a, b, c = (s.v[s.f[:, k]] for k in range(3))
        v6 = np.einsum("ij,ij->i", a, np.cross(b, c))
        V += v6.sum() / 6.0
        M += ((a + b + c) * v6[:, None]).sum(0) / 24.0
    return V, M / V


def sliced_mass_g(volume_cm3: float, area_cm2: float, rho: float) -> Tuple[float, float]:
    """Skin (A * 0.84 mm, capped at V) printed solid, remainder at INFILL_FRAC."""
    v_skin = min(volume_cm3, area_cm2 * SKIN_T_MM / 10.0)
    return rho * (v_skin + INFILL_FRAC * (volume_cm3 - v_skin)), v_skin


# ============================================================================
# 5. Main
# ============================================================================
def main(argv=None) -> Dict:
    global prop_d_mm
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[1])
    ap.add_argument("--prop-d", type=float, default=None, help="measured prop diameter mm (Sync Gate A)")
    ap.add_argument("--tip-gap", type=float, default=None, help="extra flight-ring tip gap mm (adds a ring)")
    args = ap.parse_args(argv)
    if args.prop_d is not None:
        prop_d_mm = args.prop_d
    STL_DIR.mkdir(parents=True, exist_ok=True)
    SIZING_DIR.mkdir(parents=True, exist_ok=True)

    gaps = list(STAGE0_GAPS_MM)
    if args.tip_gap is not None and all(abs(args.tip_gap - g) > 1e-6 for g in gaps):
        gaps.append(args.tip_gap)

    if 2 * MOTOR_OFFSET_A_MM - duct_dims(prop_d_mm, max(gaps))["od"] < SPINE_W_MM:
        print(f"WARNING: prop_d={prop_d_mm} mm -> duct OD leaves < {SPINE_W_MM} mm for the spine!")

    bom_raw = {"duct": 50.0, "grille": 17.0, "quarter": (135.0 + 142.0) / 4, "spine": 30.0, "leg": 25.0}
    parts = []  # (key, filename, shells, material, bom_key, expected_dims)
    for g in gaps:
        sh, d = build_duct_ring(prop_d_mm, g)
        gs = f"{g:.1f}" if abs(g * 10 - round(g * 10)) < 1e-9 else f"{g:.2f}"
        parts.append((f"duct_ring_gap{gs}", f"duct_ring_5in_gap{gs}mm.stl", sh, "PETG", "duct",
                      (d["od"], d["od"], DUCT_LEN_MM), d))
    od_b = duct_dims(prop_d_mm, 1.5)["od"]
    parts.append(("grille", "grille_5in_146mm_4mm.stl", build_grille(), "PLA", "grille",
                  (od_b, od_b, GRILLE_DEPTH_MM), {}))
    qw = (2 * MOTOR_OFFSET_A_MM + duct_dims(127.0, 1.5)["od"] + 2 * OUTER_MARGIN_MM) / 2
    parts.append(("enclosure_quarter", "enclosure_quarter_5in_177mm.stl", build_enclosure_quarter(prop_d_mm),
                  "PLA", "quarter", (qw, qw, BOX_H_MM), {}))
    parts.append(("cross_spine_payload_bay", "cross_spine_45mm_payload_bay.stl", build_cross_spine(), "PLA", "spine",
                  (SPINE_LEN_MM, SPINE_LEN_MM, SPINE_H_MM), {}))
    leg = build_retractable_leg()
    parts.append(("retractable_leg", "retractable_leg_240mm.stl", leg, "PLA", "leg",
                  (LEG_LEN_MM + LEG_W_ROOT_MM / 2, FOOT_LEN_MM, LEG_T_MM + HORN_BOSS_H_MM), {}))

    results, ok_all = [], True
    print(f"\nProject 8 STL generator  (prop_d_mm = {prop_d_mm:.2f}, box = {2*qw:.1f} mm, quarter = {qw:.2f} mm)")
    for key, fname, shells, mat, bkey, exp_dims, extra in parts:
        path = STL_DIR / fname
        write_binary_stl(path, shells, key)
        chk = verify_stl(path, [len(s.f) for s in shells])
        dims_ok = bool(np.allclose(sorted(chk["bbox_dims_mm"]), sorted(exp_dims), atol=0.05))
        rho = MATERIALS[mat]
        m_sliced, v_skin = sliced_mass_g(chk["volume_cm3"], chk["surface_area_cm2"], rho)
        m_solid = rho * chk["volume_cm3"]
        raw = bom_raw[bkey]
        row = {"part": key, "file": f"outputs/stl/{fname}", "material": mat, "density_g_cm3": rho, **chk,
               "expected_dims_mm": [round(x, 3) for x in exp_dims], "bbox_matches_design": dims_ok,
               "skin_volume_cm3": round(v_skin, 2),
               "mass_solid_100pct_g": round(m_solid, 1), "mass_sliced_est_g": round(m_sliced, 1),
               "bom_est_raw_g": round(raw, 2), "bom_counted_x1.15_g": round(raw * 1.15, 2),
               "delta_vs_bom_raw_pct": round(100 * (m_sliced / raw - 1), 1),
               "delta_vs_bom_counted_pct": round(100 * (m_sliced / (raw * 1.15) - 1), 1)}
        if "pappus_volume_mm3" in extra:
            row["pappus_exact_volume_cm3"] = round(extra["pappus_volume_mm3"] / 1000, 3)
            row["mesh_vs_pappus_pct"] = round(100 * (chk["volume_cm3"] * 1000 / extra["pappus_volume_mm3"] - 1), 3)
            row["bore_d_mm"], row["exit_d_mm"] = round(extra["bore_d"], 2), round(extra["exit_d"], 2)
        if key == "retractable_leg":
            V, cen = centroid_of(shells)
            row["centroid_from_pivot_mm"] = [round(float(x), 2) for x in cen]
        ok = chk["all_shells_closed"] and chk["all_shell_volumes_positive"] and dims_ok and chk["fits_bambu_p1s"] \
            and chk["degenerate_tris"] == 0
        row["verified"] = bool(ok)
        ok_all &= ok
        results.append(row)

    hdr = f"{'Part':<26}{'Tris':>7}{'Shl':>4}{'Closed':>7}{'Vol cm3':>9}{'Solid g':>9}{'Slice g':>9}{'BOM raw':>9}{'BOMx1.15':>9}{'d raw%':>8}  bbox mm"
    print(hdr)
    print("-" * len(hdr))
    for r in results:
        print(f"{r['part']:<26}{r['triangles']:>7}{r['shells']:>4}{str(r['all_shells_closed']):>7}{r['volume_cm3']:>9.2f}"
              f"{r['mass_solid_100pct_g']:>9.1f}{r['mass_sliced_est_g']:>9.1f}{r['bom_est_raw_g']:>9.2f}"
              f"{r['bom_counted_x1.15_g']:>9.2f}{r['delta_vs_bom_raw_pct']:>8.1f}  "
              f"{' x '.join(f'{d:.1f}' for d in r['bbox_dims_mm'])} {'OK' if r['verified'] else 'FAIL'}")

    # Aircraft-level printed-structure roll-up (flight ring = gap 1.5)
    by = {r["part"]: r for r in results}
    n_of = {"duct_ring_gap1.5": 4, "grille": 8, "enclosure_quarter": 4, "cross_spine_payload_bay": 1, "retractable_leg": 2}
    cad_total = sum(by[k]["mass_sliced_est_g"] * n for k, n in n_of.items())
    bom_total_raw = 4 * 50 + 8 * 17 + 135 + 142 + 30 + 2 * 25
    print(f"\nPrinted-structure roll-up (4 ducts, 8 grilles, 4 quarters, spine, 2 legs): CAD sliced "
          f"{cad_total:.1f} g vs BOM raw {bom_total_raw:.1f} g / counted {bom_total_raw*1.15:.1f} g "
          f"({100*(cad_total/bom_total_raw-1):+.1f} % vs raw, {100*(cad_total/(1.15*bom_total_raw)-1):+.1f} % vs counted)")

    out = {
        "generator": "src/cad/generate_cad_stl.py",
        "prop_d_mm": prop_d_mm, "tip_gaps_mm": gaps,
        "assumptions": {
            "slicer_model": f"skin {SKIN_T_MM} mm over full surface printed solid (2 x 0.42 mm walls, ~0.8-1.0 mm top/bottom), "
                            f"remaining volume at {int(INFILL_FRAC*100)} % infill; thin walls (<=1.7 mm) therefore print solid",
            "densities_g_cm3": MATERIALS,
            "materials": "ducts PETG; grille, quarter, spine, legs PLA (leg hinge pins/servo mounts are separate PA-CF parts, not meshed)",
            "not_meshed_in_stl": "duct arm-clamp bolt bosses, deck clearance holes over insert bosses, door hinge knuckles, "
                                 "side-panel camera aperture (all present in cad/project8_5in_parametric.scad)",
            "quarter_print": "STL is the +X/+Y quarter; print 2 as-is and 2 mirrored in the slicer (payload-door notch is 30 x 20)",
            "shells": "each part is a union of closed shells that touch but never overlap; volume = exact sum",
        },
        "parts": results,
        "aircraft_rollup": {"cad_sliced_total_g": round(cad_total, 1), "bom_raw_total_g": bom_total_raw,
                            "bom_counted_total_g": round(1.15 * bom_total_raw, 1),
                            "delta_vs_raw_pct": round(100 * (cad_total / bom_total_raw - 1), 1),
                            "delta_vs_counted_pct": round(100 * (cad_total / (1.15 * bom_total_raw) - 1), 1)},
        "all_verified": bool(ok_all),
    }
    (SIZING_DIR / "cad_mass_comparison.json").write_text(json.dumps(out, indent=2))
    print(f"\nAll STL checks passed: {ok_all}  -> {STL_DIR}")
    print(f"Saved {SIZING_DIR / 'cad_mass_comparison.json'}")
    if not ok_all:
        sys.exit(1)
    return out


if __name__ == "__main__":
    main()
