# Project 8 — Enclosed Ducted-Fan Quadrotor: Technical Analysis and Validation

**EC4.402 Introduction to UAV Design · IIIT Hyderabad · Monsoon 2026 · Team 4**
Shriya Kansal · Kavya Veer · Nishika Panjwani · Parth Sharma · Monishram Selvaraj · Udhav Malpani

Locked configuration (Design Review, 27 Oct 2026): **5-inch, 6S, 4 PETG bellmouth ducts in a 353 × 353 × 100 mm PLA box**, 180 g payload, flush skids + cradle baseline with retractable legs as the gated upgrade.

> **Status of the evidence in this report (30 Oct 2026).** No hardware has flown yet. Every figure tagged *predicted* comes from the sizing model, the CAD meshes, or the 6-DOF closed-loop simulator in this repository. The Stage 0 thrust-stand numbers are model-generated stand-ins (clearly marked SYNTHETIC on Fig. 7). The *Measured* column of the §11 master table is deliberately empty until Hardware Sync Gates A–D are run; every script is already wired to take the real data (Appendix B).

---

## 1. Summary

| Question | Answer (predicted) | Where |
|---|---|---|
| All-up mass | 1,825.2 g (skids) · 1,904.9 g (legs) · Stage 1 bare 1,131 g | §3, §7 |
| Thrust-to-weight | 3.10 (skids) · 2.97 (legs), target ≥ 2.5 | §3 |
| Hover endurance | 5.7 / 4.8 min (skids), 5.4 / 4.5 min (legs) at 0 / +20 % enclosure loss | §3 |
| Endurance confirmed in simulation | still-air hover to 80 % used: **4.52 min** at 27.7 A (model: 4.49 min, 27.8 A) | §9.4 |
| Converged? | Yes: Round 2 → 3 Δm = 0 %. Round 4 (CAD-sliced printed masses) adds +54 g → 1,959 g, T/W 2.89, 5.2 / 4.3 min | §3.2 |
| CG within 5 mm in every state? | Yes: worst 1.24 mm (payload out), 9 states checked | §7 |
| Autonomous mission in 15 km/h wind | 6 waypoints, all passed within 0.2 m; cross-track p95 1.45 m; lands with **58.4 %** (gate ≥ 25 %) | §9 |
| Failsafes | RC loss → RTL, low battery → RTL → LAND, geofence → RTL: all respond in < 1 s and land < 1 m from home | §9.3 |
| Main risk found | **Wind margin.** At 15 km/h the duct ram drag already needs ~19° of lean to hold position; gusts drive lean to 28–31°, against ANGLE_MAX 30° | §9.2 |
| Second risk found | Printed structure is ~23 % heavier than the raw BOM estimate (mostly the spine and grilles); still within +15 % contingency overall to +54 g | §6.3 |

---

## 2. Requirements and design baseline

**Course brief:** four ducted fans inside a complete enclosure, no rotating part exposed, designed intake/exhaust, stable enclosed flight, autonomous position waypoints with the enclosure fitted. **Professor's constraints:** cuboid box, internal payload, nothing hanging below in flight. **Committed deliverables (slide 12):** ten items, including take-off, ≥ 4 waypoints, landing with ≥ 25 % battery, 180 g payload with camera, failsafes, flight logs and this report.

**Environment.** IIIT Hyderabad, 540 m ASL, 30 °C → ρ = 1.09 kg/m³, a 0.89 thrust derating against the sea-level datasheet.

**Locked geometry (slides 3–4)**

| Parameter | Value | Rule |
|---|---|---|
| Prop D | 127.0 mm, 5×4.3 tri-blade | A = πD²/4 = 0.01267 m² |
| Tip gap δ | 1.5 mm (stand sweep 1.0 / 1.5 / 2.0) | arms flex ~0.5 mm at full thrust |
| Bore d_b | 130.0 mm | D + 2δ |
| Lip radius | 6.5 mm | 0.05 d_b |
| Wall | 1.6 mm PETG | |
| Duct OD d_o | 146.2 mm | d_b + 2(r_lip + t) |
| Length L | 50 mm (L/D 0.39), rotor 15 mm below inlet | |
| Diffuser | 35 mm, 4.1° half-angle, exit 135.0 mm, AR 1.079 | θ ≤ 5° |
| Motor offset a | 97.5 mm; diagonal 275.8 mm | true-X |
| Duct gap | 48.8 mm vs 45 mm spine → 3.8 mm clearance | 2a − d_o |
| Box | 353 × 353 × 100 mm, 4 quarters of 176.5 mm | fits 256 mm bed |
| Grilles | 1 mm bars, 12 mm pitch, 4 mm deep, β = 0.16 | intake 1.42 × disc |
| Exhaust clearance | ≥ 219.3 mm | 1.5 d_o |

---

## 3. Propulsion sizing and the iteration loop

### 3.1 Method (TA's order: mass → props → motors → ESC → battery)

Momentum theory (Leishman 2006), applied per motor:

T_h = mg/4 → P_i = T_h^{3/2}/√(2ρA) → P_e = P_i/(FM·η) → I_h = (4P_e + P_av)/V_nom → t_h = 0.8·C/I_h

with FM 0.60, η 0.80, P_av 10 W and V_nom 22.2 V. Usable thrust is T_max = 1.98 kgf × (1.09/1.225) × (21.5/24)² = **1.41 kgf**. Derivations are in Appendix A.

### 3.2 Convergence (retractable-legs configuration, Slide 8)

| Round | Mass basis | m (g) | Δm | T_h (N) | P_i (W) | P_e (W) | I_h 0 / 20 % (A) | T/W | Hover 0 / 20 % (min) |
|---|---|---|---|---|---|---|---|---|---|
| 1 | worst-case guess | 2,000.0 | — | 4.91 | 65.4 | 136.2 | 25.0 / 29.9 | 2.83 | 4.99 / 4.17 |
| 2 | BOM (spec + 1.15 est + allow) | 1,904.9 | −4.75 % | 4.67 | 60.8 | 126.6 | 23.3 / 27.8 | 2.97 | 5.37 / 4.49 |
| 3 | unchanged BOM | 1,904.9 | 0.00 % ✓ | 4.67 | 60.8 | 126.6 | 23.3 / 27.8 | 2.97 | 5.37 / 4.49 |
| **4 (new)** | **Round 3 + CAD-sliced printed parts (+54.1 g, §6.3)** | **1,959.0** | **+2.8 %** | 4.80 | — | — | 24.2 / 29.0 | 2.89 | 5.15 / 4.30 |

The skids loop converges to 1,825.2 g: T/W 3.10, 21.8 / 26.1 A, 5.71 / 4.78 min.

**Reading Round 4.** CAD mass-properties exist, so the 4th round the TA asked for can be run before anything is printed. It moves Δm by 2.8 %, just over the 2 % stop criterion. With legs, the aircraft still meets T/W ≥ 2.5, and the +20 % endurance (4.30 min) sits just under the 4.5 min target. Round 5 uses weighed parts (Gate C). The spine (§6.3) is the part to put on a diet first.

Peak current: 4 × 40 A = 160 A ≤ 2.6 Ah × 65 C = 169 A ✓. ESC: 60 A per channel ≥ 1.3 × 40 A ✓.

### 3.3 Trade studies (all at 180 g payload)

**Prop size** (legs; slide 6, reproduced by `sizing_model.py`)

| | 5-inch | 6-inch (interp.) | 7-inch fallback |
|---|---|---|---|
| All-up mass | 1,905 g | 2,320 g | 2,735 g |
| Battery | 6S 2600 | 6S 2950 | 6S 3300 |
| Usable thrust / motor | 1.41 kgf | 1.59 kgf | 1.77 kgf |
| T/W | **2.97** | 2.75 | 2.59 |
| Hover 0 / 20 % | 5.37 / 4.49 min | 5.45 / 4.55 | 5.56 / 4.65 |
| Hover / max RPM | 21.8k / 33.6k | 16.7k / 27.7k | 12.6k / 20.3k |
| Tip speed at max | 224 m/s | 221 m/s | 189 m/s |
| Box | 353 mm | 414 mm | 475 mm |

Hover time is almost flat in prop size: a bigger rotor's lower disc loading is spent carrying a bigger box and pack. 5-inch wins on T/W margin, mass and box size. 7-inch remains the fallback, sized and gated (§8).

**Battery** (skids): 2200 → 1,765 g, 5.08 / 4.25 min · **2600 → 1,825 g, 5.71 / 4.78 min** · 3300 → 1,945 g, 6.60 / 5.52 min (and needs a wider spine). The 2600 pack is the lightest that clears 4.5 min at +20 % with the payload.

**Payload sensitivity** (skids, +20 % loss): 0 g 5.57 min · 100 g 5.10 · **180 g 4.78** · 300 g 4.35. Each 100 g of payload costs ~0.24 min.

**Blade count:** the 2-blade 5×4.5 gives almost the same hover time (5.72 vs 5.71 min) at +3.1k RPM and lower static thrust. The model-based stand sweep (§8) predicts only 1.14 kgf usable, under the 1.19 kgf legs gate, so the tri-blade stays.

**4S contingency** (2400 KV, 4S 4000 mAh): 1,855 g, 1.17 kgf, T/W 2.53, 5.72 / 4.78 min, 180 A peak. Its parameter file is `config/ardupilot/stage2_4s_contingency.param`.

---

## 4. Duct, grille and crosswind aerodynamics

From `duct_aero.py`, which applies the Pereira (2008) tip-gap trend and an Idelchik screen-loss model to the locked geometry:

| Tip gap | δ/R | Clean-duct gain | Grilles (both) | Single duct + grilles (stand) | + box (flight) | Hover-power penalty |
|---|---|---|---|---|---|---|
| 1.0 mm | 1.57 % | +13.6 % | −8.1 % | ×1.055 | ×0.990 | +18.3 % |
| **1.5 mm** | 2.36 % | +10.9 % | −8.1 % | **×1.028** | **×0.963** | **+20.0 %** |
| 2.0 mm | 3.15 % | +8.4 % | −8.1 % | ×1.003 | ×0.938 | +21.8 % |

At hover the slipstream is 12.7 m/s induced (25.5 m/s jet), about 0.32 m³/s per fan. The intake grille is open over 1.42× the disc area, against the ≥ 1.2× rule. The exhaust is open over 1.35×, including the 5 % arm blockage.

**Crosswind (15 km/h, 1,905 g).** Inlet momentum (ram) drag ṁV is 5.99 N, against 0.38 N of form drag on the box side: **94 % of the side force comes from the ducts swallowing sideways air.** Holding position needs a **18.8° lean** and 5.7 % more thrust. This one number drives the wind findings in §9.2.

---

## 5. System architecture and autopilot configuration

**Avionics.** SpeedyBee F405 V4 running ArduCopter (target `speedybeef4v4`) with a BLS 60 A 4-in-1 ESC, flashed to Bluejay for bidirectional-DShot RPM. GPS/compass is a Matek M10Q-5883 with SBAS. RC and telemetry use the lab radios at 433 / 865 MHz. A separate 5 V 3 A UBEC powers the two MG90S gear servos, so a stalled servo cannot brown out the FC. The RunCam Thumb records only.

The pad-by-pad wiring, power budget and 11-gate checklist are in [docs/wiring_and_bringup_guide.md](../docs/wiring_and_bringup_guide.md).

**Parameter files** in [config/ardupilot/](../config/ardupilot/), loadable in Mission Planner:

| File | Purpose | Key values |
|---|---|---|
| `stage1_bare_5in.param` | bare quad, open props | MOT_THST_HOVER 0.20, ATC_RAT_RLL/PIT_P 0.085, INS_HNTCH_ENABLE 1 (ESC-RPM mode), MOT_BAT_CURR_MAX 160, BATT_LOW/CRT_VOLT 21.0 / 19.8 |
| `stage2_enclosed_5in_skids.param` | enclosed, skids | MOT_THST_HOVER 0.34, WPNAV_SPEED 350, WPNAV_ACCEL 120, LAND_SPEED 40, FS_THR_ENABLE 1, FENCE 120 m / 40 m, GPS_HDOP_GOOD 140, ANGLE_MAX 30° |
| `stage2_enclosed_5in_legs.param` | adds legs | SERVO5/6_FUNCTION 29, LGR_DEPLOY_ALT 3, LGR_RETRACT_ALT 5, SERVO5 1100–1900, MOT_THST_HOVER 0.35 |
| `stage2_4s_contingency.param` | 4S fallback | BATT_LOW/CRT 14.0 / 13.2, MOT_BAT_VOLT 16.8 / 13.2, MOT_BAT_CURR_MAX 180 |

Shared Stage 2 choices:
- **Battery:** BATT_LOW_MAH 780 (30 %) triggers RTL; BATT_CRT_MAH 450 triggers LAND.
- **Navigation:** WPNAV_RADIUS 1 m.
- **Yaw:** **WP_YAW_BEHAVIOR 0** (added during this work). The vehicle holds its heading because yaw is the weakest axis in a ducted box. Yaw comes only from reaction torque, and spending it turning to face each waypoint steals roll/pitch headroom. ATC_ACCEL_Y_MAX and ATC_RATE_Y_MAX are reduced for the same reason.

> **Caution: hover throttle.** `sizing_model.py` reports a `hover_throttle_est` of ~0.45. That is a heuristic PWM-like figure. ArduPilot's MOT_THST_HOVER is in *linearised thrust* units, roughly T_hover / T_max = 0.34–0.35, which the 6-DOF simulation also learns (0.32–0.35). The param files use the latter. Do not copy 0.45 into MOT_THST_HOVER.

**SITL.** `config/sitl/quad_stage1_bare.parm` and `quad_stage2_enclosed_5in.parm`, plus frame JSONs, give ready `sim_vehicle.py` command lines with home at 17.4455, 78.3489, 540 m.

**Mission.** [missions/iiith_5in_enclosed_mission.waypoints](../missions/iiith_5in_enclosed_mission.waypoints) is in QGC WPL 110 format:
- take off to 12 m;
- `MAV_CMD_AIRFRAME_CONFIGURATION` gear up;
- 6 waypoints on a 35 × 25 m box, with 5 s holds;
- `DO_LAND_START` marker;
- 6 m approach over home;
- gear down;
- `NAV_LAND` at home.

This exceeds the committed 4 waypoints. The landing gear also auto-deploys below 3 m on any RTL or LAND.

---

## 6. CAD and printed parts

### 6.1 Sources

- **Fusion 360:** `cad/fusion360_parameters_5in.csv` holds 41 user parameters. Derived ones are expressions (e.g. `bore_d = prop_d + 2*tip_clear`, `duct_od = bore_d + 2*(lip_r + wall_t)`), so changing `tip_clear` re-derives the whole box.
- **OpenSCAD:** `cad/project8_5in_parametric.scad` has a `part_to_render` selector: duct_ring, grille, enclosure_quarter, cross_spine_payload, retractable_leg, full_assembly.
- **STL generator:** `src/cad/generate_cad_stl.py` is a pure Python/NumPy binary-STL generator, with no CAD install needed. `--prop-d` and `--tip-gap` implement Hardware Sync Gate A.

### 6.2 Print-ready STLs (`outputs/stl/`)

All 7 files were parsed back after writing. For every one:
- the triangle count matches the header;
- every shell is closed, with each edge shared by exactly 2 faces;
- the volume is positive;
- the bounding box is within 0.05 mm of design;
- the part fits the 256 mm P1S bed.

The duct ring volume is within 0.018 % of the exact Pappus revolved-solid volume.

| STL | Bounding box (mm) | Material |
|---|---|---|
| `duct_ring_5in_gap1.0mm.stl` | 145.2 × 145.2 × 50 | PETG (stand only) |
| `duct_ring_5in_gap1.5mm.stl` | 146.2 × 146.2 × 50 | PETG, flight |
| `duct_ring_5in_gap2.0mm.stl` | 147.2 × 147.2 × 50 | PETG (stand only) |
| `grille_5in_146mm_4mm.stl` | 146.2 × 146.2 × 4 | PLA ×8 |
| `enclosure_quarter_5in_177mm.stl` | 176.5 × 176.5 × 100 | PLA ×4 (print 2 mirrored: door notch) |
| `cross_spine_45mm_payload_bay.stl` | 250 × 250 × 30 | PLA |
| `retractable_leg_240mm.stl` | 248 × 100 × 12 | PLA (hinges PA-CF, not meshed) |

### 6.3 CAD mass vs BOM estimate

Slicer model: 0.84 mm solid skin (2 × 0.42 mm perimeters) with 15 % infill inside. Densities are PETG 1.27 g/cm³ and PLA 1.24 g/cm³.

| Part | Volume (cm³) | Sliced (g) | BOM raw (g) | BOM × 1.15 (g) | Δ vs raw |
|---|---|---|---|---|---|
| Duct ring 1.5 mm | 64.29 | 55.5 | 50 | 57.5 | +11.0 % (inside contingency) |
| Grille | 18.73 | 23.2 | 17 | 19.6 | +36.6 % (3 mm rim ≈ 6.9 cm³; 2 mm rim → ~20 g) |
| Quarter (panels + decks share) | 71.76 | 89.0 | 69.3 | 79.6 | +28.5 % (flange rails, insert bosses) |
| Spine + bay | 39.02 | 48.4 | 30 | 34.5 | **+61.3 %** (biggest gap) |
| Leg | 34.89 | 19.5 | 25 | 28.8 | −21.9 % |
| **Printed set** (4 ducts, 8 grilles, 4 quarters, spine, 2 legs) | | **851.0** | 693.0 | 796.9 | **+22.8 % raw, +6.8 % counted** |

**Conclusion.** The +15 % contingency nearly absorbs the overrun in total, but not part by part. The 30 g spine allowance was too optimistic for a structural spine that carries the bay and the leg servos. Round 4 (§3.2) books the +54 g.

---

## 7. Mass, CG and inertia audit

`src/cad/mass_cg_audit.py` models 98 bodies in FRD axes with the origin at the box centre. It reproduces the BOM totals exactly: 1,904.90 g (legs) and 1,825.20 g (skids). Large bodies carry self-inertia, and duct properties come from the STL mesh.

| State | Mass (g) | x (mm) | y (mm) | z (mm, +down) | \|r_h\| (mm) | ≤ 5 mm |
|---|---|---|---|---|---|---|
| S2 legs UP, payload IN (reference) | 1,904.9 | 0.00 | 0.00 | +1.06 | 0.00 | ✓ |
| S2 legs DOWN, payload IN | 1,904.9 | 0.00 | 0.00 | +5.16 | 0.00 | ✓ |
| S2 legs UP, payload OUT | 1,724.9 | −1.19 | −0.30 | −2.17 | 1.22 | ✓ |
| S2 legs DOWN, payload OUT | 1,724.9 | −1.19 | −0.30 | +2.36 | 1.22 | ✓ |
| S2 skids, payload IN / OUT | 1,825.2 / 1,645.2 | 0 / −1.24 | 0 | −0.83 / −4.42 | 0 / 1.24 | ✓ |
| S1 bare legs DOWN / UP / skids | 1,131.0 / 1,131.0 / 1,051.3 | 0 | 0 | +7.96 / +1.05 / −2.22 | 0 | ✓ |

**Ballast placement (correction to the plan).** The plan assumed the ballast sits at −1.0 mm x. Countering the camera alone needs −(9.8 × 30)/170.2 = −1.73 mm. Once the asymmetric XT90/capacitor (x −80), buzzer (x −60) and UBEC (y −45) are included, nulling the whole aircraft needs the ballast centroid at **x = +10.3 mm, y = +3.0 mm**.

A 24 × 36 × 25 mm block cannot reach that y inside the 40 mm-wide bay. A **24 × 32 × 28 mm steel block** can; it reaches x up to +11 mm before hitting the camera. Final trim is done by weighing on the built aircraft.

**Inertia about the CG (kg·m²)**

| Case | Ixx | Iyy | Izz | max \|product\| |
|---|---|---|---|---|
| Stage 1 bare (legs down), 1,131 g | 0.00537 | 0.00736 | 0.00652 | 1.4e-4 |
| Stage 2 skids, 1,825 g | 0.01396 | 0.01469 | 0.02513 | 1.1e-4 |
| Stage 2 legs UP, 1,905 g | 0.01346 | 0.01495 | 0.02455 | 1.1e-4 |
| Stage 2 legs DOWN, 1,905 g | 0.01566 | 0.01776 | 0.02517 | 1.4e-4 |

Stage 2 has about 2.5× the roll/pitch inertia and 3.8× the yaw inertia of Stage 1, with the same thrust authority. That is why Stage 2 starts with lower acceleration limits and must be **re-autotuned** (Gate 8). Products of inertia are ≤ 2.7 % of the smallest diagonal term, so treating the inertia as diagonal is justified.

**Stage 1 mass (correction to the plan).** The plan quoted 1.161 kg. The BOM-derived value is **1,130.95 g** with legs and payload, or 1,051 g with skids: 30 g less.

---

## 8. Stage 0 thrust-stand plan

**Hardware.** BOM item 38: 5 kg load cell + HX711, ESC servo PWM, a voltage/current sensor and an optical RPM sensor. The sketch is [firmware/thrust_stand_logger/thrust_stand_logger.ino](../firmware/thrust_stand_logger/thrust_stand_logger.ino):
- automatic ramp from 1000 to 2000 µs in 50 µs (5 %) steps, 3.0 s settle, 20 samples averaged per step;
- hardware abort button and serial `x`;
- cut-offs for over-current > 45 A, thrust > 4.5 kg and pack < 19.8 V;
- CSV `config,tip_gap_mm,pwm_us,throttle_pct,thrust_g,voltage_v,current_a,power_w,rpm`.

**Five configurations:**
1. open 5×4.3 tri-blade;
2. open 5×4.5 two-blade (decision D3);
3. duct + both grilles at 1.0 mm tip gap;
4. duct + both grilles at 1.5 mm (flight baseline);
5. duct + both grilles at 2.0 mm.

**Analysis.** `src/analysis/thrust_stand_analyzer.py --csv <log>` computes:
- C_T, C_P and FM;
- usable thrust, scaled to 21.5 V loaded and ρ 1.09;
- duct gain at equal electrical power;
- the hover point (power, current, throttle, g/W, predicted MOT_THST_HOVER);
- T/W and hover time computed directly from stand data;

and then applies the Slide 6 gate.

**Gate rules:**

| Usable thrust per motor | Decision |
|---|---|
| ≥ 1.19 kgf | 5-inch as designed + legs |
| 1.14–1.19 kgf | skids |
| 1.10–1.14 kgf | trim about 140 g |
| 0.95–1.10 kgf | accept T/W 2.0–2.3 |
| < 0.95 kgf | switch to 7-inch |

**Predicted (synthetic) outcome, Fig. 7:**

| Config | Usable (kgf) | FM max | Thrust gain at equal power vs open 3-blade | P at 476 gf (W) | g/W | T/W (legs) | Hover 0 / +20 % (min) | Gate |
|---|---|---|---|---|---|---|---|---|
| Open 5×4.3 3-blade | 1.417 | 0.63 | — | 134.1 | 3.55 | 2.98 | 5.07 / 4.24 | legs |
| Open 5×4.5 2-blade | 1.137 | 0.66 | +3.2 % | 128.0 | 3.72 | 2.39 | 5.31 / 4.44 | trim |
| Duct + grilles 1.0 mm | 1.478 | 0.68 | +3.7 % | 128.2 | 3.72 | 3.10 | 5.30 / 4.43 | legs |
| **Duct + grilles 1.5 mm** | **1.456** | 0.66 | **+1.5 %** | 131.3 | 3.63 | **3.06** | 5.18 / 4.33 | **legs** |
| Duct + grilles 2.0 mm | 1.433 | 0.64 | +1.1 % | 131.9 | 3.61 | 3.01 | 5.15 / 4.31 | legs |

**What the stand must settle.** The model predicts that the grilles eat most of the clean-duct gain (+10.9 % − 8.1 %). The enclosure then decides whether the aircraft lands at the +0 % or the +20 % endurance figure. That makes the duct + grille hover power at 476 gf the single most valuable measurement in the project.

A real result under 1.19 kgf changes the landing-gear decision, and one under 0.95 kgf triggers the 7-inch fallback. The 1.0 mm gap is stand-only, because the arms flex ~0.5 mm in flight.

---

## 9. Closed-loop flight simulation

### 9.1 Simulator

`src/sitl/mission_simulator.py` is a deterministic 250 Hz 6-DOF model:
- **Rigid body:** quaternion attitude, using the §7 inertia tensors.
- **Motors:** ArduPilot X order and spin directions, first-order lag, yaw from reaction torque only.
- **Thrust and power:** max thrust ∝ (V_loaded/24)², times the duct net multiplier (0.963 enclosed). Power follows momentum theory with κ = 20 % in Stage 2.
- **Battery:** 6S OCV-SoC curve, 18 mΩ pack resistance, coulomb counting.
- **Aerodynamics:** duct ram drag, box form drag, the **duct-lip pitching moment in crossflow**, and a wind shear profile.
- **Wind:** Dryden-like Gauss-Markov turbulence (σ 0.9 m/s).
- **Navigation estimate:** GPS/SBAS drift (σ 0.3 m, τ 25 s) and baro drift. The controllers see only this estimate, as they would see the EKF.

The controller is an ArduCopter-style cascade: position P 1.0 → velocity PID (P 2, I 1, IMAX 8 m/s²) → lean angle, limited to ANGLE_MAX 30° → angle P 4.5 → rate PID. The rate integrator limit is sized in torque, 50 % of authority, like ATC_RAT_*_IMAX. The mixer uses yaw-priority desaturation. Navigation is a carrot-and-leash follower using WPNAV_SPEED/ACCEL, with RTL and LAND state machines, gear logic and all three failsafes.

The constants are copied from the `.param` files: WPNAV 350 / 120 / 150 / 100 cm/s, WPNAV_RADIUS 1 m, RTL_ALT 15 m, LAND_SPEED 40, BATT_LOW/CRT_MAH 780 / 450, FENCE 120 m / 40 m, LGR 3 / 5 m.

Telemetry is written as ArduPilot-like CSVs that the gate verifier reads exactly as it reads a real `.BIN`.

> **Two bugs found and fixed during verification.** Both are worth knowing for the real tune.
> 1. The velocity-loop integrator was capped at 2.4 m/s². Holding position in 15 km/h needs about 3.0 m/s² of steady lean acceleration. ArduPilot's PSC_VELXY_IMAX allows 10 m/s².
> 2. The rate integrator was capped in angular-acceleration units. With the real (smaller) CAD inertia, it could hold only ~0.06 N·m. The duct-lip moment in a 15 km/h crossflow is about **0.3 N·m**, and the aircraft flew with ~5° of attitude lag and up to 9.9 m of cross-track error.
>
> Real-world lesson: **do not reduce ATC_RAT_RLL/PIT_IMAX on the enclosed airframe.** The duct-lip moment is a steady trim load the integrator must carry.

### 9.2 Runs and results

| Run | Scenario | Flight time | Battery left | Hover I (A) | Max lean | Cross-track p95 / max | Landing error |
|---|---|---|---|---|---|---|---|
| A | Stage 1 bare, 2 m/s breeze: ALT_HOLD 30 s, LOITER 60 s | 112 s | 87.6 % | 10.4 | 12.8° | — | 0.67 m |
| B0 | Stage 2, 15 km/h: ALT_HOLD 30 s, LOITER 60 s | 113 s | 65.0 % | 29.3 (26.7 lean-corrected) | **29.1°** | — | 0.44 m |
| **B** | **Stage 2 full AUTO mission, 15 km/h + turbulence** | **129 s** | **58.4 %** | 30.0 | 27.5° | **1.45 / 2.27 m** | 0.45 m |
| B25 | as B at WPNAV_SPEED 2.5 m/s | 131 s | 57.7 % | 29.8 | 27.6° | 1.38 / 2.43 m | 0.52 m |
| B10 | as B in 10 km/h wind | 118 s | 64.0 % | 28.0 | 27.6° | **0.72 / 1.10 m** | 0.34 m |
| D | Stage 2, still air, hover to 80 % used | **270.9 s = 4.52 min** | 20.0 % | **27.7** | 2.0° | — | — |

**Wind is the binding constraint.** Holding station in 15 km/h already takes about 19° of lean; this agrees with the §4 hand calculation of 18.8°. Turbulence then drives transients to 28–31°, at the 30° lean limit. Slowing the mission to 2.5 m/s barely helps (p95 1.45 → 1.38 m), because the drag comes from wind, not ground speed. Dropping the wind to 10 km/h halves the cross-track error (p95 0.72 m).

Recommendations:
- Fly the graded demo in **≤ 10 km/h** wind.
- Keep the 15 km/h limit as an abort criterion only.
- Consider ANGLE_MAX 35° for Stage 2 once AUTOTUNE has run; T/W 2.97 has the authority for it.

### 9.3 Failsafe verification (Run C)

| Run | Injected fault | Response | Reaction | Landed | Battery at disarm |
|---|---|---|---|---|---|
| C1 | RC lost at t = 45 s | RTL: climb to 15 m → home → gear auto-deploys at 3.0 m → land | < 1 s | 0.65 m from home | 70.3 % |
| C2 | pack starts at 42 % | LOW (780 mAh) at 33.5 s → RTL; CRITICAL (450 mAh) → LAND | < 1 s | 0.27 m | 14.7 % (by design) |
| C3 | rogue waypoint 150 m out | FENCE breach at 120.0 m → RTL | < 1 s | 0.31 m | 51.9 % |

### 9.4 Endurance check

Run D hovered in still air, holding at 8 m, until 80 % of the pack was used. It took **4.52 min at 27.7 A**, against the sizing model's 4.49 min and 27.8 A at +20 %. The sim includes voltage sag and the drop in thrust with voltage, so this agreement shows the endurance figure is internally consistent.

It is *not* independent evidence. The simulator uses the same FM, η and κ assumptions. The real check is Gate 5/6 on hardware.

---

## 10. Flight-log gate verification (Gates 4–11)

`src/analysis/flight_log_analyzer.py` reads real DataFlash `.BIN` logs through pymavlink (VIBE, CTUN, BAT, POS, ATT, MODE, RCOU/LGR, PARM, MSG) or the simulator CSVs. It applies:

| Gate | Criterion |
|---|---|
| G4 | 1 s mean VIBE < 15 m/s², no clipping |
| G5/G6 | hover current, lean-corrected by cos^1.5 θ, ≤ 1.1 × model; learned MOT_THST_HOVER; enclosure loss κ inferred from the log |
| G7 | ALT_HOLD within ±0.5 m once the target has been steady for 5 s |
| G9 | 60 s LOITER within a 2 m radius; the 1 m strict reading is also reported |
| G10 | every waypoint passed within 2 m; gear up in cruise; gear down ≥ 1 s before touchdown |
| G11 | ≥ 25 % at disarm |
| FS | RTL or LAND within 1 s, landing within 5 m of home |

Runs aborted by a failsafe are graded on FS only.

**Result on the simulated flights** (`outputs/logs/gate_compliance_report.json`, Fig. 8): **every evaluable gate passes.**

| Log | G4 vibe max (X/Y/Z) | G5/6 hover I vs model | G7 max err | G9 max r (p95) | G10 | G11 | FS |
|---|---|---|---|---|---|---|---|
| A (Stage 1) | 6.5 / 7.0 / 9.1 | 10.3 A vs 10.9 (0 %) | 0.22 m | 0.34 m (0.24) | — | — | — |
| B0 (Stage 2, wind) | 9.4 / 10.0 / 13.5 | 26.7 A vs 27.8 (+20 %); κ ≈ 22.6 % | 0.16 m | 1.07 m (0.76) | — | — | — |
| B (mission) | 9.7 / 9.9 / 13.7 | pass | — | — | ✓ all 6 WPs ≤ 0.2 m; gear retract at 5.1 m, deploy at 6.2 m by mission command | 58.4 % ✓ | — |
| C1 / C2 / C3 | ≤ 14.2 | pass | — | — | — | — | ✓ / ✓ / ✓ |
| D (still air) | 9.6 / 10.0 / 13.7 | 27.7 A vs 27.8 | 0.20 m | 0.40 m | — | — | — |

Gate 8 (AUTOTUNE) is pilot-flown and is reported as N/A. On a real log the analyzer flags ATUN messages or AUTOTUNE mode.

The vibration traces are synthetic, a throttle-proportional model, so G4 here only proves the pipeline works. The first real Stage 1 log is the real test.

---

## 11. Predicted vs Measured master table

| # | Quantity | Predicted | Source | Measured | Gate / how measured |
|---|---|---|---|---|---|
| 1 | Prop true diameter | 127.0 mm | BOM | | A: calipers |
| 2 | Printed duct ring mass | 55.5 g (BOM 50 raw / 57.5 counted) | CAD §6.3 | | C: scale |
| 3 | Grille mass | 23.2 g (BOM 17) | CAD | | C |
| 4 | Quarter mass | 89.0 g (BOM 69) | CAD | | C |
| 5 | Spine + bay mass | 48.4 g (BOM 30) | CAD | | C |
| 6 | All-up mass, Stage 1 | 1,131 g (legs) / 1,051 g (skids) | BOM | | scale |
| 7 | All-up mass, Stage 2 | 1,825.2 / 1,904.9 g; CAD-informed 1,959 g | BOM / Round 4 | | scale |
| 8 | CG offset, all states | ≤ 1.24 mm (limit 5) | §7 | | balance on pins |
| 9 | Open-prop max thrust | 1.76 kgf @ 24 V; 1.41 kgf usable | datasheet × 0.89 × 0.80 | | B: stand |
| 10 | Duct + grilles usable thrust (1.5 mm) | 1.456 kgf (gate ≥ 1.19) | §8 | | B |
| 11 | Duct gain at equal power | +1.5 % (1.5 mm), +3.7 % (1.0 mm) | duct_aero | | B |
| 12 | Best tip gap | 1.0 mm on stand; 1.5 mm in flight | §4 | | B |
| 13 | FM at hover (open) | 0.58–0.60 | §8 | | B |
| 14 | Hover power per motor at 476 gf | 126.6 W (0 %) / 151.9 W (+20 %) | Slide 8 | | B + G5 |
| 15 | Hover throttle MOT_THST_HOVER | 0.20 (S1) / 0.34–0.35 (S2); sim learned 0.156 / 0.32–0.35 | params / sim | | G6: CTUN.ThH |
| 16 | Hover current Stage 1 | 10.9 A (model) / 10.4 A (sim) | §3, §9 | | G5: BAT.Curr |
| 17 | Hover current Stage 2 | 23.3 / 27.8 A (0 / +20 %); sim 27.7 A | §3, §9 | | G5 |
| 18 | Enclosure power loss κ | 20 % assumed; sim-inferred 22 % | §10 | | G5: Stage 2 vs Stage 1 current |
| 19 | Hover time Stage 2 (legs) | 5.37 / 4.49 min; sim 4.52 min | §3, §9 | | timed hover to 20 % |
| 20 | Thrust-to-weight | 2.97 (legs) / 3.10 (skids) | §3 | | from #10 |
| 21 | Vibration | < 15 m/s² | G4 | | VIBE |
| 22 | ALT_HOLD error | ≤ 0.22 m (sim) | §10 | | G7: CTUN |
| 23 | LOITER 60 s radius | 0.34 m (S1) / 1.07 m (S2, 15 km/h) | §10 | | G9: POS |
| 24 | Waypoint passage error | ≤ 0.2 m; cross-track p95 1.45 m (15 km/h) / 0.72 m (10 km/h) | §9 | | G10 |
| 25 | Hover lean in 15 km/h wind | 18.8° (hand calc) / ~19–22° (sim) | §4, §9 | | ATT in loiter |
| 26 | Gear retract / deploy altitude | > 5 m / < 3 m (auto), 6 m by mission command | params | | G10: RCOU / LGR |
| 27 | Battery left after mission | 58.4 % (sim, 129 s); model 34–38 % for a 3.2 min mission | §9, §3 | | G11: BAT |
| 28 | Peak current | 160 A design; sim mission max 35 A | §3 | | BAT.Curr max |
| 29 | Failsafe reaction | < 1 s, land < 1 m from home | §9.3 | | bench + flight test |
| 30 | ESC temperature | within 60 A rating (~27 A/motor at hover) | §3 | | thermocouple / ESC log |

---

## 12. Risks and recommendations

1. **Wind margin.** This is the main finding of this phase.
   - Fly the graded mission in ≤ 10 km/h wind and abort at 15 km/h.
   - Keep the rate and velocity integrator limits at their defaults or higher.
   - After AUTOTUNE, consider ANGLE_MAX 35° for Stage 2.
   - Log ATT and RATE to measure the real duct-lip moment.
2. **Mass growth.** The CAD estimate is +54 g. Redesign the spine first (target the 30 g allowance plus 15 %), then the grille rim (3 → 2 mm, −3 g each). Weigh every printed part (Gate C) before the legs decision.
3. **The enclosure-loss assumption drives endurance.** It is the difference between 5.4 and 4.5 min. Measure it twice:
   - on the stand: duct + grille hover power at 476 gf;
   - in flight: Stage 2 against Stage 1 hover current, which the log verifier computes automatically as κ.
4. **Yaw authority.** WP_YAW_BEHAVIOR 0 and reduced yaw limits are already set. Log the CW vs CCW current difference during hover as the yaw-margin proxy.
5. **Hardware details still to confirm** (from the configuration work):
   - the SpeedyBee F405 V4 M5/M6 pads and their timer group, needed for the PWM gear servos;
   - the UART map;
   - BATT_AMP_PERVLT (set to 25 A/V for scale 400);
   - parameter renames in ArduPilot 4.6 (`GPS1_*`);
   - the gear PWM direction;
   - the motor remap (`SERVO1..4_FUNCTION` 36, 33, 34, 35), checked with Motor Test at Gate 2.

---

## Appendix A — Derivations

**A.1 Air density.** At 540 m, p ≈ 95.0 kPa; at 30 °C, T = 303.15 K. So ρ = p/(RT) = 95,000 / (287.05 × 303.15) = 1.092 ≈ 1.09 kg/m³, and the derating factor is ρ/ρ₀ = 1.09 / 1.225 = 0.890.

**A.2 Hover power (momentum theory).** Actuator-disc momentum balance gives T = 2ρA·v_i², so v_i = √(T/2ρA). Ideal power is P_i = T·v_i = T^{3/2}/√(2ρA). For 1,905 g:
- T_h = 1.905 × 9.81 / 4 = 4.672 N;
- √(2 × 1.09 × 0.012668) = 0.16618;
- P_i = 4.672^{1.5} / 0.16618 = 10.098 / 0.16618 = 60.8 W;
- electrical power P_e = P_i/(FM·η) = 60.8 / 0.48 = 126.6 W;
- with +20 % loss, 151.9 W.

**A.3 Current and endurance.**
- I_h = (4 × 126.6 + 10) / 22.2 = 23.3 A, and t_h = 0.8 × 2.6 / 23.3 h = 5.37 min.
- At +20 %: I = (4 × 151.9 + 10) / 22.2 = 27.8 A, and t = 4.49 min.

**A.4 Usable thrust.** Motor thrust at fixed throttle scales as ρn²D⁴, and loaded RPM scales roughly with V, so T ∝ ρV². Hence T_use = 1.98 × 0.890 × (21.5/24.0)² = 1.98 × 0.890 × 0.8025 = 1.414 kgf, and T/W = 4 × 1.414 / 1.905 = 2.97.

**A.5 Duct geometry.**
- d_b = 127 + 2 × 1.5 = 130.0 mm, and d_o = 130 + 2(6.5 + 1.6) = 146.2 mm.
- Exit diameter: 130 + 2 × 35 × tan 4.1° = 135.02 mm; area ratio (135.02/130)² = 1.079.
- Box: 2 × 97.5 + 146.2 + 2 × 5.9 = 353.0 mm. Duct gap: 195 − 146.2 = 48.8 mm, which leaves 3.8 mm against the 45 mm spine.

**A.6 Grille blockage.** β = 1 − ((s − b)/s)² = 1 − (11/12)² = 0.160. The open intake area relative to the disc is (1 − β)·d_o² / A = 0.840 × 0.021374 / 0.012668 = 1.42.

**A.7 Crosswind ram drag and lean.** Each fan swallows ṁ = ρA·2v_i. The incoming horizontal momentum is destroyed, so F_ram = Σṁ·V_w.
- At 1,905 g: v_i = 13.0 m/s, so Σṁ = 4 × 1.09 × 0.01267 × 26.0 = 1.44 kg/s, and F_ram = 1.44 × 4.17 = 5.99 N.
- Form drag is ½ρV²C_d·S = 0.5 × 1.09 × 17.4 × 1.15 × 0.0353 = 0.38 N.
- Lean: tan θ = 6.38 / 18.69, so θ = 18.8°.
- Required thrust rises by 1/cos θ − 1 = 5.7 %, so hover power rises by about 8.6 % (P ∝ T^{1.5}).

**A.8 Lean-corrected hover current (gate G5).** Holding station at tilt θ needs total thrust W/cos θ, so P ∝ (W/cos θ)^{1.5}. The verifier normalises a measured current to level hover with I_level = I·cos^{1.5}θ. This is conservative, because it ignores translational lift.

**A.9 Ballast position.** CG about x: Σmᵢxᵢ = 0. For the camera alone, x_b = −m_c·x_c / m_b = −9.8 × 30 / 170.2 = −1.73 mm. Over the whole aircraft, x_b = −Σ(other mᵢxᵢ) / m_b = +10.3 mm (§7).

**A.10 Battery reserve rule (from context.md).** left = 1 − 1.15·t_mission / (t_hover/0.8). For a 3.2 min mission at 4.78 min (+20 %, skids): 1 − 1.15 × 3.2 / 5.98 = 38.4 %. With legs at 4.49 min: 34.4 %.

**A.11 Stand coefficients.** C_T = T/(ρn²D⁴) and C_P = P_shaft/(ρn³D⁵) with P_shaft = 0.8·P_elec. Then FM = C_T^{1.5}/(√2·C_P), which is equivalent to FM = T^{1.5}/√(2ρA)/P_shaft, the form used because it needs no RPM.

**A.12 Hover throttle.** ArduPilot's thrust curve is F = (1 − e)u + eu², with e = MOT_THST_EXPO = 0.65. MOT_THST_HOVER is expressed in F (linearised thrust), so MOT_THST_HOVER ≈ T_h / T_max:
- Stage 1: 283 / 1,414 = 0.20;
- Stage 2: 476 / 1,362 = 0.35, where 1,362 g is 1,414 × 0.963 for the enclosure.

---

## Appendix B — Reproducing every result

```powershell
# One-time desktop tools (OpenSCAD, Bambu Studio, Arduino IDE, Mission Planner, pymavlink/numpy-stl/trimesh)
powershell -ExecutionPolicy Bypass -File setup_external_tools.ps1           # add -DryRun to preview

python src/sizing/duct_aero.py              # geometry, tip-gap sweep, crosswind
python src/sizing/sizing_model.py           # BOM, convergence loop, trade studies  -> outputs/sizing/*.json
python src/sizing/generate_plots.py         # fig1-fig5
python src/cad/generate_cad_stl.py          # 7 STLs + cad_mass_comparison.json  [--prop-d 126.4 --tip-gap 1.5]
python src/cad/mass_cg_audit.py             # CG in 9 states + inertia tensors, fig9
python src/sitl/mission_simulator.py        # runs A, B0, B, B25, B10, C1-C3, D -> outputs/logs, fig6  (~1.5 min)
python src/analysis/thrust_stand_analyzer.py            # synthetic; real: --csv stand_log.csv   -> fig7
python src/analysis/flight_log_analyzer.py              # sim CSVs; real: --log 00000012.BIN      -> fig8
```

The simulator reads its inertia from `outputs/sizing/mass_cg_inertia_audit.json`, so run `mass_cg_audit.py` first.

**Hardware Sync Gates**

| Gate | Action | Command / file |
|---|---|---|
| A | Caliper-measure the prop | `python src/cad/generate_cad_stl.py --prop-d <mm>` |
| B | Stage 0 stand, 5 configurations | `python src/analysis/thrust_stand_analyzer.py --csv <log>` |
| C | Weigh printed parts | update `mass_each_g` in `get_locked_5in_bom()`, then re-run sizing (Round 5) |
| D | Flight logs | `python src/analysis/flight_log_analyzer.py --log <file.BIN> --stage stage2` |

## Appendix C — Figures

| Fig | File | Content |
|---|---|---|
| 1 | `outputs/plots/fig1_sizing_convergence_and_bom.png` | sizing loop and mass breakdown |
| 2 | `fig2_prop_and_battery_trades.png` | 5 / 6 / 7-inch and battery trades |
| 3 | `fig3_duct_tip_gap_and_stage0_gate.png` | tip-gap model and decision gate |
| 4 | `fig4_stage1_vs_stage2_and_payload.png` | stage comparison and payload sensitivity |
| 5 | `fig5_crosswind_ram_drag_polar.png` | ram drag vs form drag, lean angle |
| 6 | `fig6_autonomous_mission_and_failsafes.png` | ground track, altitude and gear, battery, failsafes |
| 7 | `fig7_stage0_thrust_stand_results.png` | thrust, power, FM, C_T/C_P, gate, endurance (SYNTHETIC) |
| 8 | `fig8_flight_log_gate_verification.png` | Gates 4–11 |
| 9 | `fig9_mass_cg_inertia_audit.png` | layout, CG in all states, inertia |

## Appendix D — Deviations from the execution plan and verification status

| Item | Plan said | What was done / found |
|---|---|---|
| Stage 1 mass | 1.161 kg | BOM gives **1.131 kg** (legs, payload). Used throughout. |
| Ballast offset | −1.0 mm x | Camera alone needs −1.73 mm. The whole aircraft needs +10.3 / +3.0 mm, which requires a 24 × 32 × 28 mm steel block. |
| Mission | Gear check placed under DO_LAND_START | Gear retract uses `MAV_CMD_AIRFRAME_CONFIGURATION` (2520) after take-off. DO_LAND_START is a marker before the approach, and the final leg is NAV_LAND at home (more precise than RTL). |
| WP_YAW_BEHAVIOR | not specified | Set to 0 in all four param files (weak yaw axis). |
| Extra runs | — | B25, B10 (wind study) and D (endurance) were added to the simulator. |
| Round 4 sizing | — | CAD-informed mass (+54 g) was added to the loop. |
| Not verifiable here | | The OpenSCAD file is not compiled (OpenSCAD is not installed; checked by review and bracket balance). The Arduino sketch is not compiled (no toolchain; brace balance checked). The SITL frame JSON key names were written from memory. ArduPilot parameter names were not loaded into real firmware: check 4.6 renames on the first load (Mission Planner flags unknown names). |
| Synthetic data | | Stage 0 CSV, vibration traces and every flight log are **model outputs**. The analyzers are what is validated; the physics is not. |
