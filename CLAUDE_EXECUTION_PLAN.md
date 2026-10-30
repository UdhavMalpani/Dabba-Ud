# Project 8: Enclosed Ducted-Fan Quadrotor (5-Inch Locked Baseline) — Master Claude Code Execution Plan

> **INSTRUCTIONS FOR CLAUDE / AI AGENT:**
> You are working in the repository for **Project 8: Enclosed Ducted-Fan Quadrotor** (`EC4.402 Introduction to UAV Design`, IIIT Hyderabad, Monsoon 2026, Team 4).
> Your goal is to execute all software, simulation, parametric CAD/STL generation, autopilot configuration, Stage 0 thrust-stand analysis, flight-log verification, and report generation phases described in this document **end-to-end**, running and verifying every script after creating it.

---

## 1. Authoritative Design Context & Locked 5-Inch Baseline Numbers

**IMPORTANT RECONCILIATION NOTE:**
The file `context.md` (dated Oct 25) reflects an earlier pre-presentation exploration of a 7-inch build. However, in the locked Oct 27 deliverables (`Project 8 - Component List BOM final.pdf` and `Project 8 — Design Review Presentation.pdf`), **the 5-inch, 6S configuration is LOCKED IN as the primary build**, while 6-inch, 7-inch, and 4S are retained strictly for comparative trade-study charts in the report and fallback rules.

### 1.1 Environmental & System Constants (IIIT Hyderabad, 540 m ASL, 30 °C)
- **Air Density ($\rho$):** `1.09 kg/m³` (vs. sea-level $\rho_0 = 1.225\text{ kg/m}^3 \implies$ density derating factor $= 1.09 / 1.225 = 0.8898 \approx 0.89$)
- **Gravitational Acceleration ($g$):** `9.81 m/s²`
- **Rotor Figure of Merit ($\text{FM}$):** `0.60`
- **Combined Motor + ESC Efficiency ($\eta$):** `0.80`
- **Usable Battery Fraction:** `0.80` (80% usable capacity; land with $\ge 25\%$ remaining)
- **Avionics Power ($P_{\text{av}}$):** `10.0 W` (FC, GPS, RC receiver, telemetry radio, RunCam Thumb)
- **Voltage (6S LiPo):** `22.2 V` nominal, `21.5 V` loaded full-throttle sag vs `24.0 V` bench datasheet ($\implies (21.5/24.0)^2 = 0.8025 \approx 0.80$ voltage sag factor)
- **Mass Contingency Rule:** `+15%` (`× 1.15`) applied **strictly to Estimated (`Est`) parts**, never to Datasheet (`Spec`), Allowance (`Allow`), or `Defined` ballast parts.
- **Enclosure Hover Power Loss ($\kappa_{\text{encl}}$):** Evaluated at both `0%` (ideal duct) and `+20%` (conservative enclosed box loss).

### 1.2 Locked 5-Inch Duct, Grille & Box Geometry (Slides 3 & 4)
| Parameter | Symbol / Formula | Locked 5-Inch Value | Notes / Rule |
| :--- | :--- | :--- | :--- |
| **Propeller Diameter** | $D$ | `127.0 mm` (`5.0 in`) | `5 × 4.3` tri-blade (`HQProp / Gemfan`), $A = \pi D^2/4 = 0.0127\text{ m}^2$ |
| **Radial Tip Gap** | $\delta$ | `1.5 mm` baseline | Stage 0 sweeps `1.0 / 1.5 / 2.0 mm` (arms flex `~0.5 mm` at full thrust) |
| **Duct Bore Diameter** | $d_b = D + 2\delta$ | `130.0 mm` | `129.0 / 130.0 / 131.0 mm` across Stage 0 test rings |
| **Inlet Lip Radius** | $r_{\text{lip}} = 0.05 d_b$ | `6.5 mm` | Bellmouth inlet stops lip flow separation |
| **Duct Wall Thickness** | $t$ | `1.6 mm` | Printed upright in PETG (or PETG-CF) |
| **Duct Outer Diameter** | $d_o = d_b + 2(r_{\text{lip}} + t)$ | `146.2 mm` | Fits inside each quarter cell |
| **Duct Axial Length** | $L$ | `50.0 mm` ($L/D = 0.39$) | Rotor plane at $0.30 L = 15.0\text{ mm}$ below inlet |
| **Diffuser Section** | $L_{\text{diff}} = 35.0\text{ mm}$, $\theta \le 5^\circ$ | `135.0 mm` exit ID | Half-angle $\approx 4.1^\circ$, area ratio $\approx 1.08$ |
| **Motor Axis Offset** | $a$ (from center in X & Y) | `97.5 mm` | Diagonal wheelbase $2\sqrt{2}a = 275.8 \approx 276\text{ mm}$ (True-X frame) |
| **Duct-to-Duct Gap** | $2a - d_o$ | `48.8 mm` | Clears the `45.0 mm` cross spine by `3.8 mm` ($\checkmark$) |
| **Outer Cuboid Box** | $W = 2a + d_o + 2(5.9)$ | `353 × 353 × 100 mm` | Printed as **4 PLA quarters of `176.5 × 176.5 mm`** (fits `256 mm` Bambu P1S bed) |
| **Grilles (×8, Top/Bot)** | $b = 1\text{ mm}$ bar, $s = 12\text{ mm}$ pitch | `4.0 mm` deep (`16%` block) | `11.0 mm` finger-safe openings; $\beta = 1 - (11/12)^2 = 0.16$; intake $= 1.42\times$ disc |
| **Payload Bay** | Central CG bay | `60 × 40 × 30 mm` | Flush hinged door + RunCam Thumb (`9.8 g`) window + `170.2 g` ballast (`180 g` total) |
| **Exhaust Clearance** | $\ge 1.5 \times d_o$ | `≥ 219.3 mm` | Ground cradle `≥ 250 mm` (skids baseline) or `~240 mm` retractable legs |

### 1.3 Locked 5-Inch Mass Budget & Performance (Slides 6, 8, 9 & BOM PDF)
- **Primary Baseline (5-Inch + Flush TPU Skids + Ground Cradle):**
  - `Spec` = `585.4 g` | `Est raw` = `904.0 g` $\to$ `Est × 1.15` = `1,039.6 g` | `Allow + Ballast` = `200.2 g` (`30 g` radio + `170.2 g` ballast)
  - **All-Up Mass ($m$):** **`1,825.2 g`**
  - **Usable Max Thrust per Motor ($T_{\max}$):** $1.98 \times (1.09/1.225) \times (21.5/24.0)^2 = \mathbf{1.41\text{ kgf}}$
  - **Thrust-to-Weight ($\text{T/W}$):** $4 \times 1.41 / 1.8252 = \mathbf{3.10}$
  - **Hover Power & Current (0% / +20% loss):** $I_h = 22.1\text{ A} / 26.4\text{ A}$, **Hover Time = `5.7 min` (0% loss) / `4.8 min` (+20% loss)**
- **Upgrade Option (5-Inch + Retractable Legs — 2 × MG90S + 5V 3A UBEC + 240 mm legs):**
  - Replaces `27.6 g` counted skids with `107.3 g` counted leg assembly (`+79.7 g` net)
  - `Spec` = `612.2 g` | `Est raw` = `950.0 g` $\to$ `Est × 1.15` = `1,092.5 g` | `Allow + Ballast` = `200.2 g`
  - **All-Up Mass ($m$):** **`1,904.9 g`** ($\approx 1,905\text{ g}$ on Slide 8)
  - **Slide 8 Exact Sizing Equations at $1,905\text{ g}$:**
    - $T_h = mg/4 = 1.905 \times 9.81 / 4 = \mathbf{4.67\text{ N}}$ (`476.2 gf`)
    - $P_i = T_h^{3/2} / \sqrt{2 \rho A} = \mathbf{60.8\text{ W}}$ per motor
    - $P_e = P_i / (\text{FM} \cdot \eta) = 60.8 / (0.6 \times 0.8) = \mathbf{126.6\text{ W}}$ per motor (0% loss) / `151.9 W` (+20% loss)
    - $I_h = (4 P_e + P_{\text{av}}) / 22.2 = 516.4 / 22.2 = \mathbf{23.3\text{ A}}$ (0% loss) / `27.8 A` (+20% loss)
    - $\text{T/W} = 4 \times 1.41 / 1.905 = \mathbf{2.97}$
    - **Hover Time:** $t_h = 0.8 \times 2.6\text{ Ah} / I_h = \mathbf{5.4\text{ min}}$ (0% loss) / **`4.5 min`** (+20% loss)
    - **Peak Current Check:** $4 \times 40\text{ A} = 160\text{ A} \le 2.6\text{ Ah} \times 65\text{C} = 169\text{ A}$ ($\checkmark$)

---

## 2. Target Repository Structure to Build and Verify

```text
UAV-Design/
├── CLAUDE_EXECUTION_PLAN.md                  # This master execution specification
├── setup_external_tools.ps1                  # PowerShell installer for Mission Planner, Bambu Studio, OpenSCAD, etc.
├── src/
│   ├── sizing/
│   │   ├── duct_aero.py                      # [EXISTS - VERIFY/RUN] Bellmouth duct, grille & crosswind ram-drag model
│   │   ├── sizing_model.py                   # [EXISTS - VERIFY/RUN] BOM, momentum sizing loop & trade-study engine
│   │   └── generate_plots.py                 # [EXISTS - VERIFY/RUN] Generates 5 multi-panel plots in outputs/plots/
│   ├── sim/
│   │   └── box_lab_v3.html                   # [EXISTS - VERIFY] Interactive 14-check simulator (5" skids/legs, 7", 4S)
│   ├── cad/
│   │   ├── generate_cad_stl.py               # [CREATE & RUN] Pure-Python binary STL generator for all 5" printed parts
│   │   └── mass_cg_audit.py                  # [CREATE & RUN] 3D CG shift (<=5mm) & inertia tensor (Ixx, Iyy, Izz) audit
│   ├── sitl/
│   │   └── mission_simulator.py              # [CREATE & RUN] 6-DOF closed-loop Stage 1 & Stage 2 flight/failsafe simulator
│   └── analysis/
│       ├── thrust_stand_analyzer.py          # [CREATE & RUN] Stage 0 thrust-stand parser, gap sweep & Slide 6 gate checker
│       └── flight_log_analyzer.py            # [CREATE & RUN] ArduPilot .BIN / telemetry log Gate 4-11 verifier
├── cad/
│   ├── project8_5in_parametric.scad          # [CREATE] Parametric OpenSCAD source for ducts, grilles, quarters, spine, legs
│   └── fusion360_parameters_5in.csv          # [CREATE] Ready-to-import Fusion 360 user parameter table (Modify -> Change Parameters)
├── config/
│   ├── ardupilot/
│   │   ├── stage1_bare_5in.param             # [CREATE] SpeedyBee F405 V4 parameters for Stage 1 Bare Quad
│   │   ├── stage2_enclosed_5in_skids.param   # [CREATE] SpeedyBee F405 V4 parameters for Stage 2 Enclosed (Skids)
│   │   ├── stage2_enclosed_5in_legs.param    # [CREATE] SpeedyBee F405 V4 parameters for Stage 2 Enclosed (Retractable Legs)
│   │   └── stage2_4s_contingency.param       # [CREATE] 4S 4000 mAh / 2400KV contingency parameter set
│   └── sitl/
│       ├── quad_stage1_bare.parm             # [CREATE] ArduPilot SITL physics parameters (Stage 1)
│       └── quad_stage2_enclosed_5in.parm     # [CREATE] ArduPilot SITL physics parameters (Stage 2)
├── missions/
│   └── iiith_5in_enclosed_mission.waypoints  # [CREATE] QGC WPL 110 autonomous mission (Takeoff, 6 WPs, Gear, RTL/Land)
├── firmware/
│   └── thrust_stand_logger/
│       └── thrust_stand_logger.ino           # [CREATE] Arduino HX711 + ESC PWM + Current + Optical RPM automated sweep sketch
├── docs/
│   └── wiring_and_bringup_guide.md           # [CREATE] Pin-accurate SpeedyBee F405 V4 wiring & 11-Gate flight test checklist
├── report/
│   └── Project8_Technical_Analysis_and_Validation.md # [CREATE] Complete report body, derivations appendix & Predicted-vs-Measured table
└── outputs/
    ├── sizing/                               # JSON outputs from sizing_model.py and mass_cg_audit.py
    ├── plots/                                # All PNG plots from sizing, Stage 0 thrust stand, and SITL mission simulation
    ├── stl/                                  # Ready-to-slice binary .STL files for Bambu P1S (256x256 mm bed)
    └── logs/                                 # Simulated Stage 0 CSVs and Stage 1/2 flight telemetry logs
```

---

## 3. Step-by-Step Execution Tasks for Claude

### Task 1: Run & Verify Phase 1 (Sizing, Aero & Trade-Study Suite)
`src/sizing/duct_aero.py`, `src/sizing/sizing_model.py`, `src/sizing/generate_plots.py`, and `src/sim/box_lab_v3.html` are already in place.
1. Run:
   ```powershell
   python src/sizing/duct_aero.py
   python src/sizing/sizing_model.py
   python src/sizing/generate_plots.py
   ```
2. Verify that `outputs/sizing/sizing_and_trade_studies_5in.json` and all 5 PNG plots in `outputs/plots/` (`fig1` through `fig5`) are generated cleanly.

---

### Task 2: Build & Run Phase 2 (Parametric 3D CAD, Binary `.STL` Generator & 3D Mass/CG/Inertia Audit)
1. **Create `cad/fusion360_parameters_5in.csv`:**
   - Must follow Autodesk Fusion 360's exact CSV import columns: `Name,Unit,Expression,Value,Comments,Favorite`
   - Include `prop_d (127 mm)`, `tip_clear (1.5 mm)`, `bore_d (130 mm)`, `lip_r (6.5 mm)`, `wall_t (1.6 mm)`, `duct_od (146.2 mm)`, `duct_len (50 mm)`, `rotor_depth (15 mm)`, `exit_d (135 mm)`, `spine_w (45 mm)`, `motor_offset_a (97.5 mm)`, `box_w (353 mm)`, `quarter_w (176.5 mm)`, `box_h (100 mm)`, `grille_bar (1 mm)`, `grille_pitch (12 mm)`, `grille_depth (4 mm)`, `payload_L (60 mm)`, `payload_W (40 mm)`, `payload_H (30 mm)`, `leg_len (240 mm)`.
2. **Create `cad/project8_5in_parametric.scad`:**
   - Complete parametric OpenSCAD file with a `part_to_render` selector (`"duct_ring"`, `"grille"`, `"enclosure_quarter"`, `"cross_spine_payload"`, `"retractable_leg"`, `"full_assembly"`) so the user can open it in OpenSCAD and tweak any dimension interactively.
3. **Create & Run `src/cad/generate_cad_stl.py` (Pure Python + NumPy Binary STL Mesh Generator):**
   - Do **not** rely on external CAD binaries being installed. Write a clean triangulated mesh generator in pure Python/NumPy that directly writes valid binary `.stl` files to `outputs/stl/`:
     - `duct_ring_5in_gap1.0mm.stl` (Stage 0 test ring, `129.0 mm` bore, `145.2 mm` OD, `50 mm` tall, revolved bellmouth + `4.1°` diffuser + carbon arm slot)
     - `duct_ring_5in_gap1.5mm.stl` (Baseline flight ring, `130.0 mm` bore, `146.2 mm` OD)
     - `duct_ring_5in_gap2.0mm.stl` (Wide test ring, `131.0 mm` bore, `147.2 mm` OD)
     - `grille_5in_146mm_4mm.stl` (`146.2 × 146.2 × 4.0 mm` finger-safe grid, `1 mm` bar, `12 mm` pitch)
     - `enclosure_quarter_5in_177mm.stl` (`176.5 × 176.5 × 100 mm` quarter enclosure shell with `146.2 mm` duct pocket and spine joint flange — fits the `256 × 256 mm` Bambu P1S bed!)
     - `cross_spine_45mm_payload_bay.stl` (Central `45 mm` cross spine section with `60 × 40 × 30 mm` payload bay & `RunCam Thumb` front window)
     - `retractable_leg_240mm.stl` (`240 mm` tapered leg with lateral foot & servo horn boss)
   - Compute the exact mesh volume ($\text{cm}^3$) and sliced mass (using PETG $\rho = 1.27\text{ g/cm}^3$ with perimeters/infill effective density, and PLA $\rho = 1.24\text{ g/cm}^3$) and print a comparison table against the BOM estimates.
4. **Create & Run `src/cad/mass_cg_audit.py`:**
   - Define the 3D coordinate $(x, y, z)$ in mm and mass $m_i$ (in grams) of every single component on the aircraft (4 motors, 4 ducts, 8 grilles, 4 quarters, X-frame, 6S 2600 battery at center under/in spine, FC+ESC stack at center, GPS on top mast, RC+telemetry balanced across spine, RunCam Thumb at $+30\text{ mm}$ X offset counterbalanced by ballast at $-1.0\text{ mm}$ X offset inside the $60 \times 40 \times 30\text{ mm}$ bay, skids vs retractable legs in UP and DOWN states).
   - Verify and output proof that $|\mathbf{r}_{\text{CG}}| \le 5.0\text{ mm}$ in **all 4 states** (Payload IN/OUT × Legs UP/DOWN).
   - Compute the full 3×3 Inertia Tensor ($I_{xx}, I_{yy}, I_{zz}$) for both **Stage 1 Bare (`1.16 kg`)** and **Stage 2 Enclosed (`1.83 kg` skids / `1.90 kg` legs)** and save to `outputs/sizing/mass_cg_inertia_audit.json`.

---

### Task 3: Build & Run Phase 3 (ArduPilot `.param` Files, `.waypoints` Mission & 6-DOF Closed-Loop Flight Simulator)
1. **Create the 4 SpeedyBee F405 V4 (`speedybeef4v4`) Parameter Files in `config/ardupilot/`:**
   - `stage1_bare_5in.param` (`MOT_THST_HOVER = 0.20`, low initial `ATC_RAT_PIT/RLL_P = 0.085`, `INS_HNTCH_ENABLE = 1`, `BATT_LOW_VOLT = 21.0`, `BATT_CRT_VOLT = 19.8`, `MOT_BAT_CURR_MAX = 160`)
   - `stage2_enclosed_5in_skids.param` (`MOT_THST_HOVER = 0.34`, enclosed inertia gains, `WPNAV_SPEED = 350`, `WPNAV_ACCEL = 120`, `LAND_SPEED = 40`, `FS_THR_ENABLE = 1`, `FENCE_ENABLE = 1`, `FENCE_RADIUS = 120`, `FENCE_ALT_MAX = 40`, `GPS_HDOP_GOOD = 140`)
   - `stage2_enclosed_5in_legs.param` (adds `SERVO5_FUNCTION = 29`, `SERVO6_FUNCTION = 29`, `LGR_DEPLOY_ALT = 3`, `LGR_RETRACT_ALT = 5`, `SERVO5_MIN = 1100`, `SERVO5_MAX = 1900`)
   - `stage2_4s_contingency.param` (`BATT_LOW_VOLT = 14.0`, `BATT_CRT_VOLT = 13.2`, `MOT_BAT_VOLT_MAX = 16.8`, `MOT_BAT_VOLT_MIN = 13.2`, `MOT_BAT_CURR_MAX = 180`)
2. **Create `config/sitl/quad_stage1_bare.parm` and `config/sitl/quad_stage2_enclosed_5in.parm`:**
   - Ready for ArduPilot `sim_vehicle.py -v ArduCopter --add-param-file=...`
3. **Create `missions/iiith_5in_enclosed_mission.waypoints`:**
   - Standard `QGC WPL 110` format centered at IIIT Hyderabad sports ground (`17.4455° N, 78.3489° E, 540 m ASL`).
   - Sequence: `HOME` $\to$ `NAV_TAKEOFF` (`12 m`) $\to$ `DO_LAND_START` / Gear retract check $\to$ 6 GPS waypoints forming a safe `35 m × 25 m` box pattern with `5 s` `NAV_LOITER_TIME` holds $\to$ `NAV_RETURN_TO_LAUNCH` / `NAV_LAND` with automatic landing gear deploy below `3 m`.
4. **Create & Run `src/sitl/mission_simulator.py`:**
   - A standalone, deterministic **6-DOF Closed-Loop Quadrotor Physics + Cascaded PID Autopilot Simulator** in Python that loads `missions/iiith_5in_enclosed_mission.waypoints` and simulates:
     - **Run A:** Stage 1 Bare Quad (`1.161 kg`) flying the hover & loiter check
     - **Run B:** Stage 2 Enclosed 5-Inch Quad (`1.905 kg` with retractable legs) flying the full 6-waypoint mission in `15 km/h` (`4.17 m/s`) crosswind + turbulence, including duct momentum ram drag, battery discharge (`2600 mAh 6S`), and automatic landing gear retract (`>5 m`) & deploy (`<3 m`)
     - **Run C:** Failsafe verification runs (RC-loss mid-mission $\to$ automatic RTL; Low-battery threshold $\to$ automatic RTL/Land; Geofence test)
   - Outputs synthetic ArduPilot telemetry logs (`outputs/logs/stage1_flight_telemetry.csv` and `outputs/logs/stage2_mission_telemetry.csv`) and generates a 4-panel mission verification plot (`outputs/plots/fig6_autonomous_mission_and_failsafes.png`).
5. **Create `docs/wiring_and_bringup_guide.md`:**
   - Complete pad-by-pad wiring table for SpeedyBee F405 V4 + BLS 60A ESC + Matek M10Q-5883 GPS + Lab RC/Telemetry + 5V 3A UBEC + 2× MG90S leg servos + RunCam Thumb, plus the 11-Gate Flight Test Checklist.

---

### Task 4: Build & Run Phase 4 (Stage 0 Thrust-Stand Suite, Flight Log Gate Verifier & Final Report)
1. **Create `firmware/thrust_stand_logger/thrust_stand_logger.ino`:**
   - Complete Arduino sketch for BOM Item #38 (`5 kg load cell + HX711 + ESC Servo PWM + analog voltage/current sensor + optical/ESC RPM interrupt`), executing an automated throttle ramp (`1000` to `2000 µs` in `5%` steps, `3.0 s` settle, `20` samples averaged per step) with hardware safety cutoff.
2. **Create & Run `src/analysis/thrust_stand_analyzer.py`:**
   - Generates realistic Stage 0 calibration & test datasets (`outputs/logs/stage0_thrust_stand_sweep.csv`) for the Emax ECO II 2207 1700KV at 6S across 5 configurations:
     1. Open Prop `5×4.3` 3-Blade
     2. Open Prop `5×4.5` 2-Blade (Decision D3 comparison)
     3. Duct + Grilles (`1.0 mm` tip gap)
     4. Duct + Grilles (`1.5 mm` tip gap — Flight Baseline)
     5. Duct + Grilles (`2.0 mm` tip gap)
   - Parses the CSV, computes $C_T$, $C_P$, $\text{FM}$, and usable thrust at $21.5\text{ V}$ ($\rho = 1.09\text{ kg/m}^3$), evaluates the **Slide 6 Decision Gate (`>= 1.19 kgf`)**, and generates `outputs/plots/fig7_stage0_thrust_stand_results.png`.
3. **Create & Run `src/analysis/flight_log_analyzer.py`:**
   - Supports both real ArduPilot DataFlash `.BIN` logs (via `pymavlink.DFReader` when installed) and the CSV telemetry logs from `mission_simulator.py`.
   - Automatically evaluates **Gates 4 through 11** (Vibration `< 15 m/s²`, Stage 1 vs Stage 2 hover current & learned `MOT_THST_HOVER`, Altitude Hold `±0.5 m`, 60s Loiter within `2.0 m` circle, Waypoint tracking error `<= 2.0 m`, Landing Gear deployment below `3 m`, and Post-Mission Battery Reserve `>= 25%`) and generates `outputs/plots/fig8_flight_log_gate_verification.png` + `outputs/logs/gate_compliance_report.json`.
4. **Create `report/Project8_Technical_Analysis_and_Validation.md`:**
   - The complete, publication-ready technical report body + mathematical derivations appendix + **Section 11 Predicted-vs-Measured Master Table** populated with all computed figures and links to the generated plots and STLs.

---

## 4. Desktop Software Installation Script (`setup_external_tools.ps1`)
Also create `setup_external_tools.ps1` in the root directory so the user can install any missing desktop GUIs on Windows with one command:
- **OpenSCAD** (`winget install --id OpenSCAD.OpenSCAD -e`)
- **Bambu Studio** (`winget install --id Bambultd.BambuStudio -e`)
- **Arduino IDE** (`winget install --id ArduinoSA.IDE.stable -e`)
- **Mission Planner** (direct `.msi` download from `https://firmware.ardupilot.org/Tools/MissionPlanner/MissionPlanner-latest.msi`)
- **Python MAVLink & Mesh packages** (`pip install pymavlink numpy-stl trimesh`)

---

## 5. Hardware Sync Gates (Where Physical Hardware Plugs Into This Software)

| Sync Gate | Physical Action Required | File / Variable to Update When Hardware Arrives |
| :--- | :--- | :--- |
| **Gate A: Prop Caliper Check** | Measure true diameter of the `5×4.3` tri-blade prop vs `127.0 mm`. | Update `prop_d_mm` in `src/cad/generate_cad_stl.py` and re-run to regenerate `outputs/stl/duct_ring_5in_gap1.5mm.stl`. |
| **Gate B: Stage 0 Thrust Stand** | Run 1 motor + printed test duct + grilles on the lab thrust stand. | Drop the serial CSV into `src/analysis/thrust_stand_analyzer.py --csv <file>` to auto-verify the `>= 1.19 kgf` Retractable Legs gate. |
| **Gate C: Weighed Printed Parts** | Weigh the first printed PETG duct ring and PLA grille on the lab scale. | Update `mass_each_g` in `src/sizing/sizing_model.py` (`get_locked_5in_bom`) for Round 4 of the sizing loop. |
| **Gate D: Outdoor Flight Logs** | Pull the `.BIN` log from the SpeedyBee F405 V4 SD card after Stage 1 & Stage 2 flights. | Run `python src/analysis/flight_log_analyzer.py --log <flight.BIN>` to generate the final measured charts for the December evaluation. |
