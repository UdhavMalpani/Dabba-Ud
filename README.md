# Project 8 — Enclosed Ducted-Fan Quadrotor (5-inch, 6S)

This is the design, simulation and verification pipeline for a fully enclosed ducted-fan quadrotor:

- four 5-inch tri-blade props in printed ducts;
- a 353 × 353 × 100 mm PETG box;
- an internal 180 g payload bay;
- retractable legs;
- a SpeedyBee F405 V4 running ArduPilot.

Everything below runs from this repo on a laptop. No hardware is needed.

> **Status (31 Oct 2026):** none of the evidence here comes from hardware yet. The thrust-stand data and flight logs are **synthetic** (model-generated), and each one is labelled that way in the outputs. Every script also accepts real data; see [Feeding in real hardware data](#feeding-in-real-hardware-data).

The full write-up is in [`report/Project8_Technical_Analysis_and_Validation.md`](report/Project8_Technical_Analysis_and_Validation.md). The open work is listed in [`NEXT_STEPS.md`](NEXT_STEPS.md).

---

## 1. Setup

You need Python 3.10+ (developed and tested on 3.11).

```bash
git clone https://github.com/UdhavMalpani/Dabba-Ud.git
cd Dabba-Ud
python -m venv .venv
# Windows:  .venv\Scripts\activate      Linux/macOS:  source .venv/bin/activate
pip install -r requirements.txt
```

`pymavlink` is only needed for reading real ArduPilot `.BIN` logs. Everything else runs on `numpy`, `matplotlib` and `pandas`.

Optional desktop tools are OpenSCAD, Bambu Studio, Arduino IDE and Mission Planner. On Windows you can install them with:

```powershell
powershell -ExecutionPolicy Bypass -File setup_external_tools.ps1 -DryRun   # preview
powershell -ExecutionPolicy Bypass -File setup_external_tools.ps1           # install
```

---

## 2. Simulate everything (run in this order)

Run every command from the repo root. Later steps read the JSON that earlier steps write, so keep this order.

| # | Command | What it does | Main outputs |
|---|---|---|---|
| 1 | `python src/sizing/duct_aero.py` | Duct geometry, tip-gap loss sweep, crosswind ram drag | console |
| 2 | `python src/sizing/sizing_model.py` | BOM, mass/thrust convergence loop, prop and battery trade studies | `outputs/sizing/sizing_and_trade_studies_5in.json` |
| 3 | `python src/sizing/generate_plots.py` | Sizing figures | `outputs/plots/fig1`–`fig5` |
| 4 | `python src/cad/generate_cad_stl.py` | 7 print-ready STLs (ducts at 3 tip gaps, enclosure quarter, grille, spine, leg), plus printed-mass estimate | `outputs/stl/*.stl`, `outputs/sizing/cad_mass_comparison.json` |
| 5 | `python src/cad/mass_cg_audit.py` | 98-body mass, CG and inertia model across 9 configurations | `outputs/sizing/mass_cg_inertia_audit.json`, `fig9` |
| 6 | `python src/sitl/mission_simulator.py` | 6-DOF closed-loop flight sim: ArduCopter-style controller, wind and gusts, battery model, failsafes (about 1.5 min) | `outputs/logs/*.csv`, `mission_sim_summary.json`, `fig6` |
| 7 | `python src/analysis/thrust_stand_analyzer.py` | Stage 0 thrust-stand analysis (C_T, C_P, FM) and the enclose / don't-enclose decision gate | `outputs/sizing/stage0_thrust_stand_results.json`, `fig7` |
| 8 | `python src/analysis/flight_log_analyzer.py` | Checks the flight logs against acceptance Gates 4–11 and the failsafes | `outputs/logs/gate_compliance_report.json`, `fig8` |

Step 6 loads inertia from step 5's JSON, so always run `mass_cg_audit.py` before `mission_simulator.py`.

The same pipeline as one script:

```bash
python src/sizing/duct_aero.py && python src/sizing/sizing_model.py && python src/sizing/generate_plots.py && \
python src/cad/generate_cad_stl.py && python src/cad/mass_cg_audit.py && \
python src/sitl/mission_simulator.py && \
python src/analysis/thrust_stand_analyzer.py && python src/analysis/flight_log_analyzer.py
```

### Simulator runs

`mission_simulator.py --runs A,B,C1` runs only the listed runs. With no `--runs` it runs all of them.

| Run | Scenario |
|---|---|
| A | Stage 1 bare frame (1.131 kg): ALT_HOLD, then LOITER, then LAND in a light breeze |
| B0 | Stage 2 enclosed (1.905 kg): hover and loiter in 15 km/h wind |
| B | Full autonomous mission (`missions/iiith_5in_enclosed_mission.waypoints`) in 15 km/h wind |
| B25 | Mission B at 2.5 m/s cruise |
| B10 | Mission B in 10 km/h wind |
| C1 | RC loss at 45 s, expecting RTL |
| C2 | Starts at 42 % battery, expecting low-battery RTL then LAND |
| C3 | Rogue waypoint 150 m out, expecting geofence RTL |
| D | Still-air endurance hover to 80 % battery used (failsafes off) |

Use `--mission <file.waypoints>` to fly a different QGC WPL 110 mission.

### Browser sandbox

[`src/sim/box_lab_v3.html`](src/sim/box_lab_v3.html) is an interactive simulator in a single file. Open it in any browser to explore the enclosed airframe's handling.

### Real ArduPilot SITL (optional, Linux/WSL)

[`config/sitl/`](config/sitl/) holds the SITL setup: a frame-model JSON (mass, inertia, thrust) for each stage plus a self-contained `.parm` file. From an ArduPilot checkout:

```bash
Tools/autotest/sim_vehicle.py -v ArduCopter -f quad \
  --model "quad:/abs/path/Dabba-Ud/config/sitl/project8_5in_enclosed.json" \
  --add-param-file=/abs/path/Dabba-Ud/config/sitl/quad_stage2_enclosed_5in.parm --console --map
```

Then, in MAVProxy, load the mission with `wp load /abs/path/Dabba-Ud/missions/iiith_5in_enclosed_mission.waypoints`. Arm and switch to AUTO. To check the result, run the `.BIN` log through step 8.

To test failsafes:

- RC loss: `param set SIM_RC_FAIL 1`;
- low battery: `param set SIM_BATT_VOLTAGE 20.5`.

Don't load `config/ardupilot/*.param` into SITL. Those files are for the real board, and their serial-port settings turn off the simulated GPS.

---

## 3. Feeding in real hardware data

| Gate | When | Command |
|---|---|---|
| A | Props measured with calipers | `python src/cad/generate_cad_stl.py --prop-d 126.4 --tip-gap 1.5` |
| B | Thrust-stand CSV from [`firmware/thrust_stand_logger`](firmware/thrust_stand_logger/thrust_stand_logger.ino) | `python src/analysis/thrust_stand_analyzer.py --csv stand_log.csv [--rho-test 1.09]` |
| C | Parts weighed | Update the BOM masses in `src/sizing/sizing_model.py`, then re-run steps 2–6 |
| D | Flight logs (`.BIN` from the SD card) | `python src/analysis/flight_log_analyzer.py --log 00000012.BIN --stage stage2` (repeat `--log` for each log) |

Record the measured values in the **Measured** column of report §11.

---

## 4. Repository layout

```
src/sizing/      duct_aero.py, sizing_model.py, generate_plots.py   - momentum-theory sizing and trade studies
src/cad/         generate_cad_stl.py, mass_cg_audit.py              - STL generation, mass/CG/inertia
src/sitl/        mission_simulator.py                               - 6-DOF closed-loop mission and failsafe sim
src/analysis/    thrust_stand_analyzer.py, flight_log_analyzer.py   - Stage 0 gate and flight-gate verification
src/sim/         box_lab_v3.html                                    - browser simulator
cad/             Fusion 360 parameter table, parametric OpenSCAD model
config/ardupilot Real-board parameter files: Stage 1 bare, Stage 2 skids/legs, 4S contingency
config/sitl/     SITL frame models and .parm files
missions/        IIIT Hyderabad sports-ground mission (QGC WPL 110)
firmware/        Arduino HX711 thrust-stand logger
docs/            Wiring, bring-up and 11-gate flight checklist
report/          Technical analysis and validation report
outputs/         Pre-generated results (plots, STLs, JSON, logs); the scripts regenerate them
```

## 5. Headline results (predicted)

- **Mass and thrust:** all-up mass 1,905 g with legs (1,959 g with CAD-estimated printed parts); T/W 2.97.
- **Hover endurance:** 5.4 min with no enclosure loss, 4.5 min with a +20 % loss.
- **CG:** within 1.24 mm of the centre in all 9 configurations.
- **Mission in 15 km/h wind:** all 6 waypoints within 0.2 m; lands with 58 % battery (requirement ≥ 25 %).
- **Failsafes:** RC loss, low battery and geofence all respond in under 1 s.
- **Main risk is wind.** At 15 km/h the aircraft needs about 19° of lean, and gusts reach the 30° limit. Fly the demo in winds of 10 km/h or less.
