# Pre-Component Test Guide: Single-Duct Thrust Rig

**Branch:** `pre-component-test` · **Started:** 3 Nov 2026 · **Status:** rig designed and checked; nothing printed or measured yet.

The flight parts (2207 1700 KV motors, F405 V4 stack, 6S packs) arrive in about a week. Until then, this branch uses the equipment already in the lab to measure the number the whole design depends on: **how much extra power the duct and grilles cost compared with an open prop.** The sizing assumes **+20 %**, and nothing has tested that yet. The 6-DOF simulator uses the same assumption, so it can't confirm it either.

---

## 1. Equipment in hand (3 Nov 2026)

| Item | Qty | Use here |
|---|---|---|
| Emax ECO II 2807 1300 KV | 1 | Test motor, option A |
| A2217/6T 1500 KV outrunner | 1 | Test motor, option B. **Preferred**: its higher KV is closer to the real hover RPM |
| 3-blade prop, **65.1 mm per blade → Ø 130.2 mm** | 5 | Test prop (measured 3 Nov) |
| 2200 mAh LiPo (cell count: check the label) | 1 | Power |
| BLHeli 30 A ESC | 2 | One drives the test motor |
| ArduPilot flight controller | 1 | Throttle source via Mission Planner Motor Test |
| Bambu P1S printers | — | All rig parts |

**Still needed:**
- a 5 kg / 1 g kitchen scale;
- an inline XT60 watt meter, needed for the power-penalty result;
- M3 screws: 4 × M3×8, 2 × M3×20, 1 × M3×16;
- zip ties and tape;
- calipers;
- safety glasses.

### Expected thrust
These estimates scale from the sizing model's motor data; they are not measured. The test is about ratios, so low absolute thrust is fine.

| Motor, Ø 130.2 mm prop | 3S | 4S |
|---|---|---|
| 2807 1300 KV | ~0.22 kgf | ~0.39 kgf |
| A2217 1500 KV | ~0.29 kgf | ~0.51 kgf, about 10–12 A |

Both are well inside a 30 A ESC. **Check that the prop's bore fits the motor shaft or adapter**: the 2807 has a 5 mm shaft, and the A2217 usually has a 3.17 mm shaft with a prop adapter.

---

## 2. Important finding: this prop doesn't fit the flight ducts

The prop measures **130.2 mm**, but the flight duct STLs in `outputs/stl/` were built for **127 mm**, with a bore of **130.0 mm** at a 1.5 mm gap. **A 130.2 mm prop would hit them.**

The trial duct on this branch is sized from the measurement: bore 133.2 mm, outer diameter 149.4 mm.

If 130 mm-class props are used in flight:
1. Regenerate the flight ducts with `python src/cad/generate_cad_stl.py --prop-d 130.2`.
2. Note that the duct outer diameter grows from 146.2 to 149.4 mm. The duct-to-duct gap across the spine then shrinks from 48.8 mm to **45.6 mm**, leaving only **0.6 mm** clearance around the 45 mm spine (it was 3.8 mm).

**Measure the real 5×4.3 props when they arrive (Sync Gate A) before printing any flight ducts.**

---

## 3. What's on this branch

| File | What it is |
|---|---|
| `config/test_rig_measurements.json` | **Your caliper values.** Fill in the nulls, then regenerate |
| `src/cad/generate_test_rig.py` | One command that builds and checks every rig STL |
| `cad/test_rig.scad` | Parametric OpenSCAD rig: fit band, stand, motor arm, open-prop riser, round inlet grille, assembly view |
| `outputs/test_rig/*.stl` | Print-ready parts (currently using default motor dimensions, see §4) |
| `outputs/test_rig/assembly_preview.png` | Render of the assembled rig |
| `src/analysis/single_duct_trial_analyzer.py` | Turns the logged CSV into thrust ratios, the power penalty and FM. `--demo` runs it on synthetic data |
| `data/single_duct_trial_template.csv` | Logging template |

```
            ↑ ↑ ↑  air out
        ┌ exit grille (taped on arm) ┐
   arm ═╪══════╗                     │  ← duct, exit UP (PETG)
   post │      ║ motor hangs under pad
        │   ═══╩═══ prop (38 mm below exit face, inside the straight throat)
        └─────────────── inlet lip ──┘  ← sits on a 2 mm ledge in the stand
         round inlet grille under ledge
        │  windowed tube stand (PLA)  │  inlet 150 mm above the scale
   ═══════════ kitchen scale ═══════════  reading = thrust (thrust points DOWN)
```

---

## 4. Measurements: status and how to take them

| Field | Status | How |
|---|---|---|
| `prop_diameter_mm` | **130.2 ✓** | 2 × (shaft-hole centre to the furthest blade tip) |
| `motor_bolt_spacing_mm` | default: slotted 16–19 | Centre to centre of two **adjacent** M3 holes under the motor |
| `motor_base_to_prop_seat_mm` | default 30 | From the motor's flat underside to the face the prop hub sits on |
| `prop_hub_thickness_mm` | default 7 | Height of the prop hub |
| `motor_bell_diameter_mm` | default 34 | Outer diameter of the spinning bell |
| `motor_center_clearance_d_mm` | 10 | Increase it if the shaft or circlip sticks out further under the motor |
| `battery_cells`, `esc_cell_rating`, `flight_controller_board`, `filament_on_hand` | — | Read the labels |

The two motors have different heights, so **re-run the generator once you've chosen the motor**:
```powershell
python src/cad/generate_test_rig.py
```
Only `motor_arm.stl` depends on the motor dimensions. The fit band, duct, grilles and stand depend on the prop alone, so **you can print those now.**

---

## 5. Print list

All prints use 0.2 mm layers, 2 walls and 15 % gyroid infill.

| Order | Part | Material | Mass (est.) | Notes |
|---|---|---|---|---|
| 1 | `fit_band.stl` | PETG | 8.6 g | ~20 min. Hold the prop centred inside it: the gap should be even (~1.5 mm) with **no rubbing**. If it rubs, stop and report back; printed bores often come out 0.2–0.4 mm undersize |
| 2 | `duct_test_rig.stl` | PETG | 56.8 g | Print the exit face on the bed, as the file comes. Use a 5 mm brim. Turn on tree supports only if the slicer shows the lip overhang in red |
| 3 | `stand.stl` | PLA | 177.7 g | 195 mm tall, no supports (the window roofs are 45°). The longest print; run it overnight |
| 4 | `motor_arm.stl` | PETG | 14.6 g | **Re-generate after measuring the motor.** Already oriented upside down for printing |
| 5 | `inlet_grille.stl` | PLA | 19.3 g | Round-trimmed grille |
| 6 | `grille_test_rig.stl` | PLA | 23.8 g | Square exit grille |
| 7 | `open_riser.stl` | PLA | 7.0 g | Only used for the open-prop run |

**Ready-to-print G-code** for a P1S with a 0.4 mm nozzle and the Textured PEI plate is in `outputs/test_rig/gcode/*.gcode.3mf`:

| File | Print time | Filament |
|---|---|---|
| `fit_band` | 12 min | 8.5 g PETG |
| `duct_test_rig` | 1 h 23 min | 52.1 g PETG |
| `stand` | 4 h 44 min | 184.2 g PLA |
| `motor_arm` | 29 min | 13.7 g PETG |
| `inlet_grille` | 40 min | 17.2 g PLA |
| `grille_test_rig` | 44 min | 21.1 g PLA |
| `open_riser` | 20 min | 7.2 g PLA |

To print one, open it in Bambu Studio (File → Open), check the preview, then click **Print plate**. Or copy it to the printer's SD card and start it from the printer screen.

To regenerate them after changing the measurements, run `python src/cad/generate_test_rig.py`, then `python src/cad/slice_test_rig.py`.

**Weigh every printed part** and write the masses down. The duct and grille masses check the flight mass model (Sync Gate C).

---

## 6. Assembly

1. Drop the duct into the top of the stand, **rounded inlet lip down**, so the lip rests on the ledge. Turn it until its **exit slot lines up with the arm post**.
2. Screw the motor under the arm pad with **4 × M3×8**, shaft pointing down. The screws must not go more than 4 mm into the motor.
3. Lower the arm into the slot and onto the post. Fix it with **2 × M3×20**, which self-tap into the post.
4. Fit the prop. Check the **tip gap at 6 points** with a 1.5 mm hex key: it should be even all round.
5. Set the stand on the scale. If the scale platform is smaller than about 200 mm, put a flat board on the scale first and tare it. Route the wires so they hang slack and keep the ESC and battery off the scale.

**The three test configurations:**

| Run | Configuration | Change from the base setup | CSV `config` label |
|---|---|---|---|
| A | Open prop | Remove the duct. Fit `open_riser` between post and arm (1 × M3×16 into the post, arm screws into the riser) | `open_5in_3blade` |
| B | Duct only | Base setup | `duct_5in_gap1.5` |
| C | Duct + grilles | Push `inlet_grille` up the tube until it stops under the ledge. Tape the square grille on top of the arm | `duct_5in_gap1.5_grilles` |

---

## 7. Wiring and throttle (props OFF until step 4)

```
Battery ─XT60─ [watt meter] ─ ESC power        ESC signal + GND ─ FC output 1
ESC motor wires ─ motor                         FC ─ USB ─ laptop (Mission Planner)
```
1. In Mission Planner:
   - set Frame Type to **Quad X**;
   - set `MOT_PWM_TYPE 0`, `MOT_PWM_MIN 1000`, `MOT_PWM_MAX 2000`, then reboot;
   - run **ESC Calibration**;
   - on Pixhawk-type boards, set `BRD_SAFETY_DEFLT 0` or press the safety switch.
2. With the prop off, go to Optional Hardware → **Motor Test** and run Motor A at 10 % for 2 s. Check the spin direction, and swap any two motor wires if it's wrong.
3. Fit the prop and run 15 % for 2 s. **The scale reading must go up**, and air must blow out of the top. If not, the prop is upside down.
4. For each run, step the throttle through **20, 30, 40, 50, 60, 70 %**, 4 s each, and tare the scale before each run. Film the scale and watt meter, and read the last steady second. Only add 80 % if the motor is merely warm.

---

## 8. Logging and analysis

1. Copy `data/single_duct_trial_template.csv` to, for example, `data/trial_2026-11-xx.csv`.
2. Delete the example rows and add one row per throttle step. Leave `power_w` blank and it's computed from volts × amps.
3. Analyse it:
   ```powershell
   python src/analysis/single_duct_trial_analyzer.py --csv data/trial_2026-11-xx.csv
   ```
   This writes `outputs/test_rig/single_duct_trial_results.json` and `outputs/plots/fig10_single_duct_trial.png`.

**How to read the results:**
- **Run B vs A, thrust ratio at equal power.** This is the duct's gain or loss. Above 1.0 means the duct helps.
- **Run C vs A, power penalty at equal thrust.** This is the **measured enclosure penalty**:
  - **at or below +20 %:** the sizing's conservative case holds;
  - **above +20 %:** re-run the sizing with the measured value.
- **Open-prop FM.** The design assumes 0.60.

Without a watt meter, the analyzer falls back to comparing thrust **at equal throttle**. That's weaker, because a duct changes how hard the motor is loaded.

Running `--demo` exercises the whole chain on **SYNTHETIC** data, labelled in red on the figure.

---

## 9. Safety

- **Props off** for all wiring, calibration and setup work.
- Never stand in line with the prop disc. Wear glasses, and keep your hand near the battery plug.
- Tape the stand to the scale and the scale to the table.
- Make sure the prop nut is tight.
- Charge the LiPo in a bag. Stop at about 3.7 V per cell resting (11.1 V for 3S, 14.8 V for 4S).

---

## 10. Session notes (3 Nov 2026), for carrying context into claude.ai

- **Using 935 KV 4S motors instead of the 2207 1700 KV was rejected.** At about 14.3 V under load they top out around 12,000 RPM:
  - **6" prop:** thrust-to-weight about 0.6, so it can't take off;
  - **8045 prop:** thrust-to-weight about 1.0, and an 8" box would weigh about 3 kg.
  They're only useful for an open-frame trainer quad. The plan's 4S fallback remains the 5" 2400 KV contingency (T/W 2.53).
- **Materials:**
  - ducts are PETG at 1.27 g/cm³;
  - grilles, quarters, spine and legs are PLA at 1.24 g/cm³;
  - skids are TPU at 1.21;
  - hinges are PA-CF at 1.20.
  The mass model treats a 0.84 mm skin as solid and the rest of the volume as 15 % infill, so walls of 1.7 mm or less print solid.
- **Weight (legs build): 1,904.9 g**, which includes +15 % on every estimated item:
  - structure 957.9 g;
  - propulsion 596.2 g;
  - payload 180 g;
  - gear 107.3 g;
  - avionics 63.5 g.

  Other builds:
  - skids: 1,825.2 g;
  - CAD-estimated print masses: 1,959 g;
  - Stage 1 bare: 1,131 g.

  57 % of the total is still estimated.
- **Software installed:**
  - Arduino IDE 2.3.10;
  - OpenSCAD 2021.01;
  - Bambu Studio (winget ID `Bambulab.Bambustudio`, with a lowercase s).

  Mission Planner and Python packages are checked separately.
- **Box Lab v3** is published privately at https://claude.ai/artifact/RnvLxwDuFVyszcbNrzuaNC. Its wind-lean (15.4°) and battery-reserve figures are rougher than the 6-DOF simulator's (19° lean, 58 % battery left).
