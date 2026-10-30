# Project 8: Enclosed Ducted-Fan Quadrotor. Full Context

Course: Introduction to UAV Design (EC4.402), IIIT Hyderabad, Monsoon 2026. Course announcements are by Harikumar Kandath.
Team: six people, split into three pairs. Shriya's pair partner is Kavya.
Status of this file: 25 Oct 2026, ~22:50 IST. Everything in this file is current as of that time. Where something changed, the history is noted.

---

## 1. Key dates

| Date | Event |
|---|---|
| Oct 18 | Shriya and Kavya pair session. The spec doc was created, and the team decision meeting was held in the evening. |
| Oct 19 | TA review meeting. Its guidance is in section 4. |
| Oct 25 | Sizing loop built. The design changed from 5" to 7" props after the TA's feedback. A new spec doc was made. |
| **Oct 27** | **Project presentation, worth 10 marks. No change of deliverables is allowed after it.** |
| Mid-Nov | Target date to have the enclosure printed. |
| 1st week of Dec | Evaluation. Exact dates are still to be announced. |

### Oct 27 presentation requirements (course announcement, 9 Oct 2026)

The presentation must contain:

1. Objectives
2. Solution framework / architecture
3. List of deliverables
4. List of components used

It is worth 10 marks. **No change of deliverables will be allowed after the presentation.**

TA's guidance for the presentation: no CAD is needed yet. Aim for about 75% of the conceptual analysis.

### Proposed slide outline

1. Objectives
2. Mechanical architecture: carbon X frame, 4 bellmouth ducts, a 4-part enclosure, payload bay, retractable legs
3. Electrical architecture block diagram: battery → XT90 → ESC → motors. The FC connects to GPS/compass, RC receiver, telemetry radio, ESC current sensor, landing-gear servos, and camera power.
4. Conceptual design round 1 (2.5 kg worst case)
5. Rounds 2 and 3 (real weight)
6. Comparisons: square vs rectangle, 2200/2600/3300 packs, 2- vs 3-blade, payload, 5"/6"/7"
7. Duct design
8. Deliverables
9. Components
10. Plan and schedule

### Deliverables advice

Commit only to minimums the team is confident of, and call the rest "targets".

**Commit (graded):**
- Enclosed ducted quad with no exposed rotating parts (finger-safe grilles)
- Autonomous outdoor GPS waypoint mission: ~5–8 waypoints, ~3 min, landing with ≥25% battery
- Payload compartment that opens and closes (180 g) and is carried during the mission
- Onboard camera recording the mission
- Retractable landing gear
- Modular build: carbon frame plus a quick-attach 4-part enclosure
- Report with the sizing analysis and its iteration loop, option comparisons, thrust-stand predicted vs measured, and CAD

**Targets (not commitments):**
- Thrust-to-weight 2.5–3
- 5–6 min hover

The whole team must agree on the deliverables list before Oct 27.

---

## 2. Course brief (Project 8)

- Four ducted-fan propulsion units inside a complete external enclosure.
- No rotating propulsion part exposed or sticking out.
- Designed intake and exhaust openings that still allow stable flight.
- The aircraft must fly stably with the enclosure fitted, then fly predefined position-based waypoints autonomously with the enclosure still in place.
- Graded deliverables: duct and enclosure design, propulsion sizing and selection, CAD model and fabricated enclosure, flying prototype, flight-controller integration, waypoint demo.
- Common deliverables for every team: working prototype, flight logs plus the data behind every result, and a technical report with a predicted-vs-measured comparison.
- Lab rules:
  - Design and BOM must exist before any purchase or print.
  - The TA approves print settings.
  - At most 3 team members may be in the lab at once.
  - Printer 2 (Bambu P1S, ~256 mm bed) is shared by 9 teams.
- Budget is set aside for now. The TA checks inventory and orders what is missing.

## 3. Professor's box constraints (stated verbally)

- The outer shape is a box (cuboid): no protruding arms, pods or duct rims.
- There must be internal space for a payload, and with it fitted the aircraft still looks like a box.
- Nothing hangs below: retractable legs or a takeoff platform.
- A camera was suggested to make the project more complete.
- No size limit was given.

## 4. TA guidance from the Oct 19 review (all points)

These come from an automatic transcription, so some numbers may be slightly off.

**Order of work: conceptual design first, before anything else.**
- Assume a worst-case all-up weight (he said ~1.5–2 kg). The transcription says "your estimate came out around 900 g", which conflicts with our 1.65–1.77 kg at the time. This may be a transcription error and should be checked with the TA.
- From that weight and the lift equation, choose the propellers, then the motors (set by prop speed), then the ESCs. Use a standard flight controller. Size the battery last.
- A 5–6 min flight time is fine.
- We only need normal hovering, not FPV-style agile flying, so we don't need high current draw.

**Iterate.** After the first full design, find the real weight (for example 1.6 kg instead of the assumed 2 kg), go back and adjust the sizing. Show this loop in the report.

**CAD.** Download the vendor STL files for the chosen motors and props (e.g. from T-Motor's site). Assemble everything in Fusion 360, then design the ducts and enclosure and check that everything fits.

**Hardware.** Send the TA the component specs. He checks lab inventory or orders what is missing, and delivery takes about a week. Then print and assemble. Leave at least a week before evaluation for this.

**Ducts before enclosure.**
- Buy normal propellers and motors and 3D-print our own duct around them. Don't buy ready-made ducted-fan units.
- A bell-mouth inlet is fine.
- The enclosure wraps around the ducts, so the ducts' size sets the enclosure's size. Order within each step: props/motors → ducts → enclosure.

**Modular build.** Build a basic quad frame (e.g. carbon-fibre arms), then print an enclosure that attaches to it with a quick-attach mechanism. He suggested printing a square enclosure as 4 parts and joining them.

**Grading.** The marks come from the analysis, not the controller or code. The final design goes in the report body and every derivation in an appendix. Include comparisons like "with option X we'd get this flight time, so we chose Y".

**Payload.** A compartment you can open, load and close, which still looks like a box afterwards. A payload of about 10% of the drone's weight is reasonable. The square design with two spines is fine ("as long as it works").

**Landing gear.** Retractable legs are easy: 2 small SG90-type servos with 3D-printed legs. ArduPilot/PX4 support retractable gear out of the box.

**Remove ELRS** from the component list and use the lab's RC and telemetry equipment.

**Material.** Any filament works for the enclosure. Normal PLA is fine, since the drone is only outdoors for ~5 min at the demo. Lightweight PLA is only needed if weight gets tight. Printing with more hollow infill also saves weight.

**Size.**
- At the meeting: he thinks a smaller drone might impress the professor, but that is his opinion. The professor gave no size limit.
- Later (relayed by Shriya on Oct 25): **the TA said 5" props are too small.** The team moved to **7" props** as a result.

**Oct 27 presentation.** No CAD needed. Aim for ~75% of the conceptual analysis.

---

## 5. Team decisions (current)

| Decision | Choice | Reason / history |
|---|---|---|
| Waypoint demo | Outdoors only | GPS can be used; indoor positioning (optical flow) is much harder. Testing happens indoors and outdoors; the graded mission is outdoors. |
| Build order | **Stage 1:** the bare quad (open props) flies first with a good thrust-to-weight. **Stage 2:** ducts and enclosure are added, then thrust is rebalanced. | Team decision, Oct 25 |
| Prop size | **7"** (177.8 mm) | Was 5". Changed Oct 25 because the TA judged 5" too small. |
| Thrust-to-weight | 2.5 design value, up to 3.0 (with the enclosure) | Enclosure and grilles cost thrust, and outdoor wind needs headroom |
| Flight time target | 5–6 min hover (a target) | No course requirement |
| Payload | 180 g dummy mass in a 60 × 40 × 30 mm bay with a hinged, latched door, flush when closed | Was 150 g, then 180 g after the TA's "~10%". At 7" it is ~7% of all-up weight; 270 g would be 10% and cost ~0.3 min. |
| Camera | Lightweight recording camera (~10 g, RunCam Thumb class), front-facing, behind a window | Records the mission; not wired to the FC for data |
| Layout | **Square** box with a cross-shaped spine | The TA said fine. Same roll and pitch behaviour, ArduPilot QUAD/X at 45°, and the cross spine holds the leg hinges. |
| Lip shape | Bellmouth | The TA said fine. A round nose grows the box. |
| Intake | Open-top grilles directly over each duct | Best airflow |
| Grilles | Square grid, 1 mm bars at 12 mm pitch, 4 mm deep, ~16% blockage | Finger-safe (11 mm gaps) |
| Landing gear | 2 retractable printed legs on SG90 servos | Replaced the earlier "flush skids + takeoff cradle" plan |
| Structure | Carbon-fibre X frame plus a printed enclosure in 4 quick-attach quarters | TA guidance |
| Material | PLA (PETG is the fallback) | TA guidance |
| RC + telemetry | Lab RC set plus lab telemetry radio (433 MHz or 865–867 MHz; 915 MHz is not permitted in India) | ELRS was removed per the TA |
| Positioning | GPS + compass with SBAS (~1 m) | Plain GPS (~3 m) fails an assumed ±2 m waypoint tolerance |
| Flight stack | ArduPilot (ArduCopter) + Mission Planner | Waypoint missions and a built-in simulator |
| Battery | 6S **3300 mAh** recommended, ≥45C, XT90 (2200/2600 also worked through) | Lab stock decides. Only the 3300 gives >5 min with the enclosure. |
| Payload interface | Four M3 heat-set inserts with locating features | Our choice |
| Validation | Lab thrust stand plus simulation | |

---

## 6. Current design (7"), with all numbers

All figures use air density 1.09 kg/m³ (540 m elevation, 30 °C, p ≈ 95 kPa), FM 0.6, motor+ESC efficiency 0.8, 80% of the pack usable, ~10 W avionics, 6S at 22.2 V nominal, and 15% contingency on estimated parts. Numbers are for the square layout unless stated.

### Duct geometry

| Parameter | Value |
|---|---|
| Prop | 177.8 mm (7") |
| Tip clearance | 1.0 / 1.5 / 2.0 mm candidates, 1.5 mm baseline (1.7% of blade radius), chosen by thrust-stand test |
| Bore | 180.8 mm (179.8 / 180.8 / 181.8 across the candidates) |
| Inlet lip | 9.0 mm radius bellmouth (5% of bore) |
| Length | 70 mm (L/D ≈ 0.39); rotor 21 mm below the inlet (30%) |
| Diffuser | 49 mm run, ~4.5° half-angle, exit 188.5 mm (bore + 7.7), area ratio ≈ 1.09 |
| Wall | 1.6 mm PLA (1.2 mm with ribs saves ~25 g per ring) |
| Outer diameter | D_o = bore + 2(lip + wall) = 180.8 + 2(9.0 + 1.6) = **~202 mm** |
| Round-nose alternative | D_o = 216.8 mm, box ~500 mm. Rejected. |
| Exhaust clearance | 1.5 × D_o ≈ **300 mm**; legs ideally ~300 mm, at least ~200 mm |
| Motor mounting | The motor sits on the carbon arm, which enters the ring through a slot. The arm blocks ~5% of the exit. |

**Duct shape discussion.** A teammate was sent a "bellmouth + close-fitting shroud + short diffuser" (hourglass) suggestion. That is already our design. One idea worth taking from it is an **elliptical inlet**: the same radial width as the lip, but deeper (for 5" this was ~10–12 mm; it must stay shallower than the rotor depth). It is an optional extra test ring. Copying that figure's large inlet proportions would grow the outer diameter.

### Box, frame and enclosure

| Item | Value |
|---|---|
| Spine | 55 mm, cross-shaped. Widened from 45 mm so a 6S 3300 pack (~140 × 45 × 40 mm) fits. |
| Square box | W = 2·D_o + spine + 2·6 = **~471 × 471 × ~100 mm** |
| Rectangle (comparison only) | ~471 × 422 mm |
| Motor arms | 128.5 mm on both axes (square); rectangle 128.5 / 104 mm |
| Frame | X frame, **~363 mm** between diagonal motors (rectangle ~331 mm). This is bigger than a standard 7" frame, so buy a long-arm frame or cut arms from carbon plate. |
| Enclosure | 4 quarters of ~236 × 236 mm (each fits the ~256 mm printer bed), joined along the spine centrelines with tongue-and-groove edges and M3 heat-set inserts. Attached to the frame with 4–8 quick-release clips or quarter-turn fasteners. Duct rings bolt to the arm ends around the motors. |
| Intake/exhaust | Disc area 24,830 mm²; cell 202 × 202 = 40,800 mm². Grid grille gives ~34,300 mm² (1.38 × disc), intake rule ≥1.2×. Exhaust ≤20% blockage gives ~32,600 (1.31×). A round opening at the exit diameter would give only ~0.94× disc area. |
| Airflow (3300) | Hover v ≈ 11.1 m/s, Q ≈ 0.28 m³/s per fan. At TWR 3: v ≈ 19.2 m/s, Q ≈ 0.48 m³/s (~1,700 m³/h per fan). |
| Landing gear | Legs hinge at the left and right ends of the cross spine (outside the exhaust cones). SG90 at ~1.8 kg·cm; a 40 g leg at 15 cm needs ~0.6 kg·cm. A printed hard stop takes the landing loads. ArduPilot SERVOn_FUNCTION = 29 (landing gear), with deploy below ~3 m and retract above ~5 m. Legs mount to the frame, so stage 1 flies with them too. |
| Cooling | Bleed vents from the duct cells into the spine. Log ESC and battery temperature. The ESC carries ~25–28 A at hover in stage 2. |
| CG | Within 5 mm of centre in both axes, with and without payload, and with legs up and down |
| Inertia (square) | 4 duct units × ~156 g, I ≈ 0.622 × 0.1285² = 10.3 × 10⁻³ kg·m² |

### Mass budget (stage 2, with enclosure)

Estimated parts (15% contingency applied):

| Item | Square (g) | Rectangle (g) |
|---|---|---|
| Duct rings ×4 | 400 | 400 |
| Side panels 0.8 mm | 187 | 177 |
| Grilles ×8 (4 mm deep) | 260 | 260 |
| Decks, ribs, quick-attach joints | 210 | 148 |
| Carbon X frame | 135 | 120 |
| Spine/bay walls | 45 | 35 |
| Payload door/hinge/latch | 30 | 30 |
| Retractable legs + servo mounts | 95 | 95 |
| Wiring/XT90/fasteners | 90 | 90 |
| **Subtotal** | **1,452** | **1,355** |
| **×1.15** | **1,670** | **1,558** |

Bought parts (datasheet, no contingency), 512 g total:

| Part | Mass |
|---|---|
| Motors ECO II 2807 1300KV | 4 × 47.6 = 190 g |
| Props Gemfan 7040-3 | 4 × 7.9 = 32 g |
| ESC | 23.5 g |
| FC | 10.5 g |
| GPS + mast | 18 g |
| RC receiver + telemetry allowance | 30 g |
| Camera | 9.8 g |
| SG90 servos | 2 × 9 = 18 g |
| Power module | 0 |
| Payload | 180 g |

Batteries: 2200 = 340 g, 2600 = 400 g, 3300 ≈ 520 g.

| Layout | 2200 | 2600 | 3300 |
|---|---|---|---|
| Square all-up weight | 2,522 g | 2,582 g | **2,702 g** |
| Rectangle all-up weight | 2,410 g | 2,470 g | 2,590 g |

Masses use PETG densities. PLA is ~2% lighter (~25 g), which is kept as margin.

**Levers if the weight or thrust doesn't close:**
1. Lighter enclosure: 1.2 mm rings (−100 g), 3 mm grilles (−65 g), lightweight PLA panels (−75 g)
2. Keep the 3300 pack
3. Accept TWR 2.5
4. Switch to the rectangle (−110 g)
5. Lighten the payload

### Sizing loop (TA's method)

| Item | Round 1 (2.5 kg assumed) | Round 2 (real, 2600) | Round 3 (real, 3300) |
|---|---|---|---|
| All-up weight | 2,500 g | 2,582 g | 2,702 g |
| Hover thrust per motor | 625 gf | 645 gf | 675 gf |
| Max thrust per motor (TWR 2.5–3) | 1,563–1,875 gf | 1,614–1,936 gf | 1,688–2,026 gf |
| Hover current | 24.9 A | 26.2 A | 28.0 A |
| Hover time | 5.0 min | 4.8 min (below target) | **5.7 min** |
| KV estimate (3-blade) | 1,070–1,170 | 1,090–1,190 | 1,110–1,220 |
| Motor / ESC | 2807 ~1300 KV / ≥55 A | same | same |
| Battery | 2600 (lightest that gives 5 min at 2.5 kg) | too small | 3300 |

**Round 1 details:**
- Hover power: P_ideal = 6.13^1.5 / 0.233 = 65.2 W; P_elec = 136 W per motor; total 24.9 A.
- Hover RPM ~12,000; RPM at max thrust 19,000–20,800.
- Max current 24–32 A per motor.
- Pack capacity for 5 min: 2.6 Ah. Hover times: 2200 → 4.2 min, 2600 → 5.0 min, 3300 → 6.4 min.

**Round 4** would repeat round 3 (no part changes). The weighed real parts become the next round in the report.

### Stage 1 vs stage 2

| | Stage 1: bare quad, open props | Stage 2: ducts + enclosure |
|---|---|---|
| All-up weight (3300) | ~1.19 kg without payload, ~1.37 kg with | ~2.70 kg |
| Open-prop TWR (4 × ~2.35 kgf) | ~6.9–7.9 | ~3.5, before duct gain and grille loss |
| Hover as a share of max thrust | ~13–15% | ~29% |
| Hover current / time (3300) | ~8–10 A / ~15–19 min | ~28 A / ~5.7 min |

Stage 1 will feel twitchy. Start with low gains and fly it with the payload or ballast. After stage 2 the inertia changes, so run autotune again. The difference in hover current between the stages is the measured cost of the enclosure.

### Battery details

| | 2200 | 2600 | 3300 |
|---|---|---|---|
| Square all-up weight | 2,522 g | 2,582 g | 2,702 g |
| Hover current | 25.3 A | 26.2 A | 28.0 A |
| Hover time | 4.2 min | 4.8 min | 5.7 min |
| Battery left after the ~3.2 min mission | ~30% | ~38% | ~48% |
| Mission peak / C needed | ~98 A / 45C | ~102 A / 39C | ~109 A / 33C |

- Requirement: **≥45C**, **XT90** with 12 AWG leads (the peak exceeds an XT60's ~90 A burst rating).
- Sensitivities: with no contingency the square times are 4.8 / 5.4 / 6.4 min; with no payload they are 4.7 / 5.3 / 6.3 min.
- Mission reserve formula: left = 1 − 1.15 × t_mission / (t_hover / 0.8).

### Motors and props

- **Emax ECO II 2807 1300KV** (6S): 47.6 g, M5 prop shaft. The bolt pattern is listed as **19 × 19 mm** (an earlier listing said 16 × 16), so confirm before buying the frame. Spec max thrust is 2,640 g on 6S (prop not stated), about 2.35 kgf at our air density (× 1.09/1.225 = 0.89). That is above the 2.03 kgf needed.
- KV formula: n = √(T / (C_T ρ D⁴)), KV = RPM / (0.8 × 22.2), with C_T 0.14 for 3-blade and 0.10 for 2-blade. Estimates are 3-blade ~1,050–1,220 KV and 2-blade ~1,240–1,440 KV. The 1300 KV motor suits the 3-blade; a 2-blade would push it near its limit.
- **Props:** Gemfan Flash 7040-3 (7" tri-blade, 7.9 g, 5 mm hub, ₹235 for a set of 4 at ThingBits, sold out when checked). Also a 7" 2-blade set for comparison. Measure the real diameter before printing the ducts.
- 3-blade vs 2-blade (3300): hover RPM ~12,500 vs ~14,800; tip speed at max ~184 vs ~218 m/s; hover 5.7 min vs ~6.1 min if the 2-blade's FM is 0.05 higher (not measured).
- ESC: 4-in-1, ≥55 A per channel (≥1.3 × a ~40 A full-throttle draw, still to be measured). In flight each motor draws ~27–36 A at TWR 2.5–3 and ~7 A at hover.

### Comparisons (for the report)

- **Square vs rectangle (3300):** 2,702 g / 5.7 min vs 2,590 g / 6.0 min. Arms 128.5/128.5 vs 128.5/104 mm; inertia equal vs 10.3/6.7 × 10⁻³ kg·m². **Square chosen:** +~110 g and −0.3 min buy identical roll and pitch, ArduPilot's default mixer, and the cross spine for the legs.
- **Payload 0 vs 180 g (square):** 2200 4.2 → 4.7 min; 2600 4.8 → 5.3; 3300 5.7 → 6.3. Each 100 g costs ~0.3–0.35 min.
- **Prop size (square):**

| | 5" | 6" (scaled, indicative) | 7" |
|---|---|---|---|
| D_o | 146 mm | ~174 mm | 202 mm |
| Box side | 348 mm | ~415 mm | 471 mm |
| All-up weight (2600) | 1.87 kg | ~2.0–2.2 kg | 2.58 kg |
| Hover time (2600) | 5.5 min | ~5.3–5.9 min | 4.8 min (5.7 on 3300) |
| Prop loading | ~360 N/m² | ~270–290 N/m² | ~255 N/m² |
| RPM at max | ~32,000 | ~23,500 | ~19,300 |
| Tip speed | ~214 m/s | ~190 m/s | ~184 m/s |

- **Why 7":** the TA's call. Lower prop loading, ~40% slower at full thrust, more motor margin, a strong stage 1 quad, and mature 7" parts. **Cost:** +~0.7 kg, a box ~12 cm wider each way, and a 3300 pack needed.
- **Caveat:** the earlier conclusion that 5" was "fine" depended on assuming enclosure mass scales with box area. If it scales only with side length, bigger props win. This sensitivity was shown honestly.

### Avionics and components

| Component | Choice |
|---|---|
| FC | SpeedyBee F405 V4 (ArduPilot target speedybeef4v4; F405, DPS310 baro, SD slot, 6S voltage sensing; UART4 is used by Bluetooth; bidirectional DShot on outputs 1–4). Alternative: Matek H743-SLIM V3 (7 g, 7 UARTs, no current sensor). Pixhawk 6C if the lab has one. Avoid Pixhawk 2.4.8 clones. |
| ESC | SpeedyBee BLS 55A 4-in-1 (23.5 g, 55 A continuous / 70 A burst, BLHeli_S, current sensor at scale 400). Sold as a stack with the FC, ~₹6,700 at ThingBits (sold out when checked). Flash Bluejay for RPM telemetry and the harmonic notch. Alternative: T-Motor F55A Pro II (17.5 g). |
| GPS | Matek M10Q-5883 (u-blox M10 + QMC5883L compass, 8 g, SBAS on by default). Mounted on the top face, away from power leads, never under carbon. |
| RC / telemetry | Lab equipment, ~30 g allowance |
| Power monitoring | ESC current sensor + FC voltage sensing; calibrate BATT_AMP_PERVLT. No separate power module. |
| Camera | RunCam Thumb (1080p60, 9.8 g, 5 V). Alternative: Thumb 2 (4K, 27.8 g). |
| Gear servos | 2 × SG90 (~9 g each; MG90S metal-gear as an alternative) |
| Other | Vibration grommets, buzzer + safety switch, 6S balance charger, LiPo bag, battery checker, ≥60 W soldering station, XT90 pigtails, 12 AWG wire, ≥35 V ~1000 µF low-ESR capacitor, M3 heat-set inserts (30–40), M3/M2 bolts, quick-release fasteners, leg hinge pins, battery straps, 2–3 kg PLA, servo leads |

**Key ArduPilot settings:**

| Parameter | Value |
|---|---|
| MOT_THST_HOVER | Learned; expect ~0.3–0.4 in stage 2 and ~0.13–0.15 in stage 1 |
| WPNAV_SPEED | 300–500 cm/s |
| WPNAV_ACCEL | 100–150 |
| INS_HNTCH_ENABLE | 1 (RPM-based with Bluejay, else throttle-based) |
| FS_THR_ENABLE | 1 (RTL) |
| BATT_LOW_VOLT / BATT_CRT_VOLT | 21.0 / 19.8 V |
| SERVOn_FUNCTION | 29 (landing gear) |
| LAND_SPEED | 40–50 cm/s |
| GPS_HDOP_GOOD | 140 |

Learn ArduPilot in simulation or on a borrowed quad early.

---

## 7. Box Lab simulator (run on the old 5" design)

**Kavya's Box Lab v2 page:** https://claude.ai/artifact/7BSdNecmT8HgTxm9tuzGxw

**Box Lab v1 result (5" design):** 11 of 14 tests passed; the remaining three warnings didn't apply to our layout.

What it confirmed:
- Duct geometry (129/146 mm vs our 129.5/145.7 mm)
- Box size (~34–35 cm)
- Duct mass (~200 g)
- 5" as the size where flight time peaks in the lab's prop sweep
- Wind lean of 16° at 4 m/s, with ~92% of the side push coming from the ducts swallowing sideways air (another reason to fly slowly)

What changed because of it:
1. GPS with SBAS is required.
2. Telemetry over ELRS. This is now reversed by the TA, so the 30 g radio allowance stays and CAD has to find the 11 mm of stack height.
3. Stack height is tight; a 100–105 mm box is the fallback.
4. Corner pockets are too small for boards (31 mm); use them for wiring.

Warnings: grille solidity under ~20%, the landing setting, and frame size.

**Issues found in Kavya's v2:**
- The XT90 check always flags red at 160 A.
- A 0.75 mm gap is marked bad.
- The cradle warning (needs 219 mm) was valid at the time.
- The title still said "(Copy)".
- One message refers to the "first draft".

**Expected flags with the 5" settings:** connector red, yaw yellow, rectangle roll/pitch yellow. A 3.2-min mission (8 waypoints, 40 m apart, 3 m/s, 5 s pauses) landed with 44–53% left.

**To do:** rerun the Box Lab at 7".

---

## 8. Work split: three pairs (proposed; final split to be decided after the TA review)

**Pair A: propulsion and electronics**
- eCalc runs: 2 motors × 2- and 3-blade 7" props, at 2.5 kg then 2,520 / 2,580 / 2,700 g, plus the ~1.3 kg stage 1 quad
- Pick exact models and finalise BOM v2 for the TA
- Send vendor STLs and dimensions to Pair B
- Keep the mass budget current
- Plan the thrust-stand tests (5 configurations plus the 1.0/1.5/2.0 mm gap sweep; log thrust, current, voltage, RPM and ESC temperature)

**Pair B: CAD and enclosure**
- Parametric duct ring
- Single-duct test unit with grid grilles and 3 rings
- Motor/prop STLs on the ~363 mm X frame in Fusion 360, then the 4-part square enclosure with battery, payload door, FC, ESC, GPS, camera, vents and legs
- ~300 mm exhaust cones and clearance checks
- Mass and CG check
- Drawings and a PLA print plan

**Pair C: flight software and simulation**
- Mission Planner SITL waypoint loop, then own the stage 1 bring-up
- Draft the demo mission (~3 min)
- Parameter and failsafe plan, including the landing-gear outputs
- FC wiring diagram (GPS, telemetry, RC, ESC current sensor, 2 gear servos)
- Find the outdoor site, permissions, and a test quad to borrow
- Presentation slide skeleton

**Handoffs:**
- A → B: STLs and dimensions
- A → C: FC, GPS and radio choices
- B → A: CAD and slicer masses; printed test duct
- C → B: GPS and FC placement needs
- A → all: updated all-up weight

## 9. Steps before the TA CAD review

1. Close the remaining decisions and assign the pairs.
2. Run eCalc xcopterCalc: weight 2500 g, then the real weights; elevation 540 m, 30 °C; 6S 3300/2600; controller ≥55 A; motor 2807 ~1300 KV; prop 7×4 3-blade. Pass criteria: TWR ≥2.5, hover throttle ≤50%, no overheating.
3. Finalise BOM v2 and send it to the TA.
4. Parametric duct in Fusion 360 with parameters prop_d 177.8, tip_clear 1.5, lip_r 9, duct_len 70, rotor_pos 21, exit_d 188.5, wall 1.6. Revolve the profile, add the arm slot plus bolt bosses, and test changing tip_clear to 2.0.
5. Single-duct test unit with grid grilles (print this first).
6. Box envelope: vendor STLs on the ~363 mm X frame (128.5 mm arms), duct rings, 55 mm cross spine, ~6 mm walls as 4 quarters. Add blocks for the electronics, servos and payload bay; legs up and down; ~300 mm reference cylinders below each duct (nothing may intersect them); GPS on top.
7. Mass/CG check: PLA at 1.24 g/cm³, datasheet masses for bought parts, slicer grams for printed parts; CG within 5 mm.
8. Mission Planner SITL loop (TAKEOFF → 4–5 waypoints → RTL; WPNAV_SPEED 400).
9. TA review pack: drawings, BOM, mass budget, eCalc results, print plan, question list.

## 10. Bring-up and flight testing

**Gates:**
1. Bench setup with props off: ArduCopter QUAD/X, calibrations, radio, modes, battery monitor, gear switch, failsafes
2. Motor order and direction
3. Thrust stand (runs in parallel)
4. Vibration check (<15 m/s², no clipping)
5. First hover in STABILIZE on the legs, ~50 cm
6. Trim and PID
7. ALT_HOLD (±0.5 m)
8. AUTOTUNE
9. LOITER (2 m circle for 60 s)
10. Simple AUTO (2 waypoints; legs retract and deploy)
11. Full mission rehearsal with payload, 3 times on 3 different days, landing with ≥25% battery

**Two passes:**
- **Stage 1:** gates 1, 2 and 4–10 on the bare quad.
- **Stage 2:** gates 4–10 again with the enclosure (fresh autotune), then gate 11.

**Rule:** no AUTO until LOITER is solid.

**Yaw:** yaw comes only from differences in motor reaction torque, so it is the weakest axis in a ducted box. Log the CW vs CCW current difference. If yaw is weak, reduce the yaw rate and acceleration limits.

**Safety:**
- The pilot always holds the transmitter, with a manual override or kill switch available.
- 20 m margin around the mission footprint.
- Props off for bench work.
- Failsafes tested before the first flight.
- LiPo rules: charge in a bag, never unattended.
- Check institute and DGCA requirements before flying.

## 11. Predicted vs measured (report backbone)

| Quantity | Predicted |
|---|---|
| All-up weight, stage 1 | ~1.19/1.37 kg |
| All-up weight, stage 2 | 2.52/2.58/2.70 kg |
| Enclosure + frame before contingency | ~837 g |
| CG | ≤5 mm |
| Open-prop max thrust | ~2.64 spec / ~2.35 kgf in our air |
| Duct gain | 0 to +26% |
| Grille loss | a few % |
| Tip clearance | best at 1.0 mm |
| Max thrust with full enclosure | must reach 1.69–2.03 kgf |
| Hover throttle | ~0.13–0.15 (stage 1) / ~0.3 (stage 2) |
| Hover current | ~8–10 A (stage 1) / ~28 A (stage 2) |
| Hover time, stage 2 | 4.2/4.8/5.7 min |
| ESC temperature | within rating |
| Yaw proxy | baseline |
| Vibration | <15 m/s² |
| Battery left after mission | ~30/38/48% |
| Waypoint error | ≤2 m |

## 12. Schedule (Oct 19 – Dec 5)

| Task | Dates |
|---|---|
| TA review | Oct 19 (done) |
| Sizing loop / spec / BOM | Oct 19–26 |
| Order long-lead parts | Oct 21–28 |
| **Presentation** | **Oct 27** |
| Test duct + thrust sweep | Oct 26 – Nov 4 |
| Parts arrive | Oct 28 – Nov 14 |
| Enclosure CAD + prints | Nov 2–14 |
| Assembly + bench bring-up | Nov 12–19 |
| First hover + autotune | Nov 18–24 |
| LOITER / AUTO / payload | Nov 23–29 |
| Rehearsals | Nov 26–30 |
| Evaluation | Dec 1–5 |

**Risks:**
- Part lead times: overseas orders take 2–4 weeks, so prefer Indian suppliers and ask the TA about lab stock first.
- One shared printer: book slots early.
- Weather: schedule spare flying days.
- Weight growth: weigh every part and update the budget weekly.

## 13. Open items and questions for the TA

**Still open:**
- Battery size (depends on lab stock; 3300 recommended)
- Prop blade count (thrust stand decides)
- Frame: buy a ~363 mm long-arm frame, or cut arms in the lab?
- Final pair assignments
- Deliverables list for Oct 27 (locked afterwards)

**Questions for the TA:**
- Which BOM parts are in lab inventory (FC, GPS, RC, telemetry, motors, 6S packs)?
- Which 6S packs are in stock, and their dimensions?
- Thrust stand: maximum load, and does it log current and RPM?
- Outdoor flying site and permission (institute/DGCA)?
- Can carbon plate be cut in the lab?
- Is there a test quad to borrow?
- What waypoint tolerance is used at evaluation?
- Clarify the "~900 g" estimate in the transcript, and confirm the 7" size and the ~2.7 kg weight are acceptable.

## 14. Corrections and history (so nothing is re-introduced)

- **Hover power:** first estimated at 60–65 W per fan, corrected to ~100 W. The battery went from 1800 to 2200 mAh as a result.
- **Connector:** XT60 → XT90. The full-throttle peak was mistaken for a single motor's current.
- **Contingency:** applied only to estimated parts, not datasheet parts.
- **Air density:** corrected from 1.16 to 1.09 kg/m³. Effect ~3% on flight time.
- **Cradle/clearance rule:** 1.5 × the *outer* diameter, not the exit diameter.
- **Tip gap candidates:** changed 0.75/1.25/2.0 → 1.0/1.25/2.0 mm (5"), now 1.0/1.5/2.0 mm at 7".
- **Diagrams:** the duct diagram was redrawn as a bellmouth (it had a round nose); square-vs-rect title changed 45 → 60 g (5"), now ~110 g at 7".
- **Density derating:** an earlier ~6% thrust derating was wrong; the correct figure is ~11% (1.09/1.225). The 5" motor figure was corrected from 1.86 to 1.76 kgf.
- **Ground contact:** went from skids + ~150 mm cradle → skids + ~220 mm cradle → (TA) retractable SG90 legs.
- **Radios:** went from SiK → ELRS (Box Lab suggestion) → (TA) lab radios, no ELRS.
- **Material:** PETG → (TA) PLA.
- **Payload:** 150 g → 180 g.
- **Prop size:** 5" → 7" (TA). Weight went from ~1.75–1.87 kg to ~2.52–2.70 kg; the spine was widened 45 → 55 mm; the recommended battery is now 3300.
- **Rotor mounting:** printed spokes → motor on the carbon arm.
- **Single-duct test unit:** radial-spoke grilles were replaced with grid grilles, because spokes fail the finger test.

## 15. Old 5" design (record, superseded)

- 127 mm props, 1.25 mm gap, 129.5 mm bore, 6.5 mm lip, 50 mm length, 135 mm exit, 145.7 mm outer diameter, 45 mm spine, ~350 × 350 mm box.
- Emax ECO II 2207 1700KV motors (33.4 g, 1.98 kgf), 5" props.
- After the TA's changes: all-up weight 1.75/1.81 kg (rectangle) and 1.81/1.87 kg (square) on 2200/2600; hover 4.9–5.8 min.
- Round 1 at 2.0 kg gave 25.0 A; 2600 → 5.0 min.

## 16. Files and links

| Item | Location |
|---|---|
| **Current spec doc** ("Project 8 — Design Spec", 7", stage 1/2, appendix of derivations) | https://claude.ai/artifact/VyuTp6PUfYPSJ3vahqvZU6 |
| Old spec doc ("Design Spec & Pre-TA Plan", 5"; keep or delete, not yet decided) | https://claude.ai/artifact/83FxNoSgHpE9d9ALx3sAay |
| BOM v1 Claude Doc (5", still lists ELRS; outdated) | https://claude.ai/code/artifact/2206d5fd-00a6-4520-9e2d-892ddb83cfb2 |
| Kavya's Box Lab v2 | https://claude.ai/artifact/7BSdNecmT8HgTxm9tuzGxw |
| Current component spreadsheet | `Project 8 - Component List (BOM v2, 7in).xlsx` (Components tab + Mass check tab) |
| Older spreadsheets | `... (BOM v2).xlsx` (5"), `... (BOM v1).xlsx` (5", with ELRS) |
| PDF export of the old doc | Out of date (pre-air-density fix, 5") |

## 17. Sources used

- Emax ECO II 2807: https://www.flyingtech.co.uk/product/emax-eco-ii-series-2807-motor-1300kv-1500kv-1700kv/ · https://progressiverc.com/products/emax-eco-ii-2807-brushless-motor
- Gemfan 7040-3: https://www.thingbits.in/products/gemfan-flash-7040-3-7-inch-tri-blade-propellers
- SpeedyBee F405 V4 stack: https://www.thingbits.in/products/speedybee-f405-v4-bls-55a-30x30-flight-controller-esc-stack · ArduPilot docs: https://ardupilot.org/plane/docs/common-speedybeef4-v3.html
- SpeedyBee 55A ESC: https://www.getfpv.com/speedybee-55a-blheli-s-4-in-1-esc-30x30.html
- Matek H743-SLIM V3: https://www.getfpv.com/mateksys-h743-slim-v3-flight-controller.html
- T-Motor F55A Pro II: https://www.getfpv.com/t-motor-f55a-pro-ii-55a-3-6s-am32-4-in-1-esc-w-bec-30x30.html
- Matek M10Q-5883: https://rotorvillage.ca/matek-m10q-5883-gps-and-compass-module
- RunCam Thumb: https://www.racedayquads.com/products/runcam-thumb-1080p-hd-action-camera · Thumb 2 review: https://oscarliang.com/runcam-thumb-2
- ELRS MAVLink (no longer used): https://blog.unmanned.tech/elrs-mavlink-tx-as-rx-one-radio/
- 5" motor (old): https://zbotic.in/product/eco-ii-series-2207-1700kv-brushless-motor/
