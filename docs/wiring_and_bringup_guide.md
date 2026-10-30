# Project 8: Wiring and Bring-Up Guide (5-inch 6S locked baseline)

**Airframe:** enclosed ducted quad, 353 × 353 × 100 mm box, 276 mm diagonal, true-X
**Stack:** SpeedyBee F405 V4 FC (ArduPilot target `speedybeef4v4`) + SpeedyBee BLS 60A 4-in-1 ESC (flashed to **Bluejay**)
**Power:** 6S 2600 mAh 65C, XT90 · **Motors:** Emax ECO II 2207 1700KV · **Props:** 5×4.3 tri-blade
**AUW:** Stage 1 bare ~1.13 kg · Stage 2 skids 1,825 g · Stage 2 legs 1,905 g
**Parameter files:** `config/ardupilot/stage1_bare_5in.param`, `stage2_enclosed_5in_skids.param`, `stage2_enclosed_5in_legs.param`, `stage2_4s_contingency.param`

> **Verify before soldering.** The UART and output mapping below is what the `.param` files assume. Check it against the ArduPilot board page (<https://ardupilot.org/copter/docs/common-speedybeef405v4.html>) and the SpeedyBee F405 V4 manual for your board revision. If anything differs, change the wiring **and** the `SERIALn_*` / `SERVOn_*` lines in all four `.param` files so they stay consistent.

---

## 1. Assumed ArduPilot I/O map

| ArduPilot | MCU port / pad on F405 V4 | Used for | Param lines |
|---|---|---|---|
| SERIAL0 | USB-C | Mission Planner on the bench | – |
| SERIAL1 | UART1, pads **T1 / R1** | Lab telemetry radio (433 or 865 MHz), MAVLink2 @ 57600 | `SERIAL1_PROTOCOL,2` `SERIAL1_BAUD,57` |
| SERIAL2 | UART2, pad **R2 / SBUS** (T2 for CRSF/FPort TX) | Lab RC receiver | `SERIAL2_PROTOCOL,23` |
| SERIAL3 | UART3 (DJI/HD VTX connector) | Unused (RunCam Thumb is record-only) | `SERIAL3_PROTOCOL,-1` |
| SERIAL4 | UART4 | On-board Bluetooth bridge, left at board default (bench use only) | – |
| SERIAL5 | UART5 RX (**R5 / TLM pin in the 8-pin ESC plug**) | ESC serial telemetry. Bluejay has none, so it is disabled | `SERIAL5_PROTOCOL,-1` |
| SERIAL6 | UART6, pads **T6 / R6** (GPS pads) | Matek M10Q-5883 GPS | `SERIAL6_PROTOCOL,5` `SERIAL6_BAUD,230` |
| I2C | **SDA / SCL** pads | M10Q-5883 compass (QMC5883L) | `COMPASS_*` |
| SERVO1–4 | M1–M4 via 8-pin ESC harness | Motors, DShot600 + bidirectional DShot | `MOT_PWM_TYPE,6` `SERVO_BLH_BDMASK,15` |
| SERVO5–6 | **M5 / M6 pads** (assumed present) | MG90S leg servos (legs build only) | `SERVO5/6_FUNCTION,29` |

**Motor-order remap.** The SpeedyBee ESC harness uses Betaflight numbering (M1 rear-right, M2 front-right, M3 rear-left, M4 front-left). ArduPilot QUAD/X numbering is Motor1 front-right, Motor2 rear-left, Motor3 front-left, Motor4 rear-right. The param files therefore set `SERVO1..4_FUNCTION = 36, 33, 34, 35`. If you wire the motors in ArduPilot order instead, set them to `33, 34, 35, 36`. Either way, **Gate 2 (Motor Test) is the arbiter.**

**Servo-output assumption (M5/M6).** The leg servos need two outputs on a **timer group separate from M1–M4**. If they shared a group, ArduPilot would drive them with DShot and the servos would not move or would jitter. Check the following:
1. On the ArduPilot board page, confirm the board exposes M5/M6 (or equivalent) and note how the PWM timer groups are split.
2. If M5/M6 are missing or share a group with the motors, use the next free output (for example the `LED` pad, if it is listed as a PWM output) and move `SERVO5/6_*` to that output number.
3. On the bench with props off, power the UBEC, toggle the CH8 gear switch, and confirm both servos move smoothly between 1100 and 1900 µs (Mission Planner > Servo Output).

---

## 2. Pad-by-pad wiring table

| # | From | To (FC / ESC pad) | Wire | Notes |
|---|---|---|---|---|
| 1 | Battery XT90 (+) | ESC **BAT+** pad | 12 AWG | XT90 is required: 160 A peak exceeds an XT60's burst rating |
| 2 | Battery XT90 (−) | ESC **BAT−** pad | 12 AWG | Single-point ground for the whole aircraft |
| 3 | Low-ESR capacitor ≥ 35 V, ~1000 µF | ESC BAT+ / BAT− pads, shortest leads | – | Absorbs DShot/regen spikes. Observe polarity |
| 4 | ESC 8-pin: VBAT | FC 8-pin VBAT | harness | FC voltage sensing (`BATT_VOLT_MULT`, calibrate) |
| 5 | ESC 8-pin: GND | FC 8-pin GND | harness | – |
| 6 | ESC 8-pin: CUR | FC 8-pin CUR | harness | Current sensor, scale 400 (BF) = `BATT_AMP_PERVLT 25` (calibrate) |
| 7 | ESC 8-pin: TLM | FC 8-pin R5 | harness | Unused with Bluejay (`SERIAL5_PROTOCOL -1`) |
| 8 | ESC 8-pin: M1–M4 | FC 8-pin M1–M4 | harness | DShot600 + bidirectional RPM (Bluejay required) |
| 9 | Motors (3 wires each) | ESC motor pads | motor leads | Direction set in Bluejay Configurator (props-in, ArduPilot default) |
| 10 | GPS **TX** | FC **R6** | 4-wire GPS lead | Cross TX→RX |
| 11 | GPS **RX** | FC **T6** | | |
| 12 | GPS **SDA / SCL** | FC **SDA / SCL** | | Compass on I2C |
| 13 | GPS **5V / GND** | FC **5V / GND** (GPS pads) | | Mount the GPS on the top face on a ≥ 30 mm mast, away from ESC/battery leads, never under carbon |
| 14 | RC receiver signal (SBUS) | FC **SBUS / R2** pad | 3-wire | CRSF/iBus serial receiver: RX TX→R2, RX RX→T2 |
| 15 | RC receiver **5V / GND** | FC **5V / GND** | | |
| 16 | Telemetry radio **TX / RX** | FC **R1 / T1** | 4/6-wire | Radio serial speed must be 57600. Use 433 MHz or 865–867 MHz only (915 MHz is not permitted in India) |
| 17 | Telemetry radio **5V / GND** | FC **5V / GND** | | Up to ~0.5 A on transmit at 1 W. Keep the antenna vertical and clear of carbon |
| 18 | UBEC input (5V 3A) | ESC BAT+ / BAT− (after XT90) | 20 AWG | Separate 5 V rail for servos and camera |
| 19 | UBEC **5V** | Both MG90S red wires + RunCam Thumb 5V | servo leads | **Do not** connect UBEC 5V to the FC 5V rail |
| 20 | UBEC **GND** | Servo browns + FC GND | | **Common ground is mandatory**, or servo signals float |
| 21 | MG90S #1 signal (orange) | FC **M5** pad | | `SERVO5_FUNCTION 29` (legs file only) |
| 22 | MG90S #2 signal (orange) | FC **M6** pad | | `SERVO6_FUNCTION 29`. Set `SERVO6_REVERSED 1` if mirror-mounted |
| 23 | RunCam Thumb 5V / GND | UBEC 5V / GND (or FC 5V) | | Record-only, no signal wire to the FC. Front window in the payload bay |
| 24 | Buzzer + / − | FC **BZ+ / BZ−** | | Arming, failsafe and lost-model alarm |

Not used: VTX/DJI connector, LED strip, airspeed sensor, rangefinder.

---

## 3. Power budget

### 5 V avionics (from the FC BEC; check the board's rated BEC current in the SpeedyBee manual, about 2 A)

| Load | Typical | Peak | Notes |
|---|---|---|---|
| SpeedyBee F405 V4 (MCU, IMU, baro, SD logging) | 0.15 A | 0.25 A | – |
| Matek M10Q-5883 | 0.05 A | 0.07 A | – |
| Lab RC receiver | 0.05 A | 0.10 A | – |
| Telemetry radio (433/865 MHz, ≤ 1 W) | 0.15 A | 0.50 A | Transmit bursts |
| Buzzer | 0 | 0.03 A | – |
| **FC 5 V total** | **0.40 A (2.0 W)** | **0.95 A (4.8 W)** | Well inside the BEC rating |

### 5 V servo/camera rail (separate 5V 3A UBEC)

| Load | Typical | Peak | Notes |
|---|---|---|---|
| 2 × MG90S | 0.02 A (holding) | 2 × 0.7 A = 1.4 A (stall) | Moving about 0.25 A each |
| RunCam Thumb (1080p60 recording) | 0.45 A | 0.55 A | About 2.5 W |
| **UBEC total** | **0.5 A** | **≈ 2.0 A** | 3 A UBEC gives about 1 A margin at simultaneous stall |

**Total avionics from the pack** is about 5–7.5 W including converter losses, inside the 10 W `P_av` allowance used in the sizing model.

### Propulsion (6S, from the sizing model)

| Condition | Current | Check |
|---|---|---|
| Stage 1 hover (1.13 kg) | ≈ 11 A | – |
| Stage 2 hover, skids 1,825 g | 22.1 A (0% loss) / 26.4 A (+20%) | – |
| Stage 2 hover, legs 1,905 g | 23.3 A / 27.8 A | Hover time 5.4 / 4.5 min |
| Full throttle | 4 × 40 A = 160 A | ≤ 2.6 Ah × 65C = 169 A ✓. `MOT_BAT_CURR_MAX 160` |
| ESC rating | 60 A per channel | 1.5 × the 40 A per-motor peak ✓ |

---

## 4. Bench setup sequence (props OFF)

1. Flash Bluejay on the BLS 60A with ESC-Configurator (bidirectional DShot is on by default in Bluejay). Set motor directions for props-in.
2. Flash the ArduCopter 4.5.x/4.6.x `speedybeef4v4` firmware with Mission Planner > Install Firmware.
3. Load the stage `.param` file, write it, reboot, then **load and write it again** (`INS_HNTCH_*` only appears after the reboot).
4. Do the accelerometer calibration (6-point) and level, the compass calibration (outdoors, away from metal), and the radio calibration. Set the flight-mode switch to CH5, the motor emergency stop to CH7, and the gear switch to CH8 (legs build).
5. Battery monitor: calibrate `BATT_VOLT_MULT` with a multimeter. After 2–3 flights, correct `BATT_AMP_PERVLT` by the ratio of charger mAh to logged mAh.
6. Check ESC RPM telemetry: Mission Planner Status shows `esc1_rpm`…`esc4_rpm` while spinning in Motor Test. If they read zero, use the notch fallback (`INS_HNTCH_MODE 1`, `INS_HNTCH_REF = MOT_THST_HOVER`).
7. Run CompassMot (Initial Setup > Optional). If interference is above 30 %, set `COMPASS_MOTCT 2`.

---

## 5. The 11-gate flight test checklist

**Two passes:**
- **Stage 1 (bare quad, open props, about 1.13 kg):** Gates 1, 2 and 4–10.
- **Stage 2 (ducts + enclosure, 1,825 g or 1,905 g):** Gates 4–10 again with a **fresh AUTOTUNE** (inertia rises about 2–3×), then Gate 11.

**Rule:** no AUTO until LOITER is solid (Gate 9 passed).

| Gate | Test | Pass criterion | Log messages to check |
|---|---|---|---|
| **0 (prerequisite)** | Stage 0 thrust stand: duct + grilles at 1.0 / 1.5 / 2.0 mm tip gap (`firmware/thrust_stand_logger`) | Usable thrust per motor at 21.5 V and ρ 1.09: **≥ 1.14 kgf (skids)** / **≥ 1.19 kgf (legs)**. Below 1.19 kgf, fly skids only | CSV → `python src/analysis/thrust_stand_analyzer.py --csv <file>` |
| **1** | Bench setup, props off: QUAD/X, calibrations, radio, modes, battery monitor, gear switch, failsafes (section 6) | All pre-arm checks clear with `ARMING_CHECK 1`. Every failsafe in section 6 produces the expected mode change | `MSG`, `ERR`, `EV`, `MODE` |
| **2** | Motor order and direction (MP Motor Test, props off) | A = front-right CCW, B = rear-right CW, C = rear-left CCW, D = front-left CW (props-in). RPM telemetry present on all 4 | `ESC` (RPM), `RCOU` |
| **3** | Thrust stand (runs in parallel with Gates 1–2) | Same as Gate 0. Also confirms hover RPM for `INS_HNTCH_FREQ` (Stage 2 about 250 Hz) | Stand CSV |
| **4** | Vibration: 30–60 s hover at about 1.5 m | `VIBE.VibeX/Y/Z` **< 15 m/s²** (target < 10). `VIBE.Clip0/1/2` **= 0**. FFT peaks attenuated after the notch | `VIBE`, `IMU`, `ISBH/ISBD` (batch sampler), `FTN1` |
| **5** | First hover in STABILIZE at about 0.5 m (Stage 1 on skids/stand, Stage 2 on skids or legs) | Stable hover with no oscillation. Record hover current and learned `MOT_THST_HOVER` (expect about 0.2 for Stage 1, about 0.34–0.35 for Stage 2). The Stage 2 minus Stage 1 current difference is the enclosure cost | `CTUN.ThO`, `BAT.Curr`, `PARM` (`MOT_THST_HOVER`), `RCOU` |
| **6** | Trim and initial PID check | No sustained oscillation. `ATT.DesRoll` vs `ATT.Roll` tracking error < 3° in gentle inputs. Motor outputs not saturating (`RCOU` < 1900 in hover) | `ATT`, `RATE`, `PIDR/PIDP/PIDY`, `RCOU`, `MOTB` |
| **7** | ALT_HOLD at 3–5 m for 30 s | Altitude within **±0.5 m** of target (`CTUN.Alt` vs `CTUN.DAlt`) | `CTUN`, `BARO`, `MOTB.ThLimit` |
| **8** | AUTOTUNE (roll+pitch, then yaw separately for Stage 2) in calm air, LOITER/ALT_HOLD base mode | Completes and gains are saved. Post-tune `ATC_RAT_*` are sensible (D not at its limit). Stage 2: **fresh** tune | `ATUN`, `PARM`, `MSG` |
| **9** | LOITER for 60 s at 5 m | Horizontal position stays inside a **2 m circle** for 60 s. GPS HDOP < 1.4, sats ≥ 10 | `POS`, `PSCN/PSCE`, `GPS` (HDop, NSats), `XKF4` |
| **10** | Simple AUTO: takeoff → 2 waypoints → RTL/LAND (legs build: auto-retract above 5 m, deploy below 3 m) | Cross-track error **≤ 2 m** at each WP. Gear **retracts at > 5 m** and **deploys at < 3 m** AGL | `CMD`, `NTUN`/`PSCN/PSCE`, `POS`, `RCOU.C5/C6`, `EV`/`MSG` (gear events), `CTUN.Alt` |
| **11** | Full mission rehearsal (`missions/iiith_5in_enclosed_mission.waypoints`) with the 180 g payload, **3 times on 3 different days** | Every WP ≤ 2 m, gear correct, lands with **≥ 25 % battery** (`BAT.CurrTot` ≤ 1,950 mAh of 2,600). No failsafe triggered. **CG offset ≤ 5 mm** checked on the balance jig before each flight (payload in and out, legs up and down) | `BAT` (Volt, Curr, CurrTot, `Res`), `CMD`, `POS`, `GPS`, `VIBE`, `MOTB` |

**Automation:** `python src/analysis/flight_log_analyzer.py --log <file.BIN>` evaluates Gates 4–11 from a DataFlash log (it needs `pymavlink`; install it with `setup_external_tools.ps1`). It writes `outputs/logs/gate_compliance_report.json` and `outputs/plots/fig8_flight_log_gate_verification.png`. Copy the `.BIN` from the FC's SD card (`APM/LOGS/`) or download it in Mission Planner > DataFlash Logs.

**Yaw watch item (Stage 2):** yaw comes only from differences in motor reaction torque, so it is the weakest axis in a ducted box. In each Stage 2 log, compare the CW-pair and CCW-pair outputs (`RCOU` C1+C2 vs C3+C4) and the `PIDY` saturation. If yaw is weak, reduce `ATC_ACCEL_Y_MAX` and `ATC_RATE_Y_MAX` further.

---

## 6. Failsafe test procedures (do on the bench with props OFF, then repeat in SITL)

Run these with the vehicle armed in STABILIZE or LOITER on the bench, props removed, and a USB or telemetry link to Mission Planner. Watch the HUD mode and the `MSG` messages.

| Failsafe | How to trigger | Expected result | Params |
|---|---|---|---|
| RC loss | Switch the transmitter off (receiver failsafe set to "no pulses", or throttle < 975 µs) | Mode → **RTL** (on the ground with props off: disarms). Buzzer alarm | `FS_THR_ENABLE 1`, `FS_THR_VALUE 975` |
| GCS loss | Unplug the telemetry radio while in AUTO (SITL) | **RTL** in AUTO/GUIDED. Manual modes continue | `FS_GCS_ENABLE 1` |
| Battery low | Bench supply or SITL: drop voltage below 21.0 V for > 10 s, or `CurrTot` > 1,820 mAh | **RTL** | `BATT_LOW_VOLT 21.0`, `BATT_LOW_MAH 780`, `BATT_FS_LOW_ACT 2` |
| Battery critical | Below 19.8 V, or < 450 mAh remaining | **LAND** | `BATT_CRT_VOLT 19.8`, `BATT_CRT_MAH 450`, `BATT_FS_CRT_ACT 1` |
| Geofence | SITL: fly LOITER beyond 120 m radius or above 40 m | **RTL** (Land if RTL is impossible) | `FENCE_TYPE 7`, `FENCE_ACTION 1` |
| EKF/GPS | SITL: `param set SIM_GPS_DISABLE 1` (4.6: `SIM_GPS1_ENABLE 0`) | EKF failsafe → **LAND** | `FS_EKF_ACTION 1` |
| Kill switch | CH7 high | Motors stop immediately (Motor Emergency Stop) | `RC7_OPTION 31` |
| Gear switch (legs) | CH8 low/high on the bench | Both legs deploy/retract, no stall buzz at the end stops | `RC8_OPTION 29`, `SERVO5/6_MIN/MAX` |

In SITL use `param set SIM_RC_FAIL 1` for RC loss and `param set SIM_BATT_VOLTAGE 20.5` (or `19.5`) for battery failsafes, with `config/sitl/quad_stage2_enclosed_5in.parm`.

---

## 7. Safety rules

- The pilot always holds the transmitter, with a hand near the CH7 emergency stop. A safety observer calls out people and obstacles.
- Do not fly AUTO until LOITER is solid (Gate 9). Do not fly the legs build unless Gate 0 is ≥ 1.19 kgf.
- Keep a 20 m clear margin around the mission footprint. The geofence (120 m radius, 40 m altitude) is a backstop, not the plan.
- Remove props for all bench work, firmware flashing, motor tests and failsafe tests.
- Test every failsafe in section 6 before the first flight of each stage and after any parameter reload.
- LiPo: charge in a LiPo bag, never unattended. Storage-charge to 3.8 V/cell. Retire puffed packs. Land at ≥ 25 % (about 3.75 V/cell rested).
- Before every flight, check the CG on the jig (≤ 5 mm), the payload door latched, all quarters locked, prop nuts tight, grilles seated, and the GPS mast intact.
- Check institute permission and the DGCA Digital Sky (nano/micro category, green zone) requirements before outdoor flights.
- Fly only below 15 km/h wind for Stage 2 until Gate 9 passes.
