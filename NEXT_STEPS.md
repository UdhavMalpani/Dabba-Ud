# Next Steps — Project 8 (as of 31 Oct 2026)

The software pipeline is finished, and all results so far come from models. What's left is to build, measure and fly, then feed each measurement back into the scripts that already exist. Commands are in [README §3](README.md#3-feeding-in-real-hardware-data).

## A. Before the thrust stand
- [ ] Measure the props with calipers. Re-run `generate_cad_stl.py --prop-d <mm>` (Sync Gate A).
- [ ] Weigh the motors, ESC, battery and FC. Update the BOM masses in `sizing_model.py`.
- [ ] **Make the spine lighter.** It is 48 g against a 30 g target and is the largest share of the +54 g printed-mass overrun. Try 3 walls with 15 % gyroid infill, or add pockets.
- [ ] Print one duct and one grille. Weigh them and check that the prop clears the duct at a 1.5 mm tip gap.
- [ ] Compile `firmware/thrust_stand_logger.ino` (needs the HX711 library) and `cad/project8_5in_parametric.scad`. Neither has been compiled yet.

## B. Stage 0 thrust stand (decision gate)
- [ ] Calibrate the load cell with 5 or more known weights.
- [ ] Sweep all 5 configurations: two open props, and the duct at 1.0, 1.5 and 2.0 mm tip gaps.
- [ ] Run `thrust_stand_analyzer.py --csv`. Its verdict sets the path: enclosed 6S, or a fallback such as 4S or open. It also picks the tip gap.
- [ ] Replace the 20 % enclosure-loss assumption with the measured value and re-run the sizing (Round 5).

## C. Flight controller on the bench
- [ ] Flash ArduPilot 4.6 and load `stage1_bare_5in.param`. Fix any parameter names it rejects (check the `GPS1_*` names).
- [ ] Confirm the M5/M6 servo outputs and the UART mapping on the F405 V4. With props off, use Motor Test to check motor order and spin direction.
- [ ] Calibrate the accelerometer, compass and radio. Bench-test the failsafes with props off.
- [ ] Install ArduPilot SITL and fly the mission there. Compare the log with the in-house simulator.

## D. Stage 1 flights (bare frame, 1,131 g)
- [ ] Work through Gates 1–7: hover, ALT_HOLD, LOITER, AutoTune. Run `flight_log_analyzer.py --log` after every flight.
- [ ] Measure the real hover throttle and set `MOT_THST_HOVER`. Expect about 0.35; don't use the sizing model's 0.45.

## E. Stage 2 flights (enclosed, about 1.96 kg)
- [ ] Mount the ballast at **+10.3 / +3.0 mm**, not the plan's −1 mm. Check the CG by hanging the aircraft.
- [ ] Gates 8–11: legs, waypoint mission, then the RC-loss, battery and geofence failsafes.
- [ ] Fly only in winds of **10 km/h or less**. Abort above 15 km/h, because gusts reach the 30° `ANGLE_MAX` limit.
- [ ] Keep `ATC_RAT_*_IMAX` at its default. The integrator has to hold the steady duct-lip pitching moment of about 0.3 N·m.

## F. Close-out
- [ ] Fill the **Measured** column of report §11 and replace the synthetic figures.
- [ ] Optional: add unit tests and a GitHub Actions workflow that re-runs the pipeline on each push.
