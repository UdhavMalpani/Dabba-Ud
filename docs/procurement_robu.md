# Project 8: Robu.in Procurement Check

**Checked:** 4 Nov 2026, against *Project 8 – Component List BOM final.pdf* · **Ship to:** IIIT Hyderabad, Gachibowli (PIN 500032)

> **Read first.** Robu.in blocks automated page loads, so stock status and prices below come from search-engine snapshots of Robu's pages, some of them months old. **Open every link and confirm price, stock and the exact variant before ordering.** Robu ships across India, so Hyderabad delivery isn't a problem; enter PIN 500032 at checkout to see the delivery estimate. "✓" means a Robu listing exists for this item. It does not guarantee the item is in stock today.

---

## 1. Summary of changes

| # | BOM item | Problem | Recommendation |
|---|---|---|---|
| 5, 10 | SpeedyBee F405 V4 **BLS 60A** stack | **Out of stock** on Robu (the 55A version is too) | **SpeedyBee F405 V3 BLS 50A stack** (ArduPilot target `speedybeef4v3`). 50 A continuous per motor vs our ~40 A peak, so a 1.25× margin. Includes a 1500 µF capacitor. The parameter files need regenerating for the V3 board's UART and output map |
| 6 | 6S **2600 mAh ≥65C** XT90 | **No 6S 2200–2600 ≥65C pack on Robu** | **Bonka 6S 1800 mAh 120C** (313 g). See §2 for the endurance trade-off. Or buy the 2600 mAh pack from another seller |
| 3 | HQProp **5×4.3×3** | Not on Robu | **Pro-Range 5045 tri-blade bullnose** (5.0", keeps the 127 mm ducts). The Pro-Range 51433 (5.1") is the backup, but it needs the ducts regenerated, which leaves only 1.3 mm clearance at the spine |
| 11 | Matek **M10Q-5883** GPS | Not on Robu | **Foxeer M10Q 250 GPS 5883 compass**, same chip family |
| 17 | RunCam **Thumb** | Out of stock | **RunCam Thumb Pro** if in stock, otherwise any ≤ 30 g 1080p camera. The payload stays 180 g; adjust the ballast to match |
| 31 | **PA-CF** filament | Not found on Robu | Print the hinges and mounts in **PETG** for now (lower stiffness), or buy PA-CF elsewhere |
| 32 | **TPU** filament | Pro-Range TPU 95A listed **out of stock** | Check live stock. Only 0.1 kg is needed; any TPU 95A brand works |
| — | Small parts | The BOM didn't list connectors, screws, leads etc. | Added in §4 |

---

## 2. Battery decision (needs a team call)

Robu has no 6S pack between 1800 mAh at ≥ 65C and 2600 mAh. I ran the options through the sizing model (`src/sizing/sizing_model.py`):

| Pack | Mass | Max continuous current | Max thrust/motor | T/W (skids / legs) | Hover time, 0 % / +20 % enclosure loss (legs) | Verdict |
|---|---|---|---|---|---|---|
| **BOM spec:** 6S 2600 65C | 400 g | 169 A | 1.41 kgf | 3.09 / 2.96 | 5.4 / 4.5 min | Best, but not on Robu |
| **Bonka 6S 1800 120C** | 313 g | 216 A | 1.41 kgf | **3.24 / 3.10** | **4.0 / 3.3 min** | ✓ Plenty of thrust. Endurance is short but covers the mission (the simulated mission took 2.15 min) |
| Pro-Range/Bonka 6S 2500 35C | ~390 g | 88 A | 0.94 kgf | 2.08 / 1.99 | 5.2 / 4.4 min | ✗ The pack can't supply full thrust, so T/W falls below the 2.5 target |
| Orange 6S 3300 35C | ~520 g | 116 A | 1.13 kgf | 2.33 / 2.24 | 6.2 / 5.2 min | ✗ Below the T/W target, and heavy |
| 2 × Bonka 1800 in parallel | 626 g | 432 A | 1.41 kgf | 2.75 / 2.65 | 6.3 / 5.3 min | Possible, but heavy and needs a parallel harness and space in the bay |

**Recommendation:**
- **Ideal:** buy a 6S 2200–2600 mAh ≥ 65C pack from another Indian RC seller, so the design numbers stay as they are.
- **Robu only:** buy **3 × Bonka 6S 1800 120C**. Every mission and hover number in the report then has to be recomputed (I can do this). The mission still lands with about 40 % left in the model.

**Before ordering, check two things:**
- the pack's **dimensions** against the 45 mm spine;
- its **connector**. Bonka FPV packs usually come with **XT60**. Either re-terminate to XT90 or accept XT60, which is rated 60 A continuous with short bursts above that.

---

## 3. Flight hardware

| BOM # | Item | Robu listing | Status | ≈ Price (₹) | Qty | Notes |
|---|---|---|---|---|---|---|
| 1 | Emax ECO II 2207 1700 KV | [link](https://robu.in/product/eco-ii-series-2207-1700kv-brushless-motor/) | ✓ in stock | 1,949 | 5 | 33.4 g without leads. Comes with an M5 prop nut and mounting screws. **Leads are bare, with no connectors:** solder them directly to the 4-in-1 ESC pads. Make sure it's the ECO **II**, not the ECO III, which has the same price and lower thrust |
| 3 | 5" tri-blade props | [Pro-Range 5045 tri bullnose](https://robu.in/product/orange-hd-propellers-50455x4-5-blade-bullnose-2cw2ccw-2pairs-orange/) · backup [51433](https://robu.in/product/orange-hd-51433-hurricane-pc-3-blade-orange/) | ✓ | ~200 / set | 4 sets | **Measure the diameter with calipers before printing the ducts** |
| 4 | 5" two-blade props (stand comparison only) | Pro-Range 5045 two-blade | check | — | 2 sets | Optional |
| 5 + 10 | FC + ESC stack | [SpeedyBee F405 V3 BLS 50A](https://robu.in/product/speedybee-sb-f4v3-50-stack/) (replaces the [V4 60A](https://robu.in/product/speedybee-f405-v4-bls-60a-30x30-fcesc-stack/), out of stock) | ✓ (stock uncertain) | ~6,500 | 1 + 1 spare | If the V4 60A comes back in stock, prefer it and keep the current params. The standalone [BLS 60A ESC](https://robu.in/product/speedybee-bls-60a-30x30-4-in-1-esc/) (~₹4,822) is also listed |
| 6 | 6S battery | [Bonka 6S 1800 120C](https://robu.in/product/bonka-1800mah-120c-6s-fpv-u2-series-lithium-polymer-battery-pack/) | ✓ (one snapshot showed out of stock) | 5,989 | 3 | See §2 |
| 7 | 1000 µF 50 V capacitor | [Nichicon UVR1H102MHD](https://robu.in/product/uvr1h102mhd-nichicon-aluminum-electrolytic-capacitor-1000uf/) | ✓ | 44 | 2 | The stack already includes one. This is a spare |
| 8 | XT90 connectors | [Amass XT90 male](https://robu.in/product/xt90-housing-male-1-piece/) · [female](https://robu.in/product/xt90-housing-female-1-piece/) · [female pigtail 12 AWG](https://robu.in/product/xt90-plug-female-10awg-10cm-1pcsbag/) | ✓ | 71–120 each | 3 M + 3 F | See §4 |
| 9 | 12 AWG silicone wire | [1 m red + 1 m black](https://robu.in/product/high-quality-12awg-silicone-wire-1m-red-1m-black/) | ✓ | 228 | 1 | |
| 11 | GPS + compass | [Foxeer M10Q 250 GPS 5883](https://robu.in/product/foxeer-m10q-250-gps-5883-compass/) | ✓ | 2,729 | 1 | Same M10 chip. Mount it on the top deck |
| 13, 15 | Telemetry (only if the lab has none) | [Holybro SiK V3 100 mW 433 MHz](https://robu.in/product/holybro-sik-telemetry-radio-v3-100-mw-433-mhz/) | ✓ | 6,499+ | 1 pair | **433 MHz only.** 915 MHz is not legal in India |
| 16 | Buzzer | [5 V active electromagnetic buzzer (pack of 5)](https://robu.in/product/5v-active-electromagnetic-buzzer-pack-of-5/) | ✓ | ~100 | 1 pack | Goes on the FC's BZ+/BZ− pads |
| 17 | Camera | [RunCam Thumb Pro](https://robu.in/product/runcam-thumb-pro-nd-zz/) (Thumb [out of stock](https://robu.in/product/runcam-thumb-1080p-mini-action-camera/)) | check | ~10,700 | 1 | Optional for the first flights |
| 19 | Carbon plate (frame) | [4 mm, 300 × 300 mm](https://robu.in/product/carbon-fiber-sheet-300300mm-thickness-4-mm/) | ✓ | 4,693 | 1 | The 276 mm true-X fits on the sheet. Cut it wet, with a mask: carbon dust is hazardous |
| A1 | MG90S servo, metal gear, 180° | [link](https://robu.in/product/mg90s-9g-servo-metal-gear/) | ✓ | 269 | 3 | **Not** the 360° continuous-rotation version |
| A2 | UBEC 5 V 3 A, 2–6S | [Bluesky mini UBEC](https://robu.in/product/bluesky-mini-5v-3a-fpv-gimbal-ptz-ubec-for-rc-air-models/) or [ReadytoSky switchable](https://robu.in/product/readytosky-2-6s-5v-3a-and-12v-3a-switchable-ubec/) | ✓ | 279 | 1 | Set the ReadytoSky to 5 V before connecting |

---

## 4. Small parts the BOM didn't list

| Item | Why it's needed | Robu listing | Qty |
|---|---|---|---|
| **M3 × 8 socket-head screws** | Motor to 4 mm carbon arm. The included motor screws may be too short for a 4 mm plate. **They must not reach more than 4 mm into the motor** | [M3×8 set](https://robu.in/product/easymech-set-m3-x-8mm-socket-head-cap-allen-bolt-nut-12-pcs/) | 2 packs |
| M3 × 6 / 12 / 20 / 30 screws | Stack, enclosure quarters, standoffs | EasyMech M3 SHCS sets: [6](https://robu.in/product/easymech-m3-x-6mm-chhd-bolt-nut-and-washer-set-25-pcs/), [20](https://robu.in/product/easymech-set-of-m3-x-20mm-socket-head-cap-allen-bolt-and-nut-12-pcs/), [30](https://robu.in/product/easymech-set-m3-x-30mm-socket-head-cap-allen-bolt-nut-12-pcs/) | 1 each |
| M3 nyloc nuts | Frame bolts | [EasyMech M3 nyloc](https://robu.in/product/easymech-set-m3-ms-nyloc-nut-25pcs/) | 1 pack |
| **M3 brass heat-set inserts**, 4 mm | Quarter joints, deck bosses (BOM #33) | [M3×4, 25 pcs](https://robu.in/product/m3-x-4-mm-brass-heat-set-knurl-threaded-round-insert-nut-25-pcs/) | 2 packs |
| **XT90 male + female** | ESC lead and spare battery leads. Also needed if the battery comes with XT60 and has to be re-terminated | Amass XT90 M/F (§3) | 3 + 3 |
| XT60 male + female pair | Charger lead, watt meter, thrust stand | [Amass XT60 pair](https://robu.in/product/amass-xt60-gold-plated-bullet-battery-connector-pair-xt60-fgy-and-xt60-mgy/) | 3 pairs |
| 3.5 mm gold bullets (M + F) | **Not needed on the aircraft**, since motors solder straight to the 4-in-1. Useful for the thrust stand, so motors can be swapped without soldering | [male](https://robu.in/product/amass-3-5mm-bullet-male-connector-gc3510-m/) · [female](https://robu.in/product/amass-3-5mm-bullet-female-connector-gc3510-f/) | 6 + 6 |
| 18 AWG silicone wire | Motor lead extensions if the leads don't reach the centre stack (~138 mm from motor to centre) | Robu silicone wire, 18 AWG | 1 m each of 3 colours |
| Servo extension leads, 30 cm | Leg servos to the FC (BOM A5) | Robu "servo extension cable 30cm" | 3 |
| Heat-shrink assortment | Every solder joint | Robu heat-shrink kit | 1 |
| Zip ties (100 mm), Velcro strap, 3M foam tape | Wire routing, receiver mounting | — | 1 each |
| **Battery strap**, 20 × 300 mm non-slip | BOM #36 | [30 cm strap](https://robu.in/product/30cm-lipo-battery-strap-belt-reusable-cable-tie-wrap/) | 2 |
| Medium-strength threadlocker (Loctite 243) | Motor and frame bolts (into metal, **not** into plastic) | Robu or a hardware shop | 1 |
| Spare M5 prop nuts (CW + CCW) | They get lost | Robu "M5 prop nut" | 4 + 4 |
| Solder wire 60/40 + flux paste | XT90 and 12 AWG joints need a hot iron (≥ 60 W) and flux | — | 1 each |
| **Smoke stopper** or a 5 A blade fuse in-line | First power-up of every new build. It stops a wiring short from burning the stack | Robu "smoke stopper" | 1 |
| Kapton tape | Insulating under the stack, heat-proof | — | 1 roll |

---

## 5. Build materials

| BOM # | Item | Robu listing | ≈ Price (₹) | Qty |
|---|---|---|---|---|
| 29 | PLA | [Bambu PLA Basic 1 kg](https://robu.in/product/bambu-lab-pla-basic-1-75mm-3d-printing-filament-4/) | 1,149–1,299 | 2 |
| 30 | PETG | [Bambu PETG HF 1 kg](https://robu.in/product/bambu-lab-petg-hf-1-75mm-filament-peanut-brown-2/) or [Bambu PETG](https://robu.in/product/bambu-lab-petg-1-75mm-filament-translucent-gray/) | 1,299 | 1 |
| 31 | PA-CF | **Not found**. Use PETG, or source elsewhere | — | — |
| 32 | TPU 95A | [Pro-Range TPU 95A](https://robu.in/product/pro-range-tpu-95a-filament-1-75mm-1-kg-spool-gray/) (listed out of stock) | 1,029 | 1 |

---

## 6. Support and test equipment

| BOM # | Item | Robu listing | ≈ Price (₹) | Notes |
|---|---|---|---|---|
| 38 | Load cell + HX711 (thrust stand) | [5 kg + HX711 kit](https://robu.in/product/5-kg-load-cell-with-hx711ad-module-shell-and-4p-dupont-wire-kit/) | 449 | Works with `firmware/thrust_stand_logger` |
| 38 | Watt meter | [150 A watt meter / power analyser](https://robu.in/product/150a-high-precision-watt-meter-power-analyzer/) | 689 | Bare leads: solder XT60s on |
| 39 | 6S charger | [SkyRC B6ACneo](https://robu.in/product/skyrc-b6ac-neo-dc-200w-ac-60w-battery-charger/) (AC built in) or [ToolkitRC M7](https://robu.in/product/toolkitrc-m7-200w-10a-2-6s-dc-smart-charger/) (DC only: also needs a [ToolkitRC ADP200](https://robu.in/product/toolkitrc-adp200-ac-power-adapter/)) | 5,769 / 4,999 | The B6ACneo plugs straight into the wall |
| 40 | LiPo bag | [230 × 300 mm fireproof](https://robu.in/product/red-lithium-battery-protection-bag/) | ~300 | |
| 41 | Cell checker | [1–8S checker with buzzer](https://robu.in/product/lipo-battery-voltage-checker-1s-8s-with-buzzer/) | 191 | |
| 42 | Prop balancer | Robu magnetic prop balancer | — | Check the listing |

---

## 7. Fallback sets (order only if Gate 1 calls for them)

| Set | Item | Robu listing | Status |
|---|---|---|---|
| 7-inch | Emax ECO II 2807 1300 KV | [link](https://robu.in/product/emax-ecoii-2807-1300kv-brushless-motor/) | ✓ ₹2,259. Robu's spec table looks copy-pasted (it quotes 1380 g thrust and 9×6 props); check against Emax's datasheet |
| 7-inch | 7040 tri-blade (Gemfan not listed) | [Pro-Range 7040 Flash PC](https://robu.in/product/orange-hd-7040-flash-pc-3-blade-propellers-2cw2ccw-black/) | ✓ (178.4 mm) |
| 7-inch | 6S 3300 ≥ 60C | Only the [Orange 6S 3300 **35C**](https://robu.in/product/orange-3300mah-6s-35c-22-2-v-lithium-polymer-battery-pack-li-po/) (116 A) | ✗ Needs a current limit (`MOT_BAT_CURR_MAX` ≈ 110 A) |
| 4S | Emax ECO II 2207 2400 KV | [link](https://robu.in/product/emax-ecoii-2207-2400kv-brushless-motor/) | ✓ ₹1,611 |
| 4S | 4S 4000 ≥ 60C | Not found | ✗ |

---

## 8. Approximate total (5-inch build, Robu-only, with spares)

| Group | ≈ ₹ |
|---|---|
| Motors (5) | 9,750 |
| Stacks (2) | 13,000 |
| Batteries (3 × Bonka 1800) | 18,000 |
| GPS, props, servos, UBEC, capacitor, buzzer | 4,500 |
| Carbon plate | 4,700 |
| Filament (PLA × 2, PETG, TPU) | 4,800 |
| Connectors, wire, screws, inserts, straps, small parts | 2,500 |
| Charger, bag, checker, load cell, watt meter | 7,500 |
| **Total without camera or telemetry** | **≈ 65,000** |
| Camera (Thumb Pro) / telemetry pair, if the lab can't lend them | + 10,700 / + 6,500 |

Prices include GST where Robu shows it, and exclude shipping.
