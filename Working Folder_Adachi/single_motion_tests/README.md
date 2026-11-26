# Single-Motion & Staged Commissioning Tests (`Working Folder_Adachi`)

This directory contains staged, single-motion commissioning G-code tests designed to bridge the gap between offline-validated code (`Working Folder_Adachi/testPaint.gcode`) and real hardware behavior on the KSP coating machine (`printer_data/config/printer.cfg`).

> **WARNING — NOT CERTIFIED COLLISION-FREE**
> Valid G-code does **not** guarantee safe physical execution. Wiring errors, inverted endstops, skipped steps under StealthChop, Z-axis TMC2209 `s2vsa` faults, unmeasured clearances, and generic `PAUSE`/`CANCEL_PRINT` macros can still cause crashes or liquid spills. Run every phase strictly in order, dry/liquid-isolated, with a hand on a verified **physical emergency stop / power cutoff**.

---

## 1. Shared G-code Standard (Aligned with `Working Folder_Adachi/testPaint.gcode`)

The motion and valve `.gcode` files in this folder enforce the modal and motion safety invariants introduced after the independent audit:

1. **Deterministic Safety Preamble** (before any homing or motion):
   - `G21` (millimeters), `G90` (absolute coordinates), `M220 S100` (100% feed override), `SET_GCODE_OFFSET X=0 Y=0 Z=0 MOVE=0` (zero inherited XYZ offsets), `M400` + `BALL_VALVE_CLOSED`.
2. **Z-First Homing**:
   - `G28 Z` (upward to `Z=47.250`) always precedes `G28 X Y` (`X=0`, `Y=0`). Tests that do not require X/Y movement (`01`, `03`, `04`, `08a`, `08b`) avoid unnecessary axis homing/motion.
3. **Explicit Feedrates (`F`) & Fixed `M220 S100`**:
   - Removes reliance on `M220 S500` and inherited feedrates. Every moving `G0` block specifies `F` explicitly (`Z <= F1200` / `20 mm/s`, `XY <= F7500` / `125 planner units/s`).
   - Remember: `X` is in **degrees** while `Y`/`Z` are in **mm**. `F` is in mixed planner units/min, not uniform linear surface speed.
4. **Strict Separation of Z and XY & Strict Z Bound (`0 <= Z <= 47.250`)**:
   - Zero simultaneous Z+XY (diagonal) moves.
   - Zero commands exceeding the configured upper switch coordinate (`47.250`), eliminating legacy `Z50` / `Z53` overtravel commands.
5. **Valve Motion Barriers**:
   - Every `BALL_VALVE_OPEN` / `BALL_VALVE_CLOSED` is preceded by `M400`. (Measured servo/fluid settling `G4 P...` delays must be populated after Test `03`.)
6. **Clean Epilogue**:
   - Normal completion closes the valve (`M400` + `BALL_VALVE_CLOSED`), returns via `G28 Z` then `G28 X Y` (never bare `G28`), and leaves `M220 S100` and `G0 F1500`.

---

## 2. Phase 0 — Zero-Motion & Micro-Actuation Pre-Flight (Before Running Any G-code)

**Do not run `G28` until Phase 0 passes.** Because `printer.cfg` sets `homing_retract_dist: 0`, Klipper skips second-pass switch verification and will travel up to **70.875 mm (Z), 540° (X), or 270 mm (Y)** if a switch fails to trigger.

Execute these checks from the Mainsail/Klipper console with **no workpiece** and **liquid physically isolated**. For step 0.2, [`00_endstop_sensor_check.gcode`](./00_endstop_sensor_check.gcode) provides timed prompts and readings without commanding axes, homing, or the valve. It is intentionally exempt from the motion-test preamble and epilogue.

| Step | Command / Action | Pass Criterion | Stop / Investigate If |
| --- | --- | --- | --- |
| 0.1 | Test physical power cutoff / hard E-stop and `M112` | Motion power and firmware halt immediately. **Do NOT rely on Mainsail `PAUSE` or `CANCEL_PRINT`** (default `client.cfg` parks at `X355 Y175` with only `+2 mm` Z lift and does not close the valve). | Stop path is slow, inaccessible, or unreliable. |
| 0.2 | `QUERY_ENDSTOPS` (unpressed, then press each switch by hand) | `x`, `y`, `z` each toggle cleanly between `open` and `TRIGGERED` on the correct axis. | Wrong axis triggers, inverted logic, or stuck `TRIGGERED` / `open`. |
| 0.3 | `DUMP_TMC STEPPER=stepper_x`, `stepper_y`, `stepper_z` | Valid UART response; no `drv_err`, `s2vsa`, `s2vsb`, `s2ga`, `s2gb`, `ot`, or `otpw` flags. | Any TMC fault (note: historical logs show Z-axis `s2vsa` short-to-supply faults even after short runs). |
| 0.4 | `STEPPER_BUZZ STEPPER=stepper_z`, then `stepper_x`, then `stepper_y` | Each motor oscillates a few mm/deg smoothly on the **expected axis and direction** (Z positive = upward toward switch). | Wrong axis moves, reversed direction, grinding, or shutdown. |

Run `00_endstop_sensor_check.gcode` only when the machine is idle and the console is visible. First release all sensors; then, at each prompt, block or release the named optical sensor by hand and keep the other sensors released. Hold each state until the console prints the reading. The script waits 15 seconds per prompt, including the baseline (about 105 seconds total). Expect all three `open` at baseline and after each release, and only the named axis `TRIGGERED` when blocked. Review all seven readings manually; the script does not stop automatically or declare a pass. If the timing is missed, rerun it or issue `QUERY_ENDSTOPS` manually. A wrong axis, inverted or stuck state is a failure: stop before `G28` and inspect wiring and `printer.cfg`. This verifies input response, not physical homing direction or repeatability.

---

## 3. Staged Single-Motion Test Suite (Phases 1–3)

Run **one file at a time**. After every file, inspect physical alignment marks, motor temperature, `DUMP_TMC`, and `klippy.log` before advancing.

### Phase 1 — Minimal-Stroke, Low-Speed Verification (No Workpiece, Dry)

| File | Axes / Actuator | Summary | Physical Pass Criteria |
| --- | --- | --- | --- |
| [`01_home_z_only.gcode`](./01_home_z_only.gcode) | Z only | Safe revision of `gcodes/PaintFile.gcode`: safety preamble + `G28 Z` only. | Z moves upward and stops cleanly on upper switch; `DUMP_TMC STEPPER=stepper_z` clean. |
| [`02_home_xy_only.gcode`](./02_home_xy_only.gcode) | Z → X, Y | `G28 Z` then `G28 X Y` only (no travel moves). | Z stays at top; X and Y home cleanly to `0` without stalling or switch bounce. |
| [`03_valve_static.gcode`](./03_valve_static.gcode) | Ball valve only (zero axis motion) | 3 static `OPEN` (45°) / `CLOSED` (0°) cycles with `3.0 s` dwells. | Measure actual open/close times (ms) for `testPaint.gcode` `G4 P...`; verify full mechanical shutoff; separately test valve behavior under `M112` / power loss (`PWM=0`). |
| [`04_z_step_small.gcode`](./04_z_step_small.gcode) | Z only | `Z=47.25 → 42.00 → 47.25` (`5.25 mm`) at `F300` (`5 mm/s`) with `2 s` hold. | Physical travel ≈ `5.25 mm`; no sag during hold; `DUMP_TMC` clean. |
| [`05_x_step_small.gcode`](./05_x_step_small.gcode) | X only (Z=47.25, Y=0) | `X=0 → 30° → 0°` at `F600` (`10 deg/s`) with `2 s` pauses. | Physical angle ≈ `30°` (confirms `gear_ratio 125:20`); `0°` tape mark aligns on return. |
| [`06_y_step_small.gcode`](./06_y_step_small.gcode) | Y only (Z=47.25, X=0) | `Y=0 → 20 mm → 0 mm` at `F600` (`10 mm/s`) with `2 s` pauses. | Measured travel ≈ `20 mm`; carriage returns cleanly to `Y=0` mark. |

### Phase 2 — Full-Range, Repeatability, Speed-Stepped, & Safe Revisions of Existing Tests (No Workpiece, Dry)

| File | Axes | Summary | Physical Pass Criteria |
| --- | --- | --- | --- |
| [`07_homing_repeatability.gcode`](./07_homing_repeatability.gcode) | Z, then X, then Y | Moves each axis away (`Z33`, `X30`, `Y30`) and re-homes 3× with `3 s` pauses. | Quantify homing scatter under `homing_retract_dist: 0`; if drift is excessive, fix `printer.cfg` retract settings before coating. |
| [`08a_z_full_range_slow.gcode`](./08a_z_full_range_slow.gcode) | Z only | Steps through all `testPaint` Z levels (`33, 24, 19, 18, 16, 11, 8, 6`) at `F600` (`10 mm/s`) with `2–3 s` holds. | Verify empty-bed bottom clearance at `Z=6`, stable holding, no `s2vsa` / `drv_err`. |
| [`08b_z_full_range_rated.gcode`](./08b_z_full_range_rated.gcode) | Z only | Same Z staircase at rated `F1200` (`20 mm/s`). | Smooth motion without skipped steps, vibration, or TMC fault. |
| [`09a_x_full_360_slow.gcode`](./09a_x_full_360_slow.gcode) | X only (Z=47.25, Y=0) | `0 → 360 → 0°` + 3× `285 ↔ 265°` at `F1500` (`25 deg/s`). | Exact 1 revolution; zero hose/cable entanglement; `0°` mark returns cleanly. |
| [`09b_x_full_360_purge_speed.gcode`](./09b_x_full_360_purge_speed.gcode) | X only (Z=47.25, Y=0) | Same sweep + reversals at purge speed `F5625` (`93.75 deg/s`). | Zero skipped steps under StealthChop; `0°` mark aligns before re-home. |
| [`09c_x_full_360_coating_speed.gcode`](./09c_x_full_360_coating_speed.gcode) | X only (Z=47.25, Y=0) | Same sweep + reversals at coating speed `F7500` (`125 deg/s`). | Zero skipped steps; acceptable vibration/resonance (note: at `r=135 mm`, surface speed ≈ `294.5 mm/s`). |
| [`10_y_calibration_stops.gcode`](./10_y_calibration_stops.gcode) | Y only (Z=47.25, X=0) | Stops `4 s` at every `testPaint` Y coordinate (`15, 35, 45, 47, 77, 180`) at `F1500`. | Measure physical radius `r(Y)` at every stop and verify whether `Y=180` aligns with turntable rotation center. |
| [`11_cablechain_safe_short.gcode`](./11_cablechain_safe_short.gcode) | Y only, then X, then Z only | Safe short revision of `gcodes/cableChainTest.gcode`: 5× `Y0↔180` (`F1500`) at `Z=47.25`, then `X=100` at `Z=47.25`, then 5× `Z10↔45` (`F1200`). | Smooth cable-chain flexing without pinching, snagging, or exceeding `Z=47.25`. |
| [`12_turntable_breakin_safe_short.gcode`](./12_turntable_breakin_safe_short.gcode) | X+Y (Z=47.25) | Safe short revision of `gcodes/turntableBreakin.gcode`: 5× `X0 Y180 ↔ X360 Y0` at `F1500` with Z held at `47.25`. | Smooth simultaneous XY spiral motion; zero skipped steps or hose interference. |

### Phase 3 — Workpiece-Installed High-Z Static Clearance Check (Dry)

| File | Axes | Summary | Physical Pass Criteria |
| --- | --- | --- | --- |
| [`13_clearance_check_high_z.gcode`](./13_clearance_check_high_z.gcode) | X, Y, Z (strictly separated) | Visits every `testPaint.gcode` feature XY waypoint (Purge, Circle 1–5, Raised 2/1, Inner) strictly at high travel Z (`33, 25, 24`) at low speed (`F1500` XY, `F600` Z) with `3–4 s` pauses. **Never descends to working Z (`19, 18, 16, 8, 6`).** | Visually confirm positive physical clearance over all workpiece features, clamps, and fixtures along the entire swept arc (including negative `X250 → X30` sweep) before attempting `Working Folder_Adachi/testPaint.gcode`. |

---

## 4. Mapping & Key Fixes vs. Legacy `gcodes/` Files

Original files under `gcodes/` are left completely untouched as audit baselines.

| Legacy File (`gcodes/`) | Issues Found in Legacy File | Safe Replacement in `single_motion_tests/` |
| --- | --- | --- |
| `gcodes/PaintFile.gcode` | Missing `G21`/`G90`/`M220 S100`/`SET_GCODE_OFFSET`/initial `BALL_VALVE_CLOSED`; ambiguous filename. | `01_home_z_only.gcode` |
| `gcodes/cableChainTest.gcode` | **`G0 Z50` exceeded upper switch `47.250` by `+2.75 mm`**; **`G0 X100 Z0` caused a diagonal XZ plunge to `Z=0`**; `M220 S500` with inherited `F`; 180 long sets; no ending `G28` or state restore. | `11_cablechain_safe_short.gcode` (plus single-axis `08a/08b`, `10`) |
| `gcodes/turntableBreakin.gcode` | Missing `G90`/offset clear/initial close; `M220 S500` with inherited `F`; mixes X+Y simultaneously (cannot isolate single-axis faults); 841 cycles; no ending `G28` or state restore. | `12_turntable_breakin_safe_short.gcode` (plus single-axis `09a/09b/09c`, `10`) |
| `gcodes/testInnerCircleOnly.gcode` | Partial coating sequence (not single-motion): has unknown `DE3` on line 1, 4 diagonal Z+XY moves, `G0 Z50 Y2` overtravel, outdated purge coordinates (`X360..340`). Not revised here; use Phase 1–3 tests then `Working Folder_Adachi/testPaint.gcode`. | *(Out of scope for single-motion suite)* |

---

## 5. Physical Measurement Log Sheet (Fill During Machine Commissioning)

Record these measurements during Phases 0–3 and feed the validated numbers back into `Working Folder_Adachi/testPaint.gcode` and `printer.cfg`:

- **Endstop polarity & homing repeatability (`01`, `02`, `07`)**: Z scatter = `___ mm`, X scatter = `___ deg`, Y scatter = `___ mm`
- **Valve response time (`03`)**: Open time = `___ ms`, Close time = `___ ms`, Behavior on `M112` / power off = `___`
- **Axis scaling & backlash (`04`, `05`, `06`, `09a–c`)**: Z actual/5.25 mm = `___`, X actual/360° = `___`, X return error = `___ deg`, Y actual/20 mm = `___`
- **Radial calibration `r(Y)` (`10`)**: `Y15 = ___ mm`, `Y35 = ___ mm`, `Y45 = ___ mm`, `Y47 = ___ mm`, `Y77 = ___ mm`, `Y180 = ___ mm` (center offset = `___ mm`)
- **TMC diagnostics (`DUMP_TMC` after `01`, `04`, `08a/b`, `09a–c`, `11`)**: `stepper_z` flags = `___`, `stepper_x` flags = `___`, `stepper_y` flags = `___`
- **Minimum observed workpiece clearance (`13`)**: Purge (`Z24`) = `___ mm`, Circles (`Z24/25`) = `___ mm`, Raised (`Z25`) = `___ mm`, Inner (`Z24`) = `___ mm`

---

## 6. Issues That G-code Alone Cannot Solve (`printer.cfg` / Hardware)

Passing these G-code tests does **not** resolve the following machine-level issues documented in the audit; address them in parallel:

1. **Abort / Pause Safety**: Default `CANCEL_PRINT`, `PAUSE`, and `RESUME` in `mainsail-config/client.cfg` do not command `BALL_VALVE_CLOSED`, park to `X355 Y175` with only `+2 mm` Z lift, and can resume diagonally. Configure machine-specific `_CLIENT_VARIABLE` or custom macros.
2. **Z-Axis TMC2209 `s2vsa` Faults**: Verify installed Z motor model/RMS current rating (`0.4 A` OMC vs. `1.0 A` LDO comments in `printer.cfg`), sense resistors, wiring/connectors, and Z holding torque before unattended runs.
3. **MCU Shutdown Valve Isolation**: On shutdown, Klipper sets servo PWM to `0` (not `ANGLE=0`). Ensure a mechanical or upstream hardware shutoff prevents liquid flow on power/MCU loss.
4. **Zero-Retract Homing (`homing_retract_dist: 0`)**: If Test `07` shows scatter or switch-sticking risk, validate a safe retract distance and second-homing pass in `printer.cfg`.

---

## 7. Offline Regression Validation

Run the offline validator from the repository root (no printer connection required):

```bash
python3 'Working Folder_Adachi/single_motion_tests/validate_tests.py'
```

What `validate_tests.py` verifies using the local `klipper/klippy/extras/gcode_move.py` (`GCodeMove`) implementation:
- All 16 motion/valve `.gcode` files produce **identical trajectories** across 3 hostile incoming modal states (`G91`, non-100% `M220`, arbitrary `F`, non-zero XYZ `SET_GCODE_OFFSET`).
- Complete safety preamble before any homing/motion, Z-before-XY homing order, and clean closed/restored epilogue.
- Zero simultaneous Z+XY diagonal moves and strict `0 <= X <= 360`, `0 <= Y <= 180`, `0 <= Z <= 47.250` bounds.
- Explicit `F` on every moving block (`Z <= 20 mm/s`, `XY <= 125 units/s`) and `M400` immediately before every valve command.
- All 9 original `gcodes/*.gcode` files and `Working Folder_Adachi/testPaint.gcode` remain byte-identical (SHA-256 verified).

*Limitation*: Offline validation checks Klipper G-code modal semantics and software bounds only; it does not simulate homing switches, servo timing, electrical faults, fluid dynamics, or physical collisions.
