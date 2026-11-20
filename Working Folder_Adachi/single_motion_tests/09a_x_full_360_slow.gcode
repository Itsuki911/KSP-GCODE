; Phase 2 / Test 09a: X-axis (turntable) full 360 deg + purge-range check at slow speed (F1500 = 25 deg/s).
; Preconditions:
;   - Phase 1 passed; no workpiece installed; liquid isolated.
;   - Verify hoses/cables allow full 0..360 deg non-wrapping sweep and place 0 deg tape mark.
; Scope:
;   - Keeps Z at top endstop (47.250) and Y at 0.
;   - Sweeps X 0 -> 360 -> 0 deg, then performs 3 purge-range reversals (X285 <-> X265)
;     at squeeze/base speed F1500 (25 deg/s).
; Post-check:
;   - Verify physical 360 deg equals one full revolution and 0 deg mark aligns cleanly.
;   - Check cable/hose routing and run DUMP_TMC STEPPER=stepper_x.

G21; Set units to millimeters
G90; Absolute positioning mode
M220 S100; Reset speed factor to 100%
SET_GCODE_OFFSET X=0 Y=0 Z=0 MOVE=0; Clear inherited offsets without moving
M400
BALL_VALVE_CLOSED

G28 Z; Home Z upward to Z=47.250
G28 X Y; Home X and Y at Z=47.250
G0 F1500; Initialize base feedrate

G0 X360 F1500; Full forward revolution (0 -> 360 deg at 25 deg/s)
M400
G4 P2000; Inspect 360 deg mark alignment and cable/hose slack

G0 X0 F1500; Full reverse revolution (360 -> 0 deg at 25 deg/s)
M400
G4 P2000; Inspect 0 deg mark alignment

; Short purge-range reversal check (X285 <-> X265, 3 cycles)
G0 X285 F1500
G0 X265 F1500; #1
G0 X285 F1500
G0 X265 F1500; #2
G0 X285 F1500
G0 X265 F1500; #3
G0 X0 F1500
M400
G4 P2000; Inspect final 0 deg mark alignment before re-homing

M400
BALL_VALVE_CLOSED
G28 Z
G28 X Y
M400
M220 S100
G0 F1500
