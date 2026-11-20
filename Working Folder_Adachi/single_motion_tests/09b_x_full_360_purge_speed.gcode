; Phase 2 / Test 09b: X-axis (turntable) full 360 deg + purge-range check at purge speed (F5625 = 93.75 deg/s).
; Preconditions:
;   - Test 09a passed with accurate 360 deg mark alignment and safe cable/hose clearance.
;   - No workpiece installed; liquid isolated.
; Scope:
;   - Keeps Z at top endstop (47.250) and Y at 0.
;   - Tests X 0 -> 360 -> 0 deg and 3 purge reversals (X285 <-> X265) at purge speed F5625.
; Post-check:
;   - Verify 0 deg tape mark shows zero skipped steps under StealthChop at 93.75 deg/s.
;   - Run DUMP_TMC STEPPER=stepper_x.

G21; Set units to millimeters
G90; Absolute positioning mode
M220 S100; Reset speed factor to 100%
SET_GCODE_OFFSET X=0 Y=0 Z=0 MOVE=0; Clear inherited offsets without moving
M400
BALL_VALVE_CLOSED

G28 Z; Home Z upward to Z=47.250
G28 X Y; Home X and Y at Z=47.250
G0 F1500; Initialize base feedrate

G0 X360 F5625; Full forward revolution at 93.75 deg/s
M400
G4 P2000; Inspect 360 deg mark

G0 X0 F5625; Full reverse revolution at 93.75 deg/s
M400
G4 P2000; Inspect 0 deg mark

; Purge-range reversals at F5625 (3 cycles)
G0 X285 F5625
G0 X265 F5625; #1
G0 X285 F5625
G0 X265 F5625; #2
G0 X285 F5625
G0 X265 F5625; #3
G0 X0 F5625
M400
G4 P2000; Inspect final 0 deg mark before re-homing

M400
BALL_VALVE_CLOSED
G28 Z
G28 X Y
M400
M220 S100
G0 F1500
