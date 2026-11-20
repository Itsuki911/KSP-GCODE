; Phase 1 / Test 05: X-axis (turntable) minimal angle test (30 deg at low speed F600).
; Preconditions:
;   - Tests 01-04 passed; no workpiece installed; liquid physically isolated.
;   - Place a physical alignment mark (tape) at X=0 before running.
; Scope:
;   - Keeps Z at top endstop (47.250) and Y at 0.
;   - Rotates X from 0 to 30 deg at F600 (10 deg/s), pauses 2 s, returns to 0 deg.
; Post-check:
;   - Confirm physical rotation is ~30 deg (verifies gear_ratio 125:20 direction/scale)
;     and that the mark returns accurately to X=0 (checks backlash/slip).

G21; Set units to millimeters
G90; Absolute positioning mode
M220 S100; Reset speed factor to 100%
SET_GCODE_OFFSET X=0 Y=0 Z=0 MOVE=0; Clear inherited offsets without moving
M400
BALL_VALVE_CLOSED

G28 Z; Home Z upward to Z=47.250
G28 X Y; Home X and Y at Z=47.250
G0 F1500; Initialize base feedrate

G0 X30 F600; Rotate turntable +30 deg at 10 deg/s
M400
G4 P2000; Pause 2.0 s (inspect actual angle ~30 deg)

G0 X0 F600; Return turntable to 0 deg at 10 deg/s
M400
G4 P2000; Pause 2.0 s (inspect mark alignment at 0 deg)

M400
BALL_VALVE_CLOSED
G28 Z; Confirm Z at top endstop
G28 X Y; Re-home X and Y
M400
M220 S100
G0 F1500
