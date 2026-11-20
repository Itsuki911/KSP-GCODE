; Phase 2 / Test 12: Safe short turntable/XY break-in (safe revision of gcodes/turntableBreakin.gcode).
; Key fixes vs. gcodes/turntableBreakin.gcode:
;   - Added standard modal/offset/valve preamble and clean homing/state-restore epilogue.
;   - Replaced M220 S500 + inherited feedrate with M220 S100 + explicit F1500 per move.
;   - Reduced from 841 continuous cycles to a 5-cycle low-speed verification run.
;   - Explicitly keeps Z parked at top endstop (Z=47.250) throughout.
; Preconditions:
;   - Single-axis tests 09a and 10 passed; NO workpiece installed; liquid isolated.
;   - Note: Simultaneous X+Y motion traces a spiral in physical polar coordinates;
;     confirm hose/cable routing clears the full swept envelope.

G21; Set units to millimeters
G90; Absolute positioning mode
M220 S100; Reset speed factor to 100%
SET_GCODE_OFFSET X=0 Y=0 Z=0 MOVE=0; Clear inherited offsets without moving
M400
BALL_VALVE_CLOSED

G28 Z; Home Z upward to Z=47.250
G28 X Y; Home X and Y at Z=47.250
G0 F1500; Initialize base feedrate

; 5 low-speed XY cycles at Z=47.250 (no Z motion)
G0 X0 Y180 F1500; #1
G0 X360 Y0 F1500

G0 X0 Y180 F1500; #2
G0 X360 Y0 F1500

G0 X0 Y180 F1500; #3
G0 X360 Y0 F1500

G0 X0 Y180 F1500; #4
G0 X360 Y0 F1500

G0 X0 Y180 F1500; #5
G0 X360 Y0 F1500

M400
BALL_VALVE_CLOSED
G28 Z; Home Z upward to Z=47.250
G28 X Y; Home X and Y at Z=47.250
M400
M220 S100
G0 F1500
