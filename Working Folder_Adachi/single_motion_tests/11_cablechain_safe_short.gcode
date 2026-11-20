; Phase 2 / Test 11: Safe short cable-chain test (safe revision of gcodes/cableChainTest.gcode).
; Key fixes vs. gcodes/cableChainTest.gcode:
;   - Eliminated G0 Z50 (exceeded upper endstop Z=47.250 by +2.75 mm); capped at Z=45.
;   - Eliminated G0 X100 Z0 (diagonal XZ plunge to Z=0); separated X positioning and Z motion,
;     with Z lower test bound set to Z=10 (requires empty table/bed clearance verification).
;   - Added standard modal/offset/valve preamble and clean homing/state-restore epilogue.
;   - Replaced M220 S500 with M220 S100 and explicit per-move F (Y: F1500, Z: F1200).
;   - Reduced from 180 long sets (1440 Y cycles) to a 5-cycle short verification run.
; Preconditions:
;   - Tests 08a/08b, 09a, and 10 passed; NO workpiece installed; liquid isolated.

G21; Set units to millimeters
G90; Absolute positioning mode
M220 S100; Reset speed factor to 100%
SET_GCODE_OFFSET X=0 Y=0 Z=0 MOVE=0; Clear inherited offsets without moving
M400
BALL_VALVE_CLOSED

G28 Z; Home Z upward to Z=47.250
G28 X Y; Home X and Y at Z=47.250
G0 F1500; Initialize base feedrate

; Part A: Y-axis cable-chain oscillation at homed Z=47.250 (5 cycles at 25 mm/s)
G0 Y180 F1500; #1
G0 Y0 F1500
G0 Y180 F1500; #2
G0 Y0 F1500
G0 Y180 F1500; #3
G0 Y0 F1500
G0 Y180 F1500; #4
G0 Y0 F1500
G0 Y180 F1500; #5
G0 Y0 F1500
M400
G4 P1000

; Part B: Position X=100 at top Z, then run Z-only oscillation (Z10 <-> Z45, 5 cycles at 20 mm/s)
G0 X100 F1500; Move X horizontally at Z=47.250 before any Z descent
M400
G0 Z10 F1200; #1
G0 Z45 F1200
G0 Z10 F1200; #2
G0 Z45 F1200
G0 Z10 F1200; #3
G0 Z45 F1200
G0 Z10 F1200; #4
G0 Z45 F1200
G0 Z10 F1200; #5
G0 Z45 F1200

M400
BALL_VALVE_CLOSED
G28 Z; Home Z upward to Z=47.250
G28 X Y; Home X and Y at Z=47.250
M400
M220 S100
G0 F1500
