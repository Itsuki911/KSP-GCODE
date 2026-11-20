; Phase 1 / Test 04: Z-axis minimal stroke test (5.25 mm at low speed F300).
; Preconditions:
;   - Tests 01-03 passed; no workpiece installed; liquid physically isolated.
; Scope:
;   - Homes Z only (no X/Y motion), descends 5.25 mm from Z=47.250 to Z=42.000
;     at F300 (5 mm/s), holds 2 s to check holding torque/sag, returns to Z=47.250,
;     then re-homes Z.
; Post-check:
;   - Measure actual Z travel (~5.25 mm), verify no drop/slip during hold,
;     and run DUMP_TMC STEPPER=stepper_z for s2vsa / drv_err.

G21; Set units to millimeters
G90; Absolute positioning mode
M220 S100; Reset speed factor to 100%
SET_GCODE_OFFSET X=0 Y=0 Z=0 MOVE=0; Clear inherited offsets without moving
M400
BALL_VALVE_CLOSED

G28 Z; Home Z upward to Z=47.250
G0 F1500; Initialize base feedrate

G0 Z42 F300; Descend 5.25 mm at 5 mm/s
M400
G4 P2000; Hold 2.0 s at Z=42 (check holding stability and motor sound)

G0 Z47.25 F300; Return upward to Z=47.250 at 5 mm/s
M400
G4 P1000; Pause 1.0 s before re-homing

M400
BALL_VALVE_CLOSED
G28 Z; Re-home Z to confirm repeatability
M400
M220 S100
G0 F1500
