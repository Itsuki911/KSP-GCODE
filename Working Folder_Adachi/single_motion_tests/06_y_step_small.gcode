; Phase 1 / Test 06: Y-axis (radial) minimal stroke test (20 mm at low speed F600).
; Preconditions:
;   - Tests 01-04 passed; no workpiece installed; liquid physically isolated.
;   - Mark Y=0 carriage position with tape or prepare a caliper/ruler.
; Scope:
;   - Keeps Z at top endstop (47.250) and X at 0.
;   - Moves Y from 0 to 20 mm at F600 (10 mm/s), pauses 2 s, returns to 0 mm.
; Post-check:
;   - Measure actual Y displacement (~20 mm) and return accuracy to Y=0.

G21; Set units to millimeters
G90; Absolute positioning mode
M220 S100; Reset speed factor to 100%
SET_GCODE_OFFSET X=0 Y=0 Z=0 MOVE=0; Clear inherited offsets without moving
M400
BALL_VALVE_CLOSED

G28 Z; Home Z upward to Z=47.250
G28 X Y; Home X and Y at Z=47.250
G0 F1500; Initialize base feedrate

G0 Y20 F600; Move Y +20 mm at 10 mm/s
M400
G4 P2000; Pause 2.0 s (measure 20 mm travel)

G0 Y0 F600; Return Y to 0 mm at 10 mm/s
M400
G4 P2000; Pause 2.0 s (check return mark at Y=0)

M400
BALL_VALVE_CLOSED
G28 Z; Confirm Z at top endstop
G28 X Y; Re-home X and Y
M400
M220 S100
G0 F1500
