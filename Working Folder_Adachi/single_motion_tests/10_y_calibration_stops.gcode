; Phase 2 / Test 10: Y-axis (radial) full-range calibration stops at F1500 (25 mm/s).
; Preconditions:
;   - Phase 1 passed; no workpiece installed; liquid isolated.
;   - Prepare ruler/caliper to measure actual physical radius r(Y) from rotation center.
; Scope:
;   - Keeps Z at top endstop (47.250) and X at 0.
;   - Visits every Y coordinate used by Working Folder_Adachi/testPaint.gcode
;     (15, 35, 45, 47, 77, 180) and returns to 0 with 4 s pauses for physical measurement.
; Post-check:
;   - Record actual r(Y) at each stop, confirm whether Y=180 aligns with turntable center,
;     verify cable chain movement, and run DUMP_TMC STEPPER=stepper_y.

G21; Set units to millimeters
G90; Absolute positioning mode
M220 S100; Reset speed factor to 100%
SET_GCODE_OFFSET X=0 Y=0 Z=0 MOVE=0; Clear inherited offsets without moving
M400
BALL_VALVE_CLOSED

G28 Z; Home Z upward to Z=47.250
G28 X Y; Home X and Y at Z=47.250
G0 F1500; Initialize base feedrate

G0 Y15 F1500; Purge / squeeze Y coordinate
M400
G4 P4000; Measure physical position at Y=15

G0 Y35 F1500; Circle 1 approach waypoint Y
M400
G4 P4000; Measure physical position at Y=35

G0 Y45 F1500; Circle 1-3 Y coordinate
M400
G4 P4000; Measure physical position at Y=45

G0 Y47 F1500; Circle 4-5 Y coordinate
M400
G4 P4000; Measure physical position at Y=47

G0 Y77 F1500; Raised Section 1-2 Y coordinate
M400
G4 P4000; Measure physical position at Y=77

G0 Y180 F1500; Inner Circle Y coordinate (check alignment with turntable rotation center)
M400
G4 P4000; Measure physical position at Y=180

G0 Y0 F1500; Return to Y=0
M400
G4 P2000; Check return mark at Y=0 before re-homing

M400
BALL_VALVE_CLOSED
G28 Z
G28 X Y
M400
M220 S100
G0 F1500
