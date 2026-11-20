; Phase 2 / Test 08a: Z-axis full-range staircase at slow speed (F600 = 10 mm/s).
; Preconditions:
;   - Phase 1 passed; NO workpiece or fixture under the applicator; liquid isolated.
;   - Confirm physical bottom clearance allows Z=6.000 without contacting the bed/table.
; Scope:
;   - Homes Z only (no X/Y motion). Steps down through all key Z heights used by
;     Working Folder_Adachi/testPaint.gcode (33, 24, 19, 18, 16, 11, 8, 6) and returns
;     to Z=47.250 with 2 s holds at each stop to test mechanical travel, holding torque,
;     and TMC2209 stability (s2vsa / drv_err isolation).
; Post-check:
;   - Run DUMP_TMC STEPPER=stepper_z and inspect motor temperature / klippy.log.

G21; Set units to millimeters
G90; Absolute positioning mode
M220 S100; Reset speed factor to 100%
SET_GCODE_OFFSET X=0 Y=0 Z=0 MOVE=0; Clear inherited offsets without moving
M400
BALL_VALVE_CLOSED

G28 Z; Home Z upward to Z=47.250
G0 F1500; Initialize base feedrate

G0 Z33 F600; Proposed end-of-cycle lift Z
M400
G4 P2000

G0 Z24 F600; Proposed travel Z
M400
G4 P2000

G0 Z19 F600; Purge working Z
M400
G4 P2000

G0 Z18 F600; Squeeze working Z
M400
G4 P2000

G0 Z16 F600; Raised section working Z
M400
G4 P2000

G0 Z11 F600; Circle transit Z
M400
G4 P2000

G0 Z8 F600; Inner circle working Z
M400
G4 P2000

G0 Z6 F600; Circle 1-5 working Z (lowest commanded testPaint Z)
M400
G4 P3000; Hold 3.0 s at lowest Z (check sag / motor noise / TMC status)

G0 Z24 F600; Ascend to intermediate height
M400
G4 P1000

G0 Z47.25 F600; Return to top endstop coordinate
M400
G4 P1000

M400
BALL_VALVE_CLOSED
G28 Z; Re-home Z
M400
M220 S100
G0 F1500
