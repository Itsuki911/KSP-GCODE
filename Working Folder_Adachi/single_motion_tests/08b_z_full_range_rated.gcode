; Phase 2 / Test 08b: Z-axis full-range staircase at rated speed (F1200 = 20 mm/s).
; Preconditions:
;   - Test 08a passed with zero mechanical binding and zero TMC faults.
;   - NO workpiece or fixture under the applicator; liquid isolated.
; Scope:
;   - Repeats the Test 08a Z-only staircase at the configured max_z_velocity
;     (F1200 = 20 mm/s) used in Working Folder_Adachi/testPaint.gcode.
; Post-check:
;   - Confirm no skipped steps, no audible stall, and run DUMP_TMC STEPPER=stepper_z.

G21; Set units to millimeters
G90; Absolute positioning mode
M220 S100; Reset speed factor to 100%
SET_GCODE_OFFSET X=0 Y=0 Z=0 MOVE=0; Clear inherited offsets without moving
M400
BALL_VALVE_CLOSED

G28 Z; Home Z upward to Z=47.250
G0 F1500; Initialize base feedrate

G0 Z33 F1200; Proposed end-of-cycle lift Z (20 mm/s)
M400
G4 P1500

G0 Z24 F1200; Proposed travel Z
M400
G4 P1500

G0 Z19 F1200; Purge working Z
M400
G4 P1500

G0 Z18 F1200; Squeeze working Z
M400
G4 P1500

G0 Z16 F1200; Raised section working Z
M400
G4 P1500

G0 Z11 F1200; Circle transit Z
M400
G4 P1500

G0 Z8 F1200; Inner circle working Z
M400
G4 P1500

G0 Z6 F1200; Circle 1-5 working Z
M400
G4 P3000; Hold 3.0 s at lowest Z

G0 Z24 F1200; Ascend to intermediate height
M400
G4 P1000

G0 Z47.25 F1200; Return to top endstop coordinate
M400
G4 P1000

M400
BALL_VALVE_CLOSED
G28 Z; Re-home Z
M400
M220 S100
G0 F1500
