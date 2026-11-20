; Phase 2 / Test 07: Homing repeatability check across Z, X, and Y (3 cycles each).
; Preconditions:
;   - All Phase 1 tests (01-06) passed; no workpiece installed; liquid isolated.
;   - Place physical reference marks / dial indicator / caliper at Z, X=0, Y=0.
; Scope:
;   - Because printer.cfg sets homing_retract_dist: 0, Klipper skips the second-pass
;     retract check. This test moves each axis away by a small offset and re-homes
;     3 times individually, pausing 3 s after each home so repeatability can be measured.
; Post-check:
;   - Record scatter (mm or deg) across the 3 homes on each axis.
;   - Run DUMP_TMC on all three steppers and inspect klippy.log.

G21; Set units to millimeters
G90; Absolute positioning mode
M220 S100; Reset speed factor to 100%
SET_GCODE_OFFSET X=0 Y=0 Z=0 MOVE=0; Clear inherited offsets without moving
M400
BALL_VALVE_CLOSED

G28 Z; Initial Z home (Z=47.250)
G28 X Y; Initial X/Y home (X=0, Y=0)
G0 F1500; Initialize base feedrate

; --- Z repeatability (3 cycles: Z=47.25 -> Z=33 -> G28 Z) ---
G0 Z33 F600; Move Z away 14.25 mm at 10 mm/s
M400
G28 Z; Z re-home #1
M400
G4 P3000; Pause 3.0 s (measure Z home #1)

G0 Z33 F600
M400
G28 Z; Z re-home #2
M400
G4 P3000; Pause 3.0 s (measure Z home #2)

G0 Z33 F600
M400
G28 Z; Z re-home #3
M400
G4 P3000; Pause 3.0 s (measure Z home #3)

; --- X repeatability at top Z (3 cycles: X=0 -> X=30 -> G28 X) ---
G0 X30 F1500; Move X away 30 deg at 25 deg/s
M400
G28 X; X re-home #1
M400
G4 P3000; Pause 3.0 s (measure X home #1)

G0 X30 F1500
M400
G28 X; X re-home #2
M400
G4 P3000; Pause 3.0 s (measure X home #2)

G0 X30 F1500
M400
G28 X; X re-home #3
M400
G4 P3000; Pause 3.0 s (measure X home #3)

; --- Y repeatability at top Z (3 cycles: Y=0 -> Y=30 -> G28 Y) ---
G0 Y30 F1500; Move Y away 30 mm at 25 mm/s
M400
G28 Y; Y re-home #1
M400
G4 P3000; Pause 3.0 s (measure Y home #1)

G0 Y30 F1500
M400
G28 Y; Y re-home #2
M400
G4 P3000; Pause 3.0 s (measure Y home #2)

G0 Y30 F1500
M400
G28 Y; Y re-home #3
M400
G4 P3000; Pause 3.0 s (measure Y home #3)

M400
BALL_VALVE_CLOSED
G28 Z
G28 X Y
M400
M220 S100
G0 F1500
