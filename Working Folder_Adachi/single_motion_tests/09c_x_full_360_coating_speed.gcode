; Phase 2 / Test 09c: X-axis (turntable) full 360 deg + reversal check at coating speed (F7500 = 125 deg/s).
; Preconditions:
;   - Tests 09a and 09b passed with zero skipped steps and acceptable vibration.
;   - No workpiece installed; liquid isolated.
; Note:
;   - 125 deg/s is high angular speed; at outer radius r=135 mm, linear surface speed is
;     ~294.5 mm/s and tangential acceleration at max_accel=1000 deg/s^2 is ~2356 mm/s^2.
;     Observe closely for skipped steps, ringing, or mechanical resonance.
; Post-check:
;   - Verify 0 deg tape mark alignment before re-homing and run DUMP_TMC STEPPER=stepper_x.

G21; Set units to millimeters
G90; Absolute positioning mode
M220 S100; Reset speed factor to 100%
SET_GCODE_OFFSET X=0 Y=0 Z=0 MOVE=0; Clear inherited offsets without moving
M400
BALL_VALVE_CLOSED

G28 Z; Home Z upward to Z=47.250
G28 X Y; Home X and Y at Z=47.250
G0 F1500; Initialize base feedrate

G0 X360 F7500; Full forward revolution at 125 deg/s
M400
G4 P2000; Inspect 360 deg mark

G0 X0 F7500; Full reverse revolution at 125 deg/s
M400
G4 P2000; Inspect 0 deg mark

; Short reversal check at coating speed F7500 (3 cycles)
G0 X285 F7500
G0 X265 F7500; #1
G0 X285 F7500
G0 X265 F7500; #2
G0 X285 F7500
G0 X265 F7500; #3
G0 X0 F7500
M400
G4 P2000; Inspect final 0 deg mark before re-homing

M400
BALL_VALVE_CLOSED
G28 Z
G28 X Y
M400
M220 S100
G0 F1500
