; Phase 1 / Test 02: Z-first followed by X and Y homing only.
; Preconditions:
;   - Test 01 passed with no TMC faults.
;   - Liquid supply physically isolated; no workpiece installed.
;   - Keep clear of X turntable and Y radial rail; ready physical E-stop / M112.
; Scope:
;   - Initializes modal state, closes valve, homes Z first, then homes X and Y.
;   - No subsequent travel moves.
; Post-check:
;   - Confirm Z=47.250, X=0, Y=0 stopped cleanly on their respective switches.
;   - Run DUMP_TMC on stepper_x, stepper_y, stepper_z.

G21; Set units to millimeters
G90; Absolute positioning mode
M220 S100; Reset speed factor to 100%
SET_GCODE_OFFSET X=0 Y=0 Z=0 MOVE=0; Clear inherited offsets without moving
M400; Finish any queued motion before valve command
BALL_VALVE_CLOSED
; Add measured closing/settling G4 here once calibrated in Test 03.

G28 Z; Home Z upward to top endstop (Z=47.250)
G28 X Y; Home X (0 deg) and Y (0 mm) at homed Z
M400; Wait for homing completion

M220 S100; Leave speed factor at 100%
G0 F1500; Leave defined base feedrate (no axis movement)
