; Phase 1 / Test 01: Z-axis homing only (safe revision of gcodes/PaintFile.gcode).
; Preconditions:
;   - Phase 0 checks passed (hard E-stop verified, QUERY_ENDSTOPS, DUMP_TMC, STEPPER_BUZZ).
;   - Liquid supply physically isolated (dry test only).
;   - Area above Z toolhead clear up to upper endstop (Z=47.250).
;   - Do NOT use default Mainsail PAUSE/CANCEL_PRINT to stop; use physical E-stop or M112.
; Scope:
;   - Initializes modal state and closes valve, then homes Z upward only.
;   - No X or Y motion; no G0/G1 axis travel.
; Post-check:
;   - Confirm upward motion stopped cleanly on the upper switch.
;   - Run DUMP_TMC STEPPER=stepper_z and inspect klippy.log for s2vsa / drv_err.

G21; Set units to millimeters
G90; Absolute positioning mode
M220 S100; Reset speed factor to 100%
SET_GCODE_OFFSET X=0 Y=0 Z=0 MOVE=0; Clear inherited offsets without moving
M400; Finish any queued motion before valve command
BALL_VALVE_CLOSED
; Add measured closing/settling G4 here once calibrated in Test 03.

G28 Z; Home Z upward to top endstop (Z=47.250)
M400; Wait for homing completion

M220 S100; Leave speed factor at 100%
G0 F1500; Leave defined base feedrate (no axis movement)
