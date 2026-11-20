; Phase 1 / Test 03: Ball valve static actuation and response-time measurement.
; Preconditions:
;   - Liquid supply MUST be physically isolated/drained (dry test only).
;   - No axis motion is performed; axes do not need to move.
; Scope:
;   - Executes 3 static OPEN (45 deg) / CLOSED (0 deg) cycles with 3-second dwells
;     to visually verify mechanical closure, servo travel, and measure settling time
;     needed for G4 P... delays in Working Folder_Adachi/testPaint.gcode.
; Post-check:
;   - Record actual opening and closing times (ms) and confirm zero mechanical binding.
;   - Separately verify fail-safe valve state on M112 / MCU power loss (PWM=0).

G21; Set units to millimeters
G90; Absolute positioning mode
M220 S100; Reset speed factor to 100%
SET_GCODE_OFFSET X=0 Y=0 Z=0 MOVE=0; Clear inherited offsets without moving
M400; Barrier before initial closure
BALL_VALVE_CLOSED
G4 P3000; Hold closed 3.0 s (verify baseline closed position)

; Cycle 1
M400
BALL_VALVE_OPEN
G4 P3000; Hold open 3.0 s (measure opening time and check 45 deg stop)
M400
BALL_VALVE_CLOSED
G4 P3000; Hold closed 3.0 s (measure closing time and confirm full shutoff)

; Cycle 2
M400
BALL_VALVE_OPEN
G4 P3000; Hold open 3.0 s
M400
BALL_VALVE_CLOSED
G4 P3000; Hold closed 3.0 s

; Cycle 3
M400
BALL_VALVE_OPEN
G4 P3000; Hold open 3.0 s
M400
BALL_VALVE_CLOSED
G4 P3000; Hold closed 3.0 s

M400; Ensure final dwell completes cleanly
M220 S100; Leave speed factor at 100%
G0 F1500; Leave defined base feedrate (no axis movement)
