; SIMULATION CANDIDATE ONLY - DO NOT RUN ON HARDWARE WITHOUT CALIBRATION.
; Inner path uses Y131 to place the simulated point at radius 49 mm, halfway
; across the modeled inner ring (r=40..58 mm), assuming r_center=180 mm.
; Ring dimensions, Y origin, workpiece alignment and roller contact are unverified.
; Same Z8 and valve commands as testPaint; actual contact and clearance unknown.
; This file does not establish safe physical coating or verify endstops.
;
; Revised after independent audit, 2026-11-18.
; Candidate for machine validation, NOT a certified collision-free program.
; Coordinates use the configured machine origin with ZERO XYZ G-code offsets.
; Any required fixture/tool offsets must be reconciled before using this file.
; X is degrees; Y/Z are mm. F is in mixed planner units/min, not surface mm/min.
; Each moving block specifies F. M220 stays at 100% unless changed externally.
; Requested speeds preserve the previous candidate: travel/coating 125,
; purge 93.75, squeeze 25; pure Z explicitly requests 20 mm/s.
; Working Z values and proposed travel heights still require physical validation.
; Valve response time is UNMEASURED: no arbitrary G4 delay has been inserted.
; M400 waits for axis motion; it does NOT confirm servo arrival or liquid settling.
; Before wet use, measure opening/closing response and add validated G4 P delays
; at the marked valve transitions. P is in milliseconds.
; File-end commands do NOT run on abort: CANCEL/PAUSE/RESUME and shutdown liquid
; isolation require separate machine configuration/hardware corrections.
; Existing TMC faults, homing switch/retract behavior and clearance remain unresolved.

G21; Set units to millimeters
G90; Absolute positioning mode
M220 S100; Reset speed factor to 100%
SET_GCODE_OFFSET X=0 Y=0 Z=0 MOVE=0; Clear inherited offsets without moving
M400; Finish any previously queued motion before issuing initial closure
BALL_VALVE_CLOSED
; Add measured closing/settling G4 here before homing; currently unmeasured.

G28 Z; Initial Z homing (moves upward to Z=47.250)
G28 X Y; Initial XY homing (moves to X=0, Y=0 at Z=47.250)

G0 F1500; Initialize base feedrate to 1500 mm/min (25 mm/s)

; Move at homed Z, preserving the original purge start angle X285
G0 X285 F7500;
G0 Y15 F7500;
G0 Z19 F1200;

;Purge
; Purge: requested 93.75 deg/s; original 17 nonzero strokes retained

M400; Complete approach and descent before opening
BALL_VALVE_OPEN
; Add measured opening/settling G4 here; currently unmeasured.
G0 X265 F5625; First stroke: X285 -> X265 (no added X265 -> X285 stroke)

G0 X285 F5625; #2
G0 X265 F5625;

G0 X285 F5625; #3
G0 X265 F5625;

G0 X285 F5625; #4
G0 X265 F5625;

G0 X285 F5625; #5
G0 X265 F5625;

G0 X285 F5625; #6
G0 X265 F5625;

G0 X285 F5625; #7
G0 X265 F5625;

G0 X285 F5625; #8
G0 X265 F5625;

G0 X285 F5625; #9
G0 X265 F5625;

M400; Complete the final purge stroke before closing
BALL_VALVE_CLOSED
; Add measured closing/settling G4 here before squeeze-out; currently unmeasured.


;SQUEEZE OUT
; Squeeze: requested 25 deg/s; pure Z limited explicitly to 20 mm/s
G0 Z18 F1200; Descend vertically to squeeze-out height first
G0 X285 F1500; #1
G0 X265 F1500;

G0 X285 F1500; #2
G0 X265 F1500;


; Travel/coating: requested 125 planner units/s, subject to runtime limits

G0 Z24 F1200; Lift to proposed travel height before moving to Circle 1

; Circle 1 Rev 1-2
G0 X39 Y35 F7500; Retain the original Y35 approach waypoint at proposed Z24
G0 Y45 F7500; Complete radial approach at Z24 before descending
G0 Z6 F1200; Descend vertically to working height
G0 X55 F7500;
G0 X39 F7500;
G0 Z11 F1200; Lift vertically before rotating to Circle 2

; Circle 2 Rev 1
G0 X80 F7500; Rotate at proposed travel Z
G0 Z6 F1200; Descend vertically
G0 X96 F7500;
G0 X80 F7500;
G0 Z11 F1200; Lift vertically before rotating to Circle 3

; Circle 3 Rev 1-2
G0 X122 F7500; Rotate at proposed travel Z
G0 Z6 F1200; Descend vertically
G0 X138 F7500;
G0 X122 F7500;
G0 Z11 F1200; Lift vertically before moving to Circle 4

; Circle 4 Rev 1
G0 X217 Y47 F7500; Rotate and shift Y at proposed travel Z
G0 Z6 F1200; Descend vertically
G0 X236 F7500;
G0 X217 F7500;
G0 Z25 F1200; Lift to high clearance Z25 before Circle 5 transit (clearance unverified)

; Circle 5 Rev 1
G0 X301 F7500; Rotate at Z25
G0 Z6 F1200; Descend vertically
G0 X317 F7500;
G0 X301 F7500;
G0 Z25 F1200; Lift to Z25 before moving to Raised Section 2

; Raised Section 2 Rev 1
G0 Y77 X250 F7500; Move to Raised Section 2 XY at Z25
G0 Z16 F1200; Descend vertically
G0 X295 F7500;
G0 X250 F7500;
G0 X295 F7500;
G0 Z21 F1200; Lift vertically before rotating to Raised Section 1

; Raised Section 1 Rev 1
G0 X30 F7500; Rotate at Z21
G0 Z16 F1200; Descend vertically
G0 X95 F7500;
G0 X30 F7500;
G0 X95 F7500;
G0 Z24 F1200; Lift to proposed travel height before moving to Inner Circle

; Inner circle Rev 1-2
G0 Y131 X0 F7500; SIMULATION ONLY: ring center r=49 mm when r_center=180 mm
G0 Z8 F1200; Descend vertically to working height
G0 X360 F7500;
G0 X0 F7500; Inner Circle Rev 2
G0 X360 F7500;
G0 X0 F7500; Inner Circle Rev 2

; Normal completion only: finish coating and explicitly command valve closure
M400
BALL_VALVE_CLOSED
; Add measured closing/settling G4 here; currently unmeasured.
; End-of-cycle return removes the former Z53 command; clearance remains unverified.
G0 Z33 F1200; Lift vertically to proposed Z33
G28 Z; Home Z upward to top endstop (Z=47.250)
G28 X Y; Home X and Y at homed Z (Z=47.250)
M400; Complete homing before normal file completion
M220 S100; Restore speed factor to 100%
G0 F1500; Leave a defined base feed for subsequent commands
