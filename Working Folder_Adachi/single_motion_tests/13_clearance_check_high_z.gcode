; Phase 3 / Test 13: High-Z static clearance check over installed workpiece (dry, low speed).
; Preconditions:
;   - All Phase 1 and Phase 2 tests passed with zero TMC faults and verified axis scaling.
;   - Liquid supply physically isolated; valve remains CLOSED throughout.
;   - Workpiece and fixtures installed; visually confirm Z=33, Z=25, and Z=24 clear all
;     highest workpiece/clamp points BEFORE running; keep hand on physical E-stop.
; Scope:
;   - Visits every feature XY location from Working Folder_Adachi/testPaint.gcode strictly
;     at high travel heights (Z=33, Z=25, Z=24) at low speed (XY F1500, Z F600) with
;     4-second visual inspection pauses. NEVER descends to working Z (19, 18, 16, 8, 6).
;   - Strictly separates vertical Z moves and horizontal XY moves.
; Post-check:
;   - Record actual visual/measured clearance at every waypoint before attempting any
;     lower travel Z or working Z descent in Working Folder_Adachi/testPaint.gcode.

G21; Set units to millimeters
G90; Absolute positioning mode
M220 S100; Reset speed factor to 100%
SET_GCODE_OFFSET X=0 Y=0 Z=0 MOVE=0; Clear inherited offsets without moving
M400
BALL_VALVE_CLOSED

G28 Z; Home Z upward to Z=47.250
G28 X Y; Home X and Y at Z=47.250
G0 F1500; Initialize base feedrate

; --- Purge station XY at Z=47.250 -> check Z=33 and Z=24 ---
G0 X285 F1500; Rotate to Purge start X at top Z
G0 Y15 F1500; Move to Purge Y at top Z
G0 Z33 F600; Descend vertically to high clearance Z=33
M400
G4 P4000; Inspect clearance at Purge (X285, Y15, Z33)

G0 Z24 F600; Descend vertically to proposed travel height Z=24
M400
G4 P4000; Inspect clearance at Purge (X285, Y15, Z24)

; --- Circle 1-4 waypoints at Z=24 ---
G0 X39 Y35 F1500; Move to Circle 1 approach waypoint at Z=24
M400
G4 P3000; Inspect Circle 1 approach waypoint (X39, Y35, Z24)

G0 Y45 F1500; Radial approach to Circle 1 at Z=24
M400
G4 P4000; Inspect clearance over Circle 1 (X39, Y45, Z24)

G0 X80 F1500; Rotate to Circle 2 at Z=24
M400
G4 P3000; Inspect clearance over Circle 2 (X80, Y45, Z24)

G0 X122 F1500; Rotate to Circle 3 at Z=24
M400
G4 P3000; Inspect clearance over Circle 3 (X122, Y45, Z24)

G0 X217 Y47 F1500; Move to Circle 4 at Z=24
M400
G4 P3000; Inspect clearance over Circle 4 (X217, Y47, Z24)

; --- Circle 5 and Raised Sections at Z=25 ---
G0 Z25 F600; Lift vertically to proposed Z=25 before Circle 5 / Raised transit
M400

G0 X301 F1500; Rotate to Circle 5 at Z=25
M400
G4 P3000; Inspect clearance over Circle 5 (X301, Y47, Z25)

G0 Y77 X250 F1500; Move to Raised Section 2 start at Z=25
M400
G4 P4000; Inspect clearance over Raised Section 2 (X250, Y77, Z25)

G0 X30 F1500; Sweep negative across workpiece to Raised Section 1 at Z=25 (note: 220 deg sweep)
M400
G4 P4000; Inspect clearance over Raised Section 1 (X30, Y77, Z25)

; --- Inner Circle at Z=24 ---
G0 Z24 F600; Step vertically to proposed Inner transit height Z=24
M400

G0 Y180 X0 F1500; Move to Inner Circle start at Z=24
M400
G4 P4000; Inspect clearance over Inner Circle (X0, Y180, Z24)

; --- Normal completion ---
G0 Z33 F600; Lift vertically to Z=33 before homing
M400
BALL_VALVE_CLOSED
G28 Z; Home Z upward to Z=47.250
G28 X Y; Home X and Y at Z=47.250
M400
M220 S100
G0 F1500
