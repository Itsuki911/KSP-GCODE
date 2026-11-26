; Phase 0: manual endstop input check. No axis, homing, or valve commands.
; Read the Mainsail console. For each prompt, change only the named sensor.
; Hold the requested state until its result is printed. Each prompt has 15 seconds.
; Start with all sensors released/unblocked. Stop and investigate any mismatch.
; This test reports states; it does not automatically judge pass/fail.

M400 ; Wait for any previously queued motion to finish
M118 ENDSTOP TEST: release/unblock ALL sensors and keep them released for baseline (15 s)
G4 P15000
M400
QUERY_ENDSTOPS

M118 ENDSTOP TEST: BLOCK X sensor; keep other sensors released (15 s)
G4 P15000
M400
QUERY_ENDSTOPS
; Expected: x:TRIGGERED; other axes open.

M118 ENDSTOP TEST: RELEASE X sensor; keep other sensors released (15 s)
G4 P15000
M400
QUERY_ENDSTOPS
; Expected: x:open; other axes open.

M118 ENDSTOP TEST: BLOCK Y sensor; keep other sensors released (15 s)
G4 P15000
M400
QUERY_ENDSTOPS
; Expected: y:TRIGGERED; other axes open.

M118 ENDSTOP TEST: RELEASE Y sensor; keep other sensors released (15 s)
G4 P15000
M400
QUERY_ENDSTOPS
; Expected: y:open; other axes open.

M118 ENDSTOP TEST: BLOCK Z sensor; keep other sensors released (15 s)
G4 P15000
M400
QUERY_ENDSTOPS
; Expected: z:TRIGGERED; other axes open.

M118 ENDSTOP TEST: RELEASE Z sensor; keep other sensors released (15 s)
G4 P15000
M400
QUERY_ENDSTOPS
; Expected: z:open; other axes open.

M118 ENDSTOP TEST complete: review all seven readings in the console; no automatic PASS is provided.
