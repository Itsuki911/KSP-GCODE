; Test 14: isolated 7-degree purge motion; SIMULATION ONLY.
; Unverified geometry, Y origin, roller contact and clearances: do not run on hardware.
; Dry motion only: valve stays closed; no liquid dispensing is tested.
; Assumes X is turntable degrees, Y15 is radius 165 mm, and slope X277..285.
; Nine cycles at candidate feedrates; deliberately combined X/Z on the slope.
; Original modeled purge pad remains flat; animation does not validate contact.
G21
G90
M220 S100
SET_GCODE_OFFSET X=0 Y=0 Z=0 MOVE=0
M400
BALL_VALVE_CLOSED
G28 Z
G28 X Y
G0 F1500
G0 X285 F7500
G0 Y15 F7500
G0 Z19 F1200

; Purge round trip 1
G0 X277 Z16.1712 F3000; descend the slope
G0 X265 F5625; flat section
G0 X277 F5625; flat section
G0 X285 Z19 F3000; ascend the slope
; Purge round trip 2
G0 X277 Z16.1712 F3000; descend the slope
G0 X265 F5625; flat section
G0 X277 F5625; flat section
G0 X285 Z19 F3000; ascend the slope
; Purge round trip 3
G0 X277 Z16.1712 F3000; descend the slope
G0 X265 F5625; flat section
G0 X277 F5625; flat section
G0 X285 Z19 F3000; ascend the slope
; Purge round trip 4
G0 X277 Z16.1712 F3000; descend the slope
G0 X265 F5625; flat section
G0 X277 F5625; flat section
G0 X285 Z19 F3000; ascend the slope
; Purge round trip 5
G0 X277 Z16.1712 F3000; descend the slope
G0 X265 F5625; flat section
G0 X277 F5625; flat section
G0 X285 Z19 F3000; ascend the slope
; Purge round trip 6
G0 X277 Z16.1712 F3000; descend the slope
G0 X265 F5625; flat section
G0 X277 F5625; flat section
G0 X285 Z19 F3000; ascend the slope
; Purge round trip 7
G0 X277 Z16.1712 F3000; descend the slope
G0 X265 F5625; flat section
G0 X277 F5625; flat section
G0 X285 Z19 F3000; ascend the slope
; Purge round trip 8
G0 X277 Z16.1712 F3000; descend the slope
G0 X265 F5625; flat section
G0 X277 F5625; flat section
G0 X285 Z19 F3000; ascend the slope
; Purge round trip 9
G0 X277 Z16.1712 F3000; descend the slope
G0 X265 F5625; flat section
G0 X277 F5625; flat section
G0 X285 Z19 F3000; ascend the slope

M400
BALL_VALVE_CLOSED
G0 Z24 F1200
G28 Z
G28 X Y
M400
M220 S100
G0 F1500
