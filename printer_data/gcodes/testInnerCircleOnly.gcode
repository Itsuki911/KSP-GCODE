DE3; Code start here 

M220 S500; Increasing speed 500% 

G28 Z; Initial homing 
G28 X Y;


 G0 Z6 Y15 X330; 

;Purge 
 M220 S375; Purging speed 75% 

BALL_VALVE_OPEN 
 G0 X360; #1
 G0 X340;

 G0 X360; #2
 G0 X340;

 G0 X360; #3
 G0 X340;

 G0 X360; #4
 G0 X340;


 G0 X360; #5
 G0 X340;


 G0 X360; #6
 G0 X340;

 G0 X360; #7
 G0 X340;

 G0 X360; #8
 G0 X340;
BALL_VALVE_CLOSED 



;SQUEEZE OUT

M220 S100; Squeezing speed 20% 
 G0 X360 Z3; #1
 G0 X340;

 G0 X360; #2
 G0 X340;

 G0 X360; #3
 G0 X340;


M220 S500; Increasing speed 500% 

G0 Z18;

G0 Y180 X0 Z10; Inner circle Rev 1 
G0 Z5;
G0 X360;
G0 X0; Inner Circle Rev 2
G0 X360;
G0 X0; Inner Circle Rev 2

G0 Z50 Y2;
G28;
