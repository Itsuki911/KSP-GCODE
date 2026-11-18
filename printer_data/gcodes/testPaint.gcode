; Code start here 


G28 Z; Initial homing 
G28 X Y;


M220 S100; Increasing speed 500% 

G0 X285;
G0 Z19 Y15;
 


M220 S100;
;Purge 
 M220 S375; Purging speed 75% 

 BALL_VALVE_OPEN 
 G0 X285; #1
 G0 X265;

 G0 X285; #2
 G0 X265;

 G0 X285; #3
 G0 X265;

 G0 X285; #4
 G0 X265;

 G0 X285; #5
 G0 X265;

 G0 X285; #6
 G0 X265;

 G0 X285; #7
 G0 X265;

 G0 X285; #8
 G0 X265;

 G0 X285; #9
 G0 X265;

BALL_VALVE_CLOSED 



;SQUEEZE OUT

M220 S100; Squeezing speed 20% 
G0 X285 Z18; #1
G0 X265;

G0 X285; #2
G0 X265;

 


M220 S10; Increasing speed 500% 

G0 Z20;

; Circle 1 Rev 1-2
G0 X39 Y35;
G0 Z6 Y45
G0 X55;
G0 X39; 
G0 Z10;

; Circle 2 Rev 1 

G0 X80 Z6; 
G0 X96;
G0 X80;
G0 Z10; 

; Circle 3 Rev 1-2

G0 X122 Z6; 
G0 X138;
G0 X122;
G0 Z10 Y47;


; Circle 4 Rev 1 

G0 X217 Z6; 
G0 X236;
G0 X217;
G0 Z10; 

G0 Z25;
; Circle 5 Rev 1 
G0 X301;
G0 Z6;
G0 X317;
G0 X301;
G0 Z23; 

G0 Y77 X250; Raised Section 2 Rev 1
G0 Z16;
G0 X295;
G0 X250;
G0 X295;
G0 Z21;

G0 X30; Raised Section 1 Rev 1 
G0 Z16;
G0 X95;
G0 X30;
G0 X95;
G0 Z20;

G0 Y180 X0 Z11; Inner circle Rev 1 
G0 Z8;
G0 X360;
G0 X0; Inner Circle Rev 2
G0 X360;
G0 X0; Inner Circle Rev 2

G0 Z53 Y2;
G28;
