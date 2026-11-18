; Code start here 


G28 Z; Initial homing 
G28 X Y;



M220 S500;
G0 X285;
G0 Z12 Y15;
 


;Purge 
 M220 S375; Purging speed 75% 

 BALL_VALVE_OPEN 
 G0 X285; #1
 G0 X265;

 G0 X285; #2
 G0 X265;

 G0 X285; #3
 G0 X265;
 G0 X285; #1
 G0 X265;

 G0 X285; #2
 G0 X265;

 G0 X285; #3
 G0 X265;

BALL_VALVE_CLOSED 



;SQUEEZE OUT

M220 S100; Squeezing speed 20% 
G0 X285 Z10; #1
G0 X265;

G0 X285; #2
G0 X265;


M220 S500; Increasing speed 500% 

G0 Z24;

G0 Y180 X0 Z11; Inner circle Rev 1 
G0 Z5;
G0 X360;
G0 X0; Inner Circle Rev 2
G0 X360;
G0 X0; Inner Circle Rev 2
G0 Z33;

G0 Y77 X30; Raised Section 1 Rev 1 
G0 Z13;
G0 X95;
G0 X30;
G0 X95;
G0 Z21;


G0 X250; Raised Section 2 Rev 1
G0 Z13;
G0 X295;
G0 X250;
G0 X295;
G0 Z26;




; Circle 5 Rev 1 
G0 X301 Y47;
G0 Z3;
G0 X317;
G0 X301;
G0 Z26; 

; Circle 4 Rev 1 

G0 X217;
G0 Z3;
G0 X236;
G0 X217;
G0 Z11; 

; Circle 3 Rev 1-2

G0 X122 Z4 Y45; 
G0 X138;
G0 X122;
G0 Z11;

; Circle 2 Rev 1 

G0 X80 Z4; 
G0 X96;
G0 X80;
G0 Z11; 


; Circle 1 Rev 1-2
G0 X39;
G0 Z3;
G0 X55;
G0 X39; 
G0 Z11;









G0 Z53 Y2;
G28;
