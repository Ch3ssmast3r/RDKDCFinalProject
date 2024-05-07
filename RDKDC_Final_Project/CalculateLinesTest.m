%Andrew Palacio

%This script is used to test the function ur5_calculate_lines.m

%create two transformations
g1 = [eye(3),[1;.5;.6];0,0,0,1];
g4 = [ROTX(pi/4),[1+.25*.707;.25-.1*.707;.6];0,0,0,1];

frames = ur5_calculate_lines(g1,g4);

%extract the four frames
g1 = frames(:,:,1);
g2 = frames(:,:,2);
g3 = frames(:,:,3);
g4 = frames(:,:,4);

%extract our four points
v1 = g1(1:3,4);
v2 = g2(1:3,4);
v3 = g3(1:3,4);
v4 = g4(1:3,4);

