%Andrew Palacio

%This script is used to test the function ur5_calculate_lines.m

%create two transformations
g1 = [eye(3),[1;.5;.5];0,0,0,1];
g4 = [ROTX(pi/4),[1+.1*.707;.5-.1*.707;.75];0,0,0,1];

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

%v1-v4 are the four points of the TOOL. let's find the points of the PEN.
R67 = eye(3);
p67 = [0; -49/1000; 115/1000];
t67 = [R67, p67; 0, 0, 0, 1];

% v1pen = t67*[v1;1];
% v2pen = t67*[v2;1];
% v3pen = t67*[v3;1];
% v4pen = t67*[v4;1];

% figure (1)
% hold on
% plotp3(v1,'b')
% plotp3(v2,'b')
% plotp3(v3,'b')
% plotp3(v4,'b')
% plotp3(v1pen(1:3),'r')
% plotp3(v2pen(1:3),'r')
% plotp3(v3pen(1:3),'r')
% plotp3(v4pen(1:3),'r')
% hold off