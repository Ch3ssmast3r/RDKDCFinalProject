%Andrew Palacio

%This script is used to test the function ur5_calculate_lines.m

%create two transformations
g1 = [eye(3),[1;.5;.5];0,0,0,1];
g4 = [eye(3),[1+.1*.707;.5-.1*.707;.75];0,0,0,1];

[g1, g2, g3, g4] = ur5_calculate_lines(g1,g4);

%extract our four points
v1 = g1(1:3,4);
v2 = g2(1:3,4);
v3 = g3(1:3,4);
v4 = g4(1:3,4);

figure (1)
hold on
plotp3(v1)
plotp3(v2)
plotp3(v3)
plotp3(v4)
hold off