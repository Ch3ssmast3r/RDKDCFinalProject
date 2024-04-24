%% Part D. Testing getXi
%The goal of this part is to choose some arbitrary homogeneous
%transformations, calculate the twist using the getXi function, and then
%exponentiate the twist to get the transformation that we started with. 

%Pick some arbitrary transformations.
R1 = eye(3); %translational case
p1 = [2;1;0];

R2 = ROTZ(1)*ROTX(pi/2);
p2 = [-1;0;1];

R3 = ROTY(pi/4)*ROTX(pi/2)*ROTZ(1);
p3 = [.5;.5;.5];

%take R and p into homogeneous representation.
g1 = RPtoG(R1,p1);
g2 = RPtoG(R2,p2);
g3 = RPtoG(R3,p3);

%extract the twists using getxi
xi1 = getXi(g1);
xi2 = getXi(g2);
xi3 = getXi(g3);

%extract v and w
v1 = xi1(1:3);
w1 = xi1(4:6);
xiHat1 = [SKEW3(w1),v1;0,0,0,0];

v2 = xi2(1:3);
w2 = xi2(4:6);
xiHat2 = [SKEW3(w2),v2;0,0,0,0];

v3 = xi3(1:3);
w3 = xi3(4:6);
xiHat3 = [SKEW3(w3),v3;0,0,0,0];

%calculate g again from the twists, then see if they match. This will be
%done using the function "XiToG" which takes a twist and finds the
%homogeneous transformation.
gOne = expm(xiHat1);
gTwo = expm(xiHat2);
gThree = expm(xiHat3);

error1 = g1-gOne
error2 = g2-gTwo
error3 = g3-gThree