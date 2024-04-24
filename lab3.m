%% Part D. Testing getXi
%The goal of this part is to choose some arbitrary homogeneous
%transformations, calculate the twist using the getXi function, and then
%exponentiate the twist to get the transformation that we started with. 

%Pick some arbitrary transformations.
R1 = eye(3); %translational case
p1 = [2;1;0];

R2 = ROTZ(pi)*ROTY(.5);
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

%calculate g again from the twists, then see if they match. This will be
%done using the function "XiToG" which takes a twist and finds the
%homogeneous transformation.
gOne = XiToG(xi1);
gTwo = XiToG(xi2);
gThree = XiToG(xi3);