%% RDKDC Final Project ur5FwdKinDH
% The purpose of this function is to compute the forward kinematics of the
% ur5 using the D-H parameterization to be more accurate.
% Written by Aabhas Jain

function gst = ur5FwdKinDH(q)
% Inputs: q: 6x1 joint space variable vectors (theta)
% Purpose: compute forward kinematic map of the ur5. All parameters defined
% inside the function.
% Outputs: gst: end effector pose, (4x4 matrix) NOTE THAT THIS DOES NOT
% INCLUDE THE PEN TIP.

thetas = q;
% DH parameters - NOTE, THIS IS TAKEN FROM THE ur5InvKin script.
d1 = 0.089159;
d2 = 0;
d3 = 0;
d4 = 0.10915;
d5 = 0.09465;
d6 = 0.0823;
a1 = 0;
a2 = -0.425;
a3 = -0.39225;
a4 = 0;
a5 = 0;
a6 = 0;
alpha1 = pi/2;
alpha2 = 0;
alpha3 = 0;
alpha4 = pi/2;
alpha5 = -pi/2;
alpha6 = 0;

t01 = DH(a1,alpha1, d1, thetas(1) - pi);
t12 = DH(a2, alpha2, d2, thetas(2));
t23 = DH(a3, alpha3, d3, thetas(3));
t34 = DH(a4, alpha4, d4, thetas(4));
t45 = DH(a5, alpha5, d5, thetas(5));
t56 = DH(a6, alpha6, d6, thetas(6));
R67 = eye(3);
p67 = [0; -49/1000; 122.28/1000];
t67 = [R67, p67; 0, 0, 0, 1];
gst = t01 * t12 * t23 * t34 * t45 * t56 * t67;
end