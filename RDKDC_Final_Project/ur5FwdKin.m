%% RDKDC Lab 3 Question 1 - UR5 Forward Kinematics
%
% Written by Aabhas Jain

function gst = ur5FwdKin(q)
% Inputs: q: 6x1 joint space variable vectors (thetas)
% Purpose: computes forward kinematic map of the UR5. All of the base
% twist, gst0, etc are defined inside this function. 
% Outputs: gst: end effector pose, gst (4x4 matrix)
thetas = q;
L0 = 89.2/1000;  % [m]
L1 = 425/1000;   % [m]
L2 = 392/1000;   % [m]
L3 = 109.3/1000; % [m]
L4 = 94.75/1000; % [m]
L5 = 82.5/1000;  % [m]

% Defining variables - see report for work
e1 = [1; 0; 0];
e2 = [0; 1; 0];
e3 = [0; 0; 1];
zeroVector = [0; 0; 0];
w1 = e3; q1 = zeroVector;
w2 = e2; q2 = [0; 0; L0];
w3 = e2; q3 = [L1; 0; L0];
w4 = e2; q4 = [L1 + L2; 0; L0];
w5 = - e3; q5 = [L1 + L2; L3; 0];
w6 = e2; q6 = [L1 + L2; 0; L0 - L4];
p0 = [L1 + L2; L3 + L5; L0-L4]; % this is the zero configuration
R0 = ROTZ(pi)*ROTX(pi/2); % The zero configuration is rotated 
gst0 = [R0, p0; 0, 0, 0, 1];

% calculating gst - PoE
gst = EXPTWIST(w1, q1, thetas(1))*EXPTWIST(w2, q2, thetas(2))...
*EXPTWIST(w3, q3, thetas(3))*EXPTWIST(w4, q4, thetas(4))...
*EXPTWIST(w5, q5, thetas(5))*EXPTWIST(w6, q6, thetas(6))*gst0;
end