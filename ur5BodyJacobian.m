function Jb = ur5BodyJacobian(q)
%Function accepts q: 6 × 1 joint space variable vector = [θ1, θ2, θ3, θ4,
%θ5, θ6]' where θn is the angle of joint n for n = 1, · · · , 6.
%Returns J: Body Jacobian, Jb_st (6 × 6 matrix)

%Author:James Kaluna

thetas = q;

%define xi's
L0 = 89.2/1000;  % [m]
L1 = 425/1000;   % [m]
L2 = 392/1000;   % [m]
L3 = 109.3/1000; % [m]
L4 = 94.75/1000; % [m]
L5 = 82.5/1000;  % [m]

e1 = [1; 0; 0];
e2 = [0; 1; 0];
e3 = [0; 0; 1];
zeroVector = [0; 0; 0];
w1 = e3; q1 = zeroVector;
w2 = e2; q2 = L0*e3;
w3 = e2; q3 = (L0 + L1)*e3;
w4  = e2; q4 = (L0 + L1 + L2)*e3;
w5 = e3; q5 = L3*e2;
w6 = e2; q6 = (L0 + L1 + L2 + L4)*e3;

p0 = [0; L3 + L5; L0 + L1 + L2 + L4];
R0 = ROTX(-pi/2);
gst0 = [R0, p0; 0, 0, 0, 1];

xi1 = [cross(-w1,q1);w1];
xi2 = [cross(-w2,q2);w2];
xi3 = [cross(-w3,q3);w3];
xi4 = [cross(-w4,q4);w4];
xi5 = [cross(-w5,q5);w5];
xi6 = [cross(-w6,q6);w6];

w_matrix = [w1, w2, w3, w4, w5, w6]; %matrix with all the omegas
q_matrix = [q1, q2, q3, q4, q5, q6]; %matrix with all the points q (not the angles)
XI = [xi1, xi2, xi3, xi4, xi5, xi6]; %matrix with all xi vectors

%define a matrix containing all the matrix exponentials of the twists
exp_twist_matrix = zeros(6,6,6);
for i=1:6
    exp_twist_matrix(:,:,i) = EXPTWIST(w_matrix(:,i), q_matrix(:,i), thetas(i));
end

%for loop to calculate Jb = [(Ad^-1 of exp(ξi^θi)...exp(ξn^θn)gst(0))*ξi]
Jb = zeros(6); %initialize Body Jacobian Matrix
for i=1:6

    %To get Jb, first compute Ad^-1 of exp(ξi^θi)...exp(ξn^θn)gst(0)
    tempM = exp_twist_matrix(:,:,i:end); %temp matrix with exp matrices from i to n

    poe_i_to_n = tempM(:,:,1); %initialize poe as the first matrix of the temporary matrix
    for j = 2:size(tempM,3)
        poe_i_to_n = poe_i_to_n * tempM(:,:,j);
    end

    poe_i_to_n = poe_i_to_n * gst0;

    %compute Ad^-1 of poe_i_to_n
    poe_i_to_n_inv = FINV(poe_i_to_n);
    Ad_inv = adjoint(poe_i_to_n_inv);

    Jb(:,i) = Ad_inv*XI(:,i);

end





