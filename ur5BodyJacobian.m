function Jb = ur5BodyJacobian(q)
%Function accepts q: 6 × 1 joint space variable vector = [θ1, θ2, θ3, θ4,
%θ5, θ6]' where θn is the angle of joint n for n = 1, · · · , 6.
%Returns J: Body Jacobian, Jb_st (6 × 6 matrix)

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

xi1 = [cross(-w1,q1);w1];
xi2 = [cross(-w2,q2);w2];
xi3 = [cross(-w3,q3);w3];
xi4 = [cross(-w4,q4);w4];
xi5 = [cross(-w5,q5);w5];
xi6 = [cross(-w6,q6);w6];


%call ur5FwdKin.m to get gst, then get the Adjoint matrix
gst = urFwdKin(q);
gst_inv = FINV(gst);
Ad_gst_inv = adjoint(gst_inv); 
xi_matrix = [xi1, xi2, xi3, xi4, xi5, xi6];

Jb = zeros(6);

for i = 1:6
    Jb(:,i) = Ad_gst_inv*xi_matrix(:,i);
end




