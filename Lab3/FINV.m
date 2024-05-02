function g_inv = FINV(g)
%Function accepts 4x4 homogeneous transformation g and returns its invere
%uses SKEW3() and Rodrigues' formula

%recall: g=(p, R) in SE(3) 

R = g(1:3, 1:3); 
p = g(1:3,4);

%g^-1 = (-R'p, R')
R_inv = transpose(R); %rotation matrix -> R^-1 = R' (FYI transpose(R)=R.')
p_inv = -R_inv*p;

g_inv = zeros(4,4);
g_inv(1:3, 1:3) = R_inv;
g_inv(1:3, 4) = p_inv;
g_inv(4,4) = 1;
