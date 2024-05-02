function Adj = Adjoint(gst)
%Function accepts a homogeneous transformation matrix (4x4)
%Returns its 6x6 adjoint

%Author: James Kaluna

R = gst(1:3,1:3); %rotation matrix
p = gst(1:3, 4); %translation vector

p_hat = SKEW3(p);

Adj = [R, p_hat*R; zeros(3), R];

