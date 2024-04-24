function xi_vec = twistify(XI)
%function accepts xi_hat in se(3)
%vectorizes it and returns ξ twist = [v;w]

%Author: James Kaluna

w_hat = XI(1:3,1:3); %pull the skew-symmetric part of the upper 3x3 matrix
w = [w_hat(3,2); w_hat(1,3); w_hat(2,1)];
v = XI(1:3, 4);
xi_vec = [v; w];
