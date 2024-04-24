%% RDKDC Lab 3 Question 5 - ur5RRcontrol
%
% Written by Aabhas Jain

% Inputs: 
    % gdesired: a homogenous trnasform that is the desired end-effector
    % pose, gst
    % K: the gain on the controller
    % ur5: the ur5 interface object.

% Purpose:
    % implement a discrete-time resolved-rate control systme. It iterataively
    % implements the Resolved-Rate control system discussed in class:
    % q_(k+1) = q_k - K*T_step*[Jb(q_k)]^-1*xi_k
% Outputs: 
    % finalerr: this should be -1 if there is a failure. if there is no
    % failure then it should be the final positional error in cm.

function finalerr = ur5RRcontrol(gdesired, K, ur5)
T_step = 1;
v_abs_error = 5;
w_abs_error = 15;

q_k = ur5.get_current_joints;
J_bqk = ur5BodyJacobian(q_k);

xi_error = getXi(FINV(gdesired)*ur5FwdKin(q_k));
v_k = xi_error(1:3);
w_k = xi_error(4:6);

if (manipulability(J_bqk, 'invcond') < 0.1)
    finalerr = -1;
end
if (norm(v_k) < v_abs_error && norm(w_k) < w_abs_error)
    finalerr = xi_error;
end

q_k_new = q_k - K*T_step*inv(J_bqk)*xi_error;
g_new = ur5FwdKin(q_k_new);
fwdKinToolFrame = tf_frame('base_link', 'fwdKinToolFrame', eye(4));
fwdKinToolFrame.move_frame('base_link', g_new);
ur5.move_joints(thetas, T_step);
pause(T_step);
end