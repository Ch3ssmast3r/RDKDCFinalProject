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
T_step = 0.05;
v_abs_error = 1 / 100;
w_abs_error = 4 * pi/180;

q_k = ur5.get_current_joints;
J_bqk = ur5BodyJacobian(q_k);

xi_error = getXi(FINV(gdesired)*ur5FwdKin(q_k));
v_k = xi_error(1:3);
w_k = xi_error(4:6);
norm_v_k = norm(v_k);
norm_w_k = norm(w_k);
ABORT = 0;

while (norm_v_k > v_abs_error || norm_w_k > w_abs_error && ABORT ~= 1)
    q_k = ur5.get_current_joints;
    J_bqk = ur5BodyJacobian(q_k);

    xi_error = getXi(FINV(gdesired)*ur5FwdKin(q_k));
    v_k = xi_error(1:3);
    w_k = xi_error(4:6);
    norm_v_k = norm(v_k);
    norm_w_k = norm(w_k);
    inv_cond = manipulability(J_bqk, 'invcond');

    % if (inv_cond < 0.002)
    %     ABORT = 1;
    %     break;
    % end
    q_k_new = q_k - K*T_step*transpose(J_bqk)*xi_error;
    g_new = ur5FwdKin(q_k_new);
    fwdKinToolFrame = tf_frame('base_link', 'fwdKinToolFrame', eye(4));
    fwdKinToolFrame.move_frame('base_link', g_new);
    ur5.move_joints(q_k_new, 1.25);
    pause(1.25);
end
g_error = gdesired - ur5FwdKin(ur5.get_current_joints());
pos_error = g_error(1:3, 4) * 100;
finalerr = norm(pos_error);
if (ABORT == 1)
    finalerr = -1;
end
end