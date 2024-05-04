%% RDKDC Final Project - ur5RRcontrol
%
% Written by Aabhas Jain
% Edited by Andrew Palacio

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

%So far, the simulation worked at a bare minimum by fixing K at 55.

function finalerr = ur5TJcontrol(gdesired, K, ur5)
k_min = 20;
k_max = 75;
K = 45;
T_step = 0.01;
v_abs_error = 2 / 100;
w_abs_error = 3 * pi/180;
q_k = ur5.get_current_joints;
J_bqk = ur5BodyJacobian(q_k);

xi_error = getXi(FINV(gdesired)*ur5FwdKinDH(q_k));
v_k = xi_error(1:3);
w_k = xi_error(4:6);
norm_v_k = norm(v_k);
norm_w_k = norm(w_k);
ABORT = 0;

while (norm_v_k > v_abs_error || norm_w_k > w_abs_error && ABORT ~= 1)
    q_k = ur5.get_current_joints;
    J_bqk = ur5BodyJacobian(q_k);

    xi_error = getXi(FINV(gdesired)*ur5FwdKinDH(q_k));
    v_k = xi_error(1:3);
    w_k = xi_error(4:6);
    norm_v_k = norm(v_k);
    norm_w_k = norm(w_k);
    inv_cond = manipulability(J_bqk, 'invcond');
    % K_rr = 4/norm(J_bqk\xi_error);
    term = transpose(J_bqk)*xi_error;
    % if norm_v_k > .1
    %     % K = 8/norm_v_k;
    %     K = 45;
    % else
    %     K = 65;
    % end
    % K = 100 - 250*norm_v_k; %finding K using a linear function of error
    % K = 8/norm_v_k;
    % fprintf('K: %6.2f \n', K);
    % fprintf('Determinant: %6.4f \n', det(J_bqk))
    % fprintf('Error: %6.4f \n', norm_v_k);
    % fprintf('Term: %6.2f \n', term)
    if (K < k_min)
        K = k_min;
        fprintf('K: %6.2f \n', K);
    end
    if (K > k_max) 
        K = k_max;
        fprintf('K: %6.2f \n', K);
    end


    if (inv_cond < 0.002)
        ABORT = 1;
        break;
    end
    q_k_new = q_k - K*T_step*(transpose(J_bqk)*xi_error);
    g_new = ur5FwdKinDH(q_k_new);
    fwdKinToolFrame = tf_frame('base_link', 'fwdKinToolFrame', eye(4));
    fwdKinToolFrame.move_frame('base_link', g_new);
    ur5.move_joints(q_k_new, 1.25);
    pause(1.25);
end
g_error = gdesired - ur5FwdKinDH(ur5.get_current_joints());
pos_error = g_error(1:3, 4) * 100;
finalerr = norm(pos_error);
if (ABORT == 1)
    finalerr = -1;
end
end