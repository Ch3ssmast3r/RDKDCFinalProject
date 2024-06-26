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

function finalerr = ur5TJcontrol(gdesired, ur5)
K = 15; % proportionality constant for TJ control
T_step = 0.01; % time step amount
v_abs_error = 3 / 1000; % mm
w_abs_error = 0.5 * pi/180; % degrees
q_k = ur5.get_current_joints; % getting current joints
speed_limit = 0.25; % from ur5_interface code

xi_error = getXi(FINV(gdesired)*ur5FwdKinDH(q_k));
v_k = xi_error(1:3);
w_k = xi_error(4:6);
norm_v_k = norm(v_k);
norm_w_k = norm(w_k);
ABORT = 0;

% The program will run until the errors are less than the allowed errors. 
while (norm_v_k > v_abs_error || norm_w_k > w_abs_error && ABORT ~= 1)
    q_k = ur5.get_current_joints; % get current joints
    J_bqk = ur5BodyJacobian(q_k); % compute body jacobian

    xi_error = getXi(FINV(gdesired)*ur5FwdKinDH(q_k)); % compute Xi error
    v_k = xi_error(1:3); % getting the v component
    w_k = xi_error(4:6); % getting the w component
    norm_v_k = norm(v_k); % computing norm of v component of xi error
    norm_w_k = norm(w_k); % computing norm of w component of xi error.

    % safety check, seeing if the manipulability is good. 
    inv_cond = manipulability(J_bqk, 'invcond');
    if (inv_cond < 0.002)
        ABORT = 1;
        break;
    end
    
    % computing q_k_new, formula from lecture. 
    q_k_new = q_k - K*T_step*(transpose(J_bqk)*xi_error); 
    g_new = ur5FwdKinDH(q_k_new); % computing new frame

    % plotting new frame so we can see where the robot is moving. 
    fwdKinToolFrame = tf_frame('base_link', 'fwdKinToolFrame', eye(4));
    fwdKinToolFrame.move_frame('base_link', g_new);

    % use the speed limit to calculate the minimum time. This calculation
    % is based off of the one done in ur5_interface. we just set the speed
    % limit and joint velocity to calculate the shortest time interval, and
    % add 0.01 for a little breathing room. This is to get the shortest
    % motions possible. 
    joint_velocity = q_k_new - q_k;
    time_interval = 0.1 + max(abs(joint_velocity))/speed_limit;
    ur5.move_joints(q_k_new, time_interval);
    pause(time_interval + .05);
end

% computing errors at end.
g_error = gdesired - ur5FwdKinDH(ur5.get_current_joints());
pos_error = g_error(1:3, 4) * 100;
finalerr = norm(pos_error);

% abort condition. 
if (ABORT == 1)
    finalerr = -1;
end
end