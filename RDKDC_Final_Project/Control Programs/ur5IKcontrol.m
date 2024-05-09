function finalerr = ur5IKcontrol(gdesired, ur5)
% Move robot along desired Cartesian path by finding corresponding path in joint space
% uses IK to find joint angles from the cartesian space (SE(3))

%Author: James Kaluna

%get initial pose
q_initial = ur5.get_current_joints; %current joint angles
g_initial = ur5FwdKinDH(q_initial);

%% generate a straight line Cartesian path

%{
Note that a straight line in joint space does not generally yield a
straight line in Cartesian/task space. We want straight lines in task
space, so we define the path in task space.
We also decouple rotation and translation becuse a straight-line screw only
means that the screw axis is constant.
So, we define the path g = (R,p)
%}

p_initial = g_initial(1:3,4);
R_initial = g_initial(1:3,1:3);

p_final = gdesired(1:3,4);
R_final = gdesired(1:3,1:3);

delta = .025; %step
s = 0:delta:1; %can make it smoother using cubic or fifth order polynomials


%% Use Inverse Kinematics to get joint angles and move UR5 to the 'best' solution
%q = q_initial;
q = zeros(6, length(s)); %joint angle vectors
q(:,1) = q_initial; 
speed_limit = 0.25;

for i = 2:length(s)
    p_s = p_initial + s(i).*(p_final-p_initial); %translation path
    R_s = R_initial * MatrixExp(MatrixLog(R_initial' * R_final) * s(i)); %rotation path
    g_s = [R_s, p_s; zeros(1,3), 1]; %cartesian path in SE(3)

    %use Inverse Kinematics to get the joint angles
    thetas = ur5InvKin(g_s);

    %find the best theta by comparing with previous joint angles. Choose the closest 
    %More - avoid solutions with singulaties or joint (velocity) limits
    [min_err, index] = min(vecnorm(thetas - q(:,i-1)));

    q(:,i) = thetas(:,index);
    
    %move frame
    fwdKinToolFrame = tf_frame('base_link', 'fwdKinToolFrame', eye(4));
    fwdKinToolFrame.move_frame('base_link', g_s);

    %move ur5

    % use the speed limit to calculate the minimum time. This calculation
    % is based off of the one done in ur5_interface. we just set the speed
    % limit and joint velocity to calculate the shortest time interval, and
    % add 0.01 for a little breathing room. This is to get the shortest
    % motions possible. 
    joint_velocity = q(:,i) - ur5.get_current_joints();
    time_interval = 0.01 + max(abs(joint_velocity))/speed_limit;
    ur5.move_joints(q(:,i), time_interval);
    pause(time_interval+0.05);
end

g_error = gdesired - ur5FwdKinDH(ur5.get_current_joints());
pos_error = g_error(1:3, 4) * 100;
finalerr = norm(pos_error);