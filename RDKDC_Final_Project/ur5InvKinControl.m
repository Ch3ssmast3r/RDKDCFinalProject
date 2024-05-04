function finalerr = ur5InvKinControl(gdesired, ur5)
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

deltaT = 0.01; %timestep
s = 0:deltaT:1; %can make it smoother using cubic or fifth order polynomials

p_s = p_initial + s.*(p_final-p_initial); %translation path
%R_s = R_initial*MatrixExp(MatrixLog(R_initial'*R_final).*s); %rotation
%path - axis of rotation is constant in the body frame. Needs 'for loop'.
%No worries now since we keep R constant

%cartesian path in SE(3)
g_s = zeros(4,4,length(s));
g_s(:,:,1) = g_initial;
%g_s(:,:,end) = gdesired;

%% Use Inverse Kinematics to get joint angles and move UR5 to the 'best' solution

thetas = zeros(6, 8, length(s)); %initialize joint angle matrices
q = q_initial;
%q = zeros(6, length(s)); %joint angle vectors
%q(:,1) = q_initial; 

for i = 2:length(s)
    g_s(:,:,i) = [R_final, p_s(i); zeros(1,3), 1];

    %use Inverse Kinematics to get the joint angles
    thetas(:,:,i) = ur5InvKin(g_s(:,:,i));

    %find the best theta by comparing with previous joint angles. Choose the closest 
    %More - avoid solutions with singulaties or joint (velocity) limits
    [min_err, index] = min(vecnorm(thetas(:,:,i) - q(:,i-1)));
    q = thetas(:,index,i);
    
    %move frame
    fwdKinToolFrame = tf_frame('base_link', 'fwdKinToolFrame', eye(4));
    fwdKinToolFrame.move_frame('base_link', g_s(:,:,i));

    %move ur5
    ur5.move_joints(q, 0.5);
    pause(0.5)
end

g_error = gdesired - ur5FwdKinDH(ur5.get_current_joints());
pos_error = g_error(1:3, 4) * 100;
finalerr = norm(pos_error);