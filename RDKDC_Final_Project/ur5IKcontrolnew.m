
function finalerr = ur5IKcontrolnew(gdesired, ur5)
% Move robot along desired Cartesian path by finding corresponding path in joint space
% uses IK to find joint angles from the cartesian space (SE(3))

%Author: James Kaluna

%get initial pose
q_initial = ur5.get_current_joints; %current joint angles
g_initial = ur5FwdKin(q_initial);

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

%deltaT = 1; %timestep
%s = 0:deltaT:1; %can make it smoother using cubic or fifth order polynomials
N = 10; %number of points
Tf = 1; %total time to achieve trajectory
deltaT = Tf / (N - 1);

path = cell(1, N);
thetas = cell(1, N);
q = zeros(6, N); %joint angle vectors
q(:,1) = q_initial; 

for i = 2: N

    t = deltaT * (i-1);
    s = 3 * (t / Tf) ^ 2 - 2 * (t / Tf) ^ 3;
    %s = 10 * (t / Tf) ^ 3 - 15 * (t / Tf) ^ 4 + 6 * (t / Tf) ^ 5;

    path{i} = [R_initial * MatrixExp(MatrixLog(R_initial' * R_final) * s), ...
       p_initial + s * (p_final - p_initial); 0, 0, 0, 1];

    thetas{i} = ur5InvKin(path{i});

    %find the best theta by comparing with previous joint angles. Choose the closest 
    %More - avoid solutions with singulaties or joint (velocity) limits
    [min_err, index] = min(vecnorm(thetas{i} - q(:,i-1)));
    tempMat = thetas{i};
    q(:,i) = tempMat(:,index);
    
    %move frame
    fwdKinToolFrame = tf_frame('base_link', 'fwdKinToolFrame', eye(4));
    fwdKinToolFrame.move_frame('base_link', path{i});

    %move ur5
    ur5.move_joints(q(:,i), 2);
    pause(2)

end
g_error = gdesired - ur5FwdKin(ur5.get_current_joints());
pos_error = g_error(1:3, 4) * 100;
finalerr = norm(pos_error);

end