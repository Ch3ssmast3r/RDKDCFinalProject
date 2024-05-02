
%% UR5 interface
ur5 = ur5_interface();
theta_home = [0; -pi/2; 0; -pi/2; 0; 0];

%% Question 3A
% 
% The purpose of this program is to test the ur5FwdKin method and see if 
% it outputs the right results
%
% Written by Aabhas Jain

thetas = [0;0;0;0;0;0]; % define thetas
g1 = ur5FwdKin(thetas) % compute forward kinematics
fwdKinToolFrame = tf_frame('base_link', 'fwdKinToolFrame', eye(4)); % make frame
fwdKinToolFrame.move_frame('base_link', g1); % move frame
ur5.move_joints(thetas, 10); % move joints using the thetas
pause(10); % wait to give robot time to move
ur5_g1 = ur5.get_current_transformation('base_link', 'tool0') % get transformation from RVIZ
disp('The difference between the calculated and actual end effector pose is: ');
disp(g1-ur5_g1); % compute the difference between RVIZ and the expected position
pause(10); % give time to look at results

% this process is repeated 6 times to test each joint and the overall
% forward kinematics. 
thetas = [pi/2; 0; 0; 0; 0; 0];
g2 = ur5FwdKin(thetas)
fwdKinToolFrame = tf_frame('base_link', 'fwdKinToolFrame', eye(4));
fwdKinToolFrame.move_frame('base_link', g2);
ur5.move_joints(thetas, 10);
pause(10);
ur5_g2 = ur5.get_current_transformation('base_link', 'tool0')
disp('The difference between the calculated and actual end effector pose is: ');
disp(g2-ur5_g2);
pause(10);

thetas = [pi/2; -pi/2; 0; 0; 0; 0];
g3 = ur5FwdKin(thetas)
fwdKinToolFrame = tf_frame('base_link', 'fwdKinToolFrame', eye(4));
fwdKinToolFrame.move_frame('base_link', g3);
ur5.move_joints(thetas, 15);
pause(15);
ur5_g3 = ur5.get_current_transformation('base_link', 'tool0')
disp('The difference between the calculated and actual end effector pose is: ');
disp(g3-ur5_g3);
pause(10);

thetas = [pi/2; -pi/2; pi/6; 0; 0; 0];
g5 = ur5FwdKin(thetas)
fwdKinToolFrame = tf_frame('base_link', 'fwdKinToolFrame', eye(4));
fwdKinToolFrame.move_frame('base_link', g5);
ur5.move_joints(thetas, 15);
pause(15);
ur5_g4 = ur5.get_current_transformation('base_link', 'tool0')
disp('The difference between the calculated and actual end effector pose is: ');
disp(g5-ur5_g4);
pause(10);

thetas = [pi/2; -pi/2; pi/6; pi/4; 0; 0];
g5 = ur5FwdKin(thetas)
fwdKinToolFrame = tf_frame('base_link', 'fwdKinToolFrame', eye(4));
fwdKinToolFrame.move_frame('base_link', g5);
ur5.move_joints(thetas, 15);
pause(15);
ur5_g5 = ur5.get_current_transformation('base_link', 'tool0')
disp('The difference between the calculated and actual end effector pose is: ');
disp(g5-ur5_g5);
pause(10);

thetas = [pi/2; -pi/2; pi/6; pi/4; pi/3; 0];
g6 = ur5FwdKin(thetas)
fwdKinToolFrame = tf_frame('base_link', 'fwdKinToolFrame', eye(4));
fwdKinToolFrame.move_frame('base_link', g6);
ur5.move_joints(thetas, 15);
pause(15);
ur5_g6 = ur5.get_current_transformation('base_link', 'tool0')
disp('The difference between the calculated and actual end effector pose is: ');
disp(g6-ur5_g6);
pause(10);

thetas = [pi/2; -pi/2; pi/6; pi/4; pi/3; pi/4];
g7 = ur5FwdKin(thetas)
fwdKinToolFrame = tf_frame('base_link', 'fwdKinToolFrame', eye(4));
fwdKinToolFrame.move_frame('base_link', g7);
ur5.move_joints(thetas, 15);
pause(15);
ur5_g7 = ur5.get_current_transformation('base_link', 'tool0')
disp('The difference between the calculated and actual end effector pose is: ');
disp(g7-ur5_g7);
pause(10);

thetas = theta_home;
g = ur5FwdKin(thetas)
fwdKinToolFrame = tf_frame('base_link', 'fwdKinToolFrame', eye(4));
fwdKinToolFrame.move_frame('base_link', g);
ur5.move_joints(thetas, 15);
pause(15);
ur5_g = ur5.get_current_transformation('base_link', 'tool0')
disp('The difference between the calculated and actual end effector pose is: ');
disp(g-ur5_g);
pause(10);

%% Part B. Testing ur5BodyJacobian
% The goal of this test is to determine if the ur5BodyJacobian function
% works as intended. To do so, we compute the central difference
% approximate to the Jacobian and see if the error between the Approximate
% Jacobian and actual Jacobian is small. 

q = rand(6,1)*2*pi - pi; % joint angles
Jb = ur5BodyJacobian(q); %calculate Body Jocabian

%define e_i's
e1 = [1 0 0 0 0 0]'; e2 = [0 1 0 0 0 0]'; e3 = [0 0 1 0 0 0]'; 
e4 = [0 0 0 1 0 0]'; e5 = [0 0 0 0 1 0]'; e6 = [0 0 0 0 0 1]';

e = [e1, e2, e3, e4, e5, e6];

eps = 0.00001; %slight offset

gst = ur5FwdKin(q);
gst_inv = FINV(gst);

Japprox = zeros(6); %initialize Jacobian approximation

for i = 1:6
    upper = ur5FwdKin(q+eps*e(:,i));
    lower = ur5FwdKin(q-eps*e(:,i));
    dg_dqi = (1/(2*eps))*(upper-lower);
    Japprox(:,i) = twistify(gst_inv*dg_dqi);
end

%compute matrix norm of the error btn Japprox and Jb
err_norm = norm(Japprox-Jb);
disp('norm(Jappox-J) = ')
disp(err_norm)
     
%% Part C - Testing manipulability measures
%This should be added to main lab3 code later.

%This part will calculate the manipulability near a singularity, more
%specifically when theta3 = 0.

q = [pi/3;-pi/3;NaN;pi/4;pi/6;-pi/6]; %joint space variables. Theta 3 will be left blank for now.
theta3 = (-pi/4:.01:pi/4); %array of values of theta3 near the singularity, theta3 = 0

Q = zeros(6,length(theta3)); %pre-define these arrays
muSigmaMin = zeros(length(theta3),1);
muDetJac = zeros(length(theta3),1);
muInvCond = zeros(length(theta3),1);
for i = 1:length(theta3) %we will calulate manipulability for each value of theta3.
    Q(:,i)=q; %start by placing the other 5 joint space variables.
    Q(3,i)=theta3(i); %fill in theta3
    
    %calculate the Jacobian for each qi
    J = ur5BodyJacobian(Q(:,i));
    
    %calculate each of the 3 manipulability measures: 'sigmamin','detjac',or'invcond'
    muSigmaMin(i) = manipulability(J,'sigmamin');
    muDetJac(i) = manipulability(J,'detjac');
    muInvCond(i) = manipulability(J,'invcond');
end

%generate plots of manipulability, mu, vs theta.
figure(1)
plot(theta3,muSigmaMin)
title("Singularity: Minimum Singular value")
xlabel('Theta3, (rad)')
ylabel('Manipulability, mu')

figure(2)
plot(theta3,muDetJac)
title("Signularity: Determinant of Jacobian")
xlabel('Theta3 (rad)')
ylabel('Manipulability, mu')

figure(3)
plot(theta3,muInvCond)
title("Singularity: Inverse Condition")
xlabel('Theta3 (rad)')
ylabel('Manipulability, mu')

%% Part D. Testing getXi
%The goal of this part is to choose some arbitrary homogeneous
%transformations, calculate the twist using the getXi function, and then
%exponentiate the twist to get the transformation that we started with. 

%Pick some arbitrary transformations.
R1 = eye(3); %translational case
p1 = [2;1;0];

R2 = ROTZ(1)*ROTX(pi/2);
p2 = [-1;0;1];

R3 = ROTY(pi/4)*ROTX(pi/2)*ROTZ(1);
p3 = [.5;.5;.5];

%take R and p into homogeneous representation.
g1 = RPtoG(R1,p1);
g2 = RPtoG(R2,p2);
g3 = RPtoG(R3,p3);

%extract the twists using getxi
xi1 = getXi(g1);
xi2 = getXi(g2);
xi3 = getXi(g3);

%extract v and w
v1 = xi1(1:3);
w1 = xi1(4:6);
xiHat1 = [SKEW3(w1),v1;0,0,0,0];

v2 = xi2(1:3);
w2 = xi2(4:6);
xiHat2 = [SKEW3(w2),v2;0,0,0,0];

v3 = xi3(1:3);
w3 = xi3(4:6);
xiHat3 = [SKEW3(w3),v3;0,0,0,0];

%calculate g again from the twists, then see if they match. This will be
%done using the function expm which takes a twist and finds the
%homogeneous transformation.
gOne = expm(xiHat1);
gTwo = expm(xiHat2);
gThree = expm(xiHat3);

error1 = norm(g1-gOne)
error2 = norm(g2-gTwo)
error3 = norm(g3-gThree)

%% Part E. testing Resolved Rate Controller. 
% The goal of this test is to determine whether the resolved rate
% controller works or not. This will be done by defining a desired
% configuration and a gain and then using the resolved rate controller
% command to show that the robot moves to the goal. A second test will be
% conducted to show that the robot fails when we are near a singularity.
ur5 = ur5_interface();
starting_config = [0; 0; -pi/4; 0; pi/6; 0];
starting_frame = ur5FwdKin(starting_config);
fwdKinToolFrame = tf_frame('base_link', 'fwdKinToolFrame', eye(4));
fwdKinToolFrame.move_frame('base_link', starting_frame);
ur5.move_joints(starting_config, 10);
pause(10);
thetas = [pi/2; -pi/2; pi/6; pi/4; pi/3; pi/4];
gdesired = ur5FwdKin(thetas);
K = 1.3;
final_error = ur5RRcontrol(gdesired, K, ur5);
fprintf('The final positional error is %.2f cm\n', final_error)

% Part E Test 2. Showing that the command terminates if we are near a
% singularity. 
thetas = [pi/2; -pi/2; 0; pi/4; pi/3; pi/4];
gdesired = ur5FwdKin(thetas);
K = 1.3;
final_error = ur5RRcontrol(gdesired, K, ur5);
fprintf('The final positional error is %.2f cm\n', final_error)
