%% Question 3A
% 
% The purpose of this program is to test the ur5FwdKin method and see if 
% it outputs the right results
%
% Written by Aabhas Jain

ur5=ur5_interface();
theta_home = [0; -pi/2; 0; -pi/2; 0; 0];


thetas = [0;0;0;0;0;0]; % define thetas
g1 = ur5FwdKin(thetas) % compute forward kinematics
fwdKinToolFrame = tf_frame('base_link', 'fwdKinToolFrame', eye(4)); % make frame
fwdKinToolFrame.move_frame('base_link', g1); % move frame
ur5.move_joints(thetas, 10); % move joints using the thetas
pause(10); % wait to give robot time to move
ur5_g1 = ur5.get_current_transformation('base_link', 'tool0') % get transformation from RVIZ
disp('The difference between the calculated and actual end effector pose is: ');
disp(g1-ur5_g1); % compute the difference between RVIZ and the expected position

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

thetas = [pi/2; -pi/2; 0; 0; 0; 0];
g3 = ur5FwdKin(thetas)
fwdKinToolFrame = tf_frame('base_link', 'fwdKinToolFrame', eye(4));
fwdKinToolFrame.move_frame('base_link', g3);
ur5.move_joints(thetas, 15);
pause(15);
ur5_g3 = ur5.get_current_transformation('base_link', 'tool0')
disp('The difference between the calculated and actual end effector pose is: ');
disp(g3-ur5_g3);

thetas = [pi/2; -pi/2; pi/6; 0; 0; 0];
g5 = ur5FwdKin(thetas)
fwdKinToolFrame = tf_frame('base_link', 'fwdKinToolFrame', eye(4));
fwdKinToolFrame.move_frame('base_link', g5);
ur5.move_joints(thetas, 15);
pause(15);
ur5_g4 = ur5.get_current_transformation('base_link', 'tool0')
disp('The difference between the calculated and actual end effector pose is: ');
disp(g5-ur5_g4);

thetas = [pi/2; -pi/2; pi/6; pi/4; 0; 0];
g5 = ur5FwdKin(thetas)
fwdKinToolFrame = tf_frame('base_link', 'fwdKinToolFrame', eye(4));
fwdKinToolFrame.move_frame('base_link', g5);
ur5.move_joints(thetas, 15);
pause(15);
ur5_g5 = ur5.get_current_transformation('base_link', 'tool0')
disp('The difference between the calculated and actual end effector pose is: ');
disp(g5-ur5_g5);

thetas = [pi/2; -pi/2; pi/6; pi/4; pi/3; 0];
g6 = ur5FwdKin(thetas)
fwdKinToolFrame = tf_frame('base_link', 'fwdKinToolFrame', eye(4));
fwdKinToolFrame.move_frame('base_link', g6);
ur5.move_joints(thetas, 15);
pause(15);
ur5_g6 = ur5.get_current_transformation('base_link', 'tool0')
disp('The difference between the calculated and actual end effector pose is: ');
disp(g6-ur5_g6);

thetas = [pi/2; -pi/2; pi/6; pi/4; pi/3; pi/4];
g7 = ur5FwdKin(thetas)
fwdKinToolFrame = tf_frame('base_link', 'fwdKinToolFrame', eye(4));
fwdKinToolFrame.move_frame('base_link', g7);
ur5.move_joints(thetas, 15);
pause(15);
ur5_g7 = ur5.get_current_transformation('base_link', 'tool0')
disp('The difference between the calculated and actual end effector pose is: ');
disp(g7-ur5_g7);

thetas = theta_home;
g = ur5FwdKin(thetas)
fwdKinToolFrame = tf_frame('base_link', 'fwdKinToolFrame', eye(4));
fwdKinToolFrame.move_frame('base_link', g);
ur5.move_joints(thetas, 15);
pause(15);
ur5_g = ur5.get_current_transformation('base_link', 'tool0')
disp('The difference between the calculated and actual end effector pose is: ');
disp(g-ur5_g);


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
%done using the function "XiToG" which takes a twist and finds the
%homogeneous transformation.
gOne = expm(xiHat1);
gTwo = expm(xiHat2);
gThree = expm(xiHat3);

error1 = g1-gOne
error2 = g2-gTwo
error3 = g3-gThree