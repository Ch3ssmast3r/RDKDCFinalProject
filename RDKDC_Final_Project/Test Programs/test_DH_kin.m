%% UR5 interface
ur5 = ur5_interface();
theta_home = [0; -pi/2; 0; -pi/2; 0; 0];

%% Question 3A
% 
% The purpose of this program is to test the ur5FwdKinDH method and see if 
% it outputs the right results
%
% Written by Aabhas Jain

thetas = [0;0;0;0;0;0]; % define thetas
g1 = ur5FwdKinDH(thetas) % compute forward kinematics
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
g2 = ur5FwdKinDH(thetas)
fwdKinToolFrame = tf_frame('base_link', 'fwdKinToolFrame', eye(4));
fwdKinToolFrame.move_frame('base_link', g2);
ur5.move_joints(thetas, 10);
pause(10);
ur5_g2 = ur5.get_current_transformation('base_link', 'tool0')
disp('The difference between the calculated and actual end effector pose is: ');
disp(g2-ur5_g2);
pause(10);

thetas = [pi/2; -pi/2; 0; 0; 0; 0];
g3 = ur5FwdKinDH(thetas)
fwdKinToolFrame = tf_frame('base_link', 'fwdKinToolFrame', eye(4));
fwdKinToolFrame.move_frame('base_link', g3);
ur5.move_joints(thetas, 15);
pause(15);
ur5_g3 = ur5.get_current_transformation('base_link', 'tool0')
disp('The difference between the calculated and actual end effector pose is: ');
disp(g3-ur5_g3);
pause(10);

thetas = [pi/2; -pi/2; pi/6; 0; 0; 0];
g5 = ur5FwdKinDH(thetas)
fwdKinToolFrame = tf_frame('base_link', 'fwdKinToolFrame', eye(4));
fwdKinToolFrame.move_frame('base_link', g5);
ur5.move_joints(thetas, 15);
pause(15);
ur5_g4 = ur5.get_current_transformation('base_link', 'tool0')
disp('The difference between the calculated and actual end effector pose is: ');
disp(g5-ur5_g4);
pause(10);

thetas = [pi/2; -pi/2; pi/6; pi/4; 0; 0];
g5 = ur5FwdKinDH(thetas)
fwdKinToolFrame = tf_frame('base_link', 'fwdKinToolFrame', eye(4));
fwdKinToolFrame.move_frame('base_link', g5);
ur5.move_joints(thetas, 15);
pause(15);
ur5_g5 = ur5.get_current_transformation('base_link', 'tool0')
disp('The difference between the calculated and actual end effector pose is: ');
disp(g5-ur5_g5);
pause(10);

thetas = [pi/2; -pi/2; pi/6; pi/4; pi/3; 0];
g6 = ur5FwdKinDH(thetas)
fwdKinToolFrame = tf_frame('base_link', 'fwdKinToolFrame', eye(4));
fwdKinToolFrame.move_frame('base_link', g6);
ur5.move_joints(thetas, 15);
pause(15);
ur5_g6 = ur5.get_current_transformation('base_link', 'tool0')
disp('The difference between the calculated and actual end effector pose is: ');
disp(g6-ur5_g6);
pause(10);

thetas = [pi/2; -pi/2; pi/6; pi/4; pi/3; pi/4];
g7 = ur5FwdKinDH(thetas)
fwdKinToolFrame = tf_frame('base_link', 'fwdKinToolFrame', eye(4));
fwdKinToolFrame.move_frame('base_link', g7);
ur5.move_joints(thetas, 15);
pause(15);
ur5_g7 = ur5.get_current_transformation('base_link', 'tool0')
disp('The difference between the calculated and actual end effector pose is: ');
disp(g7-ur5_g7);
pause(10);

thetas = theta_home;
g = ur5FwdKinDH(thetas)
fwdKinToolFrame = tf_frame('base_link', 'fwdKinToolFrame', eye(4));
fwdKinToolFrame.move_frame('base_link', g);
ur5.move_joints(thetas, 15);
pause(15);
ur5_g = ur5.get_current_transformation('base_link', 'tool0')
disp('The difference between the calculated and actual end effector pose is: ');
disp(g-ur5_g);
pause(10);