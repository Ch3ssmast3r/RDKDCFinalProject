%% RDKDC Lab3 Question 3a
% 
% The purpose of this program is to test the ur5FwdKin method and see if 
% it outputs the right results
%
% Written by Aabhas Jain

ur5=ur5_interface();
theta_home = [0; -pi/2; 0; -pi/2; 0; 0];
thetas = [0;0;0;0;0;0];
g = ur5FwdKin(thetas)
fwdKinToolFrame = tf_frame('base_link', 'fwdKinToolFrame', eye(4));
fwdKinToolFrame.move_frame('base_link', g);
ur5.move_joints(thetas, 10);
pause(10);
ur5_g = ur5.get_current_transformation('base_link', 'tool0')
disp('The difference between the calculated and actual end effector pose is: ');
disp(g-ur5_g);

thetas = [pi/2; 0; 0; 0; 0; 0];
g = ur5FwdKin(thetas)
fwdKinToolFrame = tf_frame('base_link', 'fwdKinToolFrame', eye(4));
fwdKinToolFrame.move_frame('base_link', g);
ur5.move_joints(thetas, 10);
pause(10);
ur5_g = ur5.get_current_transformation('base_link', 'tool0')
disp('The difference between the calculated and actual end effector pose is: ');
disp(g-ur5_g);

thetas = [pi/2; -pi/2; 0; 0; 0; 0];
g = ur5FwdKin(thetas)
fwdKinToolFrame = tf_frame('base_link', 'fwdKinToolFrame', eye(4));
fwdKinToolFrame.move_frame('base_link', g);
ur5.move_joints(thetas, 15);
pause(15);
ur5_g = ur5.get_current_transformation('base_link', 'tool0')
disp('The difference between the calculated and actual end effector pose is: ');
disp(g-ur5_g);

thetas = [pi/2; -pi/2; pi/6; 0; 0; 0];
g = ur5FwdKin(thetas)
fwdKinToolFrame = tf_frame('base_link', 'fwdKinToolFrame', eye(4));
fwdKinToolFrame.move_frame('base_link', g);
ur5.move_joints(thetas, 15);
pause(15);
ur5_g = ur5.get_current_transformation('base_link', 'tool0')
disp('The difference between the calculated and actual end effector pose is: ');
disp(g-ur5_g);

thetas = [pi/2; -pi/2; pi/6; pi/4; 0; 0];
g = ur5FwdKin(thetas)
fwdKinToolFrame = tf_frame('base_link', 'fwdKinToolFrame', eye(4));
fwdKinToolFrame.move_frame('base_link', g);
ur5.move_joints(thetas, 15);
pause(15);
ur5_g = ur5.get_current_transformation('base_link', 'tool0')
disp('The difference between the calculated and actual end effector pose is: ');
disp(g-ur5_g);

thetas = [pi/2; -pi/2; pi/6; pi/4; pi/3; 0];
g = ur5FwdKin(thetas)
fwdKinToolFrame = tf_frame('base_link', 'fwdKinToolFrame', eye(4));
fwdKinToolFrame.move_frame('base_link', g);
ur5.move_joints(thetas, 15);
pause(15);
ur5_g = ur5.get_current_transformation('base_link', 'tool0')
disp('The difference between the calculated and actual end effector pose is: ');
disp(g-ur5_g);

thetas = [pi/2; -pi/2; pi/6; pi/4; pi/3; pi/4];
g = ur5FwdKin(thetas)
fwdKinToolFrame = tf_frame('base_link', 'fwdKinToolFrame', eye(4));
fwdKinToolFrame.move_frame('base_link', g);
ur5.move_joints(thetas, 15);
pause(15);
ur5_g = ur5.get_current_transformation('base_link', 'tool0')
disp('The difference between the calculated and actual end effector pose is: ');
disp(g-ur5_g);

thetas = theta_home;
g = ur5FwdKin(thetas)
fwdKinToolFrame = tf_frame('base_link', 'fwdKinToolFrame', eye(4));
fwdKinToolFrame.move_frame('base_link', g);
ur5.move_joints(thetas, 15);
pause(15);
ur5_g = ur5.get_current_transformation('base_link', 'tool0')
disp('The difference between the calculated and actual end effector pose is: ');
disp(g-ur5_g);