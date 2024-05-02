%% Testing Resolved Rate Controller
% The purpose of this program is to test our current resolved rate
% controller for moving between the two provided frames. 

ur5 = ur5_interface();
%ur5.switch_to_ros_control();
% starting_config = [1.1343; -0.7203; 0.8921; 1.3989; 1.5708; -2.0072];
 starting_config = [0.9556; -1.5709; 2.1994; -2.2750; -1.5708; 0]
g1 = ur5FwdKin(theta1);
%starting_frame = ur5FwdKin(starting_config);
gst1 = [0 -1 0 0.2;
        -1 0 0 0.60;
        0 0 -1 0.05;
        0 0 0 1];
gst2 = [0 -1 0 0.35;
        -1 0 0 0.60;
        0 0 -1 0.05;
        0 0 0 1];
disp('Moving to home');
ur5.move_joints(ur5.home, 20);
pause(20)
disp('Moving to starting config');
ur5.move_joints(starting_config, 20);
pause(20);
disp('Moving to gst1');
K = 3;
fwdKinToolFrame = tf_frame('base_link', 'fwdKinToolFrame', eye(4));
fwdKinToolFrame.move_frame('base_link', gst1);
move_to_gst1 = ur5RRcontrol(gst1, K, ur5)
pause(5)
disp('Moving to gst2');
fwdKinToolFrame = tf_frame('base_link', 'fwdKinToolFrame', eye(4));
fwdKinToolFrame.move_frame('base_link', gst2);
move_to_gst2 = ur5RRcontrol(gst2, K, ur5)
