%% Testing Resolved Rate Controller
% The purpose of this program is to test our current resolved rate
% controller for moving between the two provided frames. 

ur5 = ur5_interface();
starting_config = [0; -pi/2; pi/2; 0; pi/6; 0];
theta1 = [0; -pi/2; -pi/2; 0; pi/6; 0];
g1 = ur5FwdKin(theta1);
%starting_frame = ur5FwdKin(starting_config);
gst1 = [0 -1 0 0.25;
        -1 0 0 0.60;
        0 0 -1 0.22;
        0 0 0 1];
gst2 = [0 -1 0 0.40;
        -1 0 0 0.60;
        0 0 -1 0.22;
        0 0 0 1];
%fwdKinToolFrame.move_frame('base_link', starting_frame);
ur5.move_joints(starting_config, 10);
pause(1);
disp('Moving to gst1');
K = 0.8;
fwdKinToolFrame = tf_frame('base_link', 'fwdKinToolFrame', eye(4));
fwdKinToolFrame.move_frame('base_link', g1);
move_to_gst1 = ur5RRcontrol(g1, K, ur5)
disp('Moving to gst2');
fwdKinToolFrame = tf_frame('base_link', 'fwdKinToolFrame', eye(4));
fwdKinToolFrame.move_frame('base_link', gst2);
move_to_gst2 = ur5RRcontrol(gst2, K, ur5)
