%% RDKDC Final Project - Teach and Move
% Written by Aabhas Jain 
% edited by James Kaluna

%somehow doesn't work with ur5FwdKinDH
%% Calculate Robot points
ur5 = ur5_interface();

starting_config = [0.8996; -1.7011; 2.4081; -2.2778; -1.5627; 0.1232];
% ur5.switch_to_ros_control()
if (norm(ur5.get_current_joints() - starting_config) > 0.1)
    disp('Moving to starting_config');
    ur5.move_joints(starting_config, 15);
    pause(15)
end
% first teach the points we want to go to. 
gst1 = [0 -1 0 0.2;
       -1 0 0 0.60;
        0 0 -1 0.05;
        0 0 0 1];
gst2 = [0 -1 0 0.35;
        -1 0 0 0.60;
        0 0 -1 0.05;
        0 0 0 1];
frames(:,:,1) = gst1;
frames(:,:,2) = gst2;

% second compute all of the target frames (4 frames)
target_frames = ur5_calculate_lines(frames(:,:,1), frames(:,:, 2));

%% Start robot motions
% This starting configuration was empirically determined.
gst1 = target_frames(:,:,1);
gst2 = target_frames(:,:,2);
gst3 = target_frames(:,:,3);
gst4 = target_frames(:,:,4);

K = 1.5;
disp('moving to start location');
up_displacement = zeros(4);
up_displacement(3, 4) = 0.05;
up_frame = gst1 + up_displacement;
move_to_up_frame = ur5IKcontrol(up_frame, ur5);

K = 5;
disp('Moving to gst1');
pen_frame = tf_frame('base_link', 'pen_frame', eye(4));
pen_frame.move_frame('base_link', gst1);
move_to_gst1 = ur5IKcontrol(gst1, ur5);

disp('Moving to gst2');
pen_frame = tf_frame('base_link', 'pen_frame', eye(4));
pen_frame.move_frame('base_link', gst2);
move_to_gst2 = ur5IKcontrol(gst2, ur5);

disp('moving up');
up_displacement = zeros(4);
up_displacement(3, 4) = 0.05;
ur5FwdKin(ur5.get_current_joints)
up_frame = ur5FwdKin(ur5.get_current_joints()) + up_displacement;
move_to_up_frame = ur5IKcontrol(up_frame, ur5);

disp('moving to next point')
next_frame = target_frames(:,:,3);
next_frame(3, 4) = up_frame(3,4);
move_to_next_frame = ur5IKcontrol(next_frame, ur5);

disp('Moving to gst3');
pen_frame = tf_frame('base_link', 'pen_frame', eye(4));
pen_frame.move_frame('base_link', gst3);
move_to_gst3 = ur5IKcontrol(gst3, ur5);
disp('Moving to gst4');
pen_frame = tf_frame('base_link', 'pen_frame', eye(4));
pen_frame.move_frame('base_link', gst4);
move_to_gst4 = ur5IKcontrol(gst4, ur5);

disp('done, moving to starting config');
up_displacement = zeros(4);
up_displacement(3, 4) = 0.05;
up_frame = ur5FwdKin(ur5.get_current_joints()) + up_displacement;
move_to_up_frame = ur5IKcontrol(up_frame, ur5);

ur5.move_joints(starting_config, 15);
pause(15);