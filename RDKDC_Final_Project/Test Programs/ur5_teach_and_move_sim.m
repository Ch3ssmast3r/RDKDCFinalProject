%% RDKDC Final Project - Teach and Move
% Written by Aabhas Jain 


% Ask user to specify control method
control_method = ask_control_method();

%% Teach and calculate robot points
ur5 = ur5_interface(); % starting the ur5 interface

% pre-defined starting configuration - empirically set. 
starting_config = [0.8996; -1.7011; 2.4081; -2.2778; -1.5627; 0.1232];

% only moving to start configuration if we are not near it. 
if (norm(ur5.get_current_joints() - starting_config) > 0.1)
    disp('Moving to starting_config');
    ur5.move_joints(starting_config, 15);
    pause(15)
end

% teach the points we want to go to. 
gst1 = [0 -1 0 0.2;
       -1 0 0 0.60;
        0 0 -1 0.05;
        0 0 0 1];
gst2 = [0 -1 0 0.4;
        -1 0 0 0.7;
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

% 1. assume that we are touching the paper, so we move up.
disp('moving up');
up_displacement = zeros(4);
up_displacement(3, 4) = 0.05;
ur5FwdKinDH(ur5.get_current_joints);
up_frame = ur5FwdKinDH(ur5.get_current_joints()) + up_displacement;
ur5_move_specified_control(up_frame, ur5, control_method);

% 2. move to be above the first frame. 
disp('moving to start location');
up_frame = gst1 + up_displacement;
ur5_move_specified_control(up_frame, ur5, control_method);

% 3. put the pen on the paper. 
disp('Moving to gst1');
pen_frame = tf_frame('base_link', 'pen_frame', eye(4));
pen_frame.move_frame('base_link', gst1);
move_to_gst1 = ur5_move_specified_control(gst1, ur5, control_method);

% 4. draw the first line
disp('Moving to gst2');
pen_frame = tf_frame('base_link', 'pen_frame', eye(4));
pen_frame.move_frame('base_link', gst2);
move_to_gst2 = ur5_move_specified_control(gst2, ur5, control_method);

% 5. is moving back up so we don't draw
disp('moving up');
ur5FwdKinDH(ur5.get_current_joints);
up_frame = ur5FwdKinDH(ur5.get_current_joints()) + up_displacement;
ur5_move_specified_control(up_frame, ur5, control_method);

% 6. move to above the point we start the second line at
disp('moving to next point')
next_frame = gst3;
next_frame(3, 4) = up_frame(3,4); % need to change the height
move_to_next_frame = ur5_move_specified_control(next_frame, ur5, control_method);
 
% 7. put pen to paper for second line
disp('Moving to gst3');
pen_frame = tf_frame('base_link', 'pen_frame', eye(4));
pen_frame.move_frame('base_link', gst3);
move_to_gst3 = ur5_move_specified_control(gst3, ur5, control_method);

% 8. draw second line
disp('Moving to gst4');
pen_frame = tf_frame('base_link', 'pen_frame', eye(4));
pen_frame.move_frame('base_link', gst4);
move_to_gst4 = ur5_move_specified_control(gst4, ur5, control_method);

% 9. move upwards before going to starting config. 
disp('done, moving upwards');
up_frame = ur5FwdKinDH(ur5.get_current_joints()) + up_displacement;
ur5_move_specified_control(up_frame, ur5, control_method);

% 10. move to starting config just to reset
ur5.move_joints(starting_config, 15);
pause(15);
disp('Program complete!')