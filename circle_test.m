%% RDKDC Final Project - testing draw circle
% This is a program to test how well we can draw circles.
% Written by Aabhas Jain 

ur5 = ur5_interface(); % starting the ur5 interface

starting_config = [0.8996; -1.7011; 2.4081; -2.2778; -1.5627; 0.1232];
control_method = 1;

% only moving to start configuration if we are not near it. 
if (norm(ur5.get_current_joints() - starting_config) > 0.1)
    ur5.switch_to_ros_control();
    disp('Moving to starting_config');
    ur5.move_joints(starting_config, 15);
    pause(15)
end

% teach the points we want to go to. 
gst1 = [0 -1 0 0.12;
       -1 0 0 0.33;
        0 0 -1 0.11;
        0 0 0 1];
gst2 = [0 -1 0 0.32;
       -1 0 0 0.33;
        0 0 -1 0.11;
        0 0 0 1];
ur5.switch_to_pendant_control();
frames = ur5_teach_points(ur5);
ur5.switch_to_ros_control();
[x,y, l, g0] = ur5_calcgridparameters(frames(:,:,1), frames(:,:,2));
grid = calc_grid(g0(1:3, 4), x, y, l);
disp('moving up');
up_displacement = zeros(4);
up_displacement(3, 4) = 0.05;
ur5FwdKinDH(ur5.get_current_joints);
up_frame = ur5FwdKinDH(ur5.get_current_joints()) + up_displacement;
move_to_up_frame = ur5RRcontrolSmooth(up_frame, ur5);

draw_list(g0, grid, ur5);
disp('Program done!');
% 
% % 3. put the pen on the paper. 
% gst2 = gst1;
% gst2(3, 4) = gst2(3,4) + 0.05;
% disp('Moving to gst2');
% pen_frame = tf_frame('base_link', 'pen_frame', eye(4));
% pen_frame.move_frame('base_link', gst2);
% move_to_gst2 = ur5_move_specified_control(gst2, ur5, control_method);
% 
% % disp('Moving to gst1');
% % pen_frame = tf_frame('base_link', 'pen_frame', eye(4));
% % pen_frame.move_frame('base_link', gst1);
% % move_to_gst1 = ur5_move_specified_control(gst1, ur5, control_method);
% 
% circle = calc_circle(gst1(1:3, 4), 0.025);
% draw_list(circle, ur5);
% cross = calc_cross(gst1(1:3,4), 0.025, [1;0], [0;1]);
% draw_list(cross, ur5);
% disp('Program done!');