%% ur5_teach_points
% The purpose of this function is to teach the points the user specifies to
% the ur5. 
% Written by James Kaluna
function frames = ur5_teach_points(ur5)
    ur5.switch_to_pendant_control();
    disp('Please move the UR5 safely to the first point you would like!');
    waitforbuttonpress;
    theta_1 = ur5.get_current_joints();
    disp('Thank you for teaching the first point! Please move to the second point!');
    waitforbuttonpress;
    close all;
    theta_2 = ur5.get_current_joints();
    disp('Thank you!');
    frames(:,:, 1) = ur5FwdKin(theta_1);
    frames(:,:, 2) = ur5FwdKin(theta_2);
    close all;
end