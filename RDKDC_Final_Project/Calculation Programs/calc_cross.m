%% RDKDC Final Project - Calc cross
%
% Written by Aabhas Jain

% Inputs:
% center: center of x: x,y,z
% l: half length of 1 line of x. kind of like the radius
% v1: first base vector - deals with the rotation
% v2: second base vector - also deals with the rotation

% Purpose:
% calculates the list of points needed to draw an x
% Outputs:
% list of points: x,y
function cross = calc_cross(center, l, v1, v2)
    leg = l/sqrt(2);
    v1(3) = 0;
    v2(3) = 0;
    up_amount = 2/100; % 2cm
    v_up = [0;0;up_amount];
    cross(:, 1) = center + leg*v1 + leg*v2 + v_up;
    cross(:, 2) = cross(:, 1) - v_up;
    cross(:, 3) = center - leg*v1 - leg*v2;
    cross(:, 4) = cross(:, 3) + v_up;
    cross(:, 5) = center + leg*v1 - leg*v2 + v_up;
    cross(:, 6) = cross(:, 5) - v_up;
    cross(:, 7) = center - leg*v1 + leg*v2;
    cross(:, 8) = cross(:, 7) + v_up;
    %scatter3(cross(1,:), cross(2,:), cross(3,:))
end