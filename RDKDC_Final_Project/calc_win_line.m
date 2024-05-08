%% RDKDC Final Project - Calc win_line
%
% Written by Aabhas Jain

% Inputs:
% c1: center of first winning square
% c2: center of second winning square

% Purpose:
% calculates the list of points needed to draw a line
% Outputs:
% list of points: x,y
function line = calc_win_line(c1, c2)
    up_amount = 2/100; % 2cm
    v_up = [0;0;up_amount];
    line(:, 1) = c1 + v_up;
    line(:, 2) = line(:, 1) - v_up;
    line(:, 3) = c2;
    line(:, 4) = line(:, 3) + v_up;
end