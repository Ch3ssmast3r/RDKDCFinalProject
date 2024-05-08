%% RDKDC Final Project - Draw list
%
% Written by Aabhas Jain

% Inputs:
% list: list of x,y,z points to draw
% ur5: ur5 object so we can move the robot.

% Purpose:
% draws the list of points using the ur5

% Outputs:
% list of points: x,y,z

function draw_list(gstart, list, ur5)
    gdesired = gstart;
    for i = 1:length(list)
        gdesired(1, 4) = list(1, i);
        gdesired(2, 4) = list(2, i);
        gdesired(3, 4) = list(3, i);
        %  disp(gdesired);
        ur5RRcontrolSmooth(gdesired, ur5);
    end    
end