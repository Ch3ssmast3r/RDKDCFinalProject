%% RDKDC Final Project - Calc grid
%
% Written by Aabhas Jain
% Math by Andrew Palacio

% Inputs:
% l: length of 1 side of the bounding box
% v1: first corner of the box 
% v2: second corner of the box

% Purpose:
% calculates the list of points needed to draw the grid in order 
% Outputs:
% list of points: x,y,z 
function grid = calc_grid(v0, x, y, l)
    up_amount = 2/100; % 2cm
    v_up = [0;0;up_amount];
    %v1
    grid(:, 1) = v0 + (l/3)*x + v_up;
    grid(:, 2) = grid(:, 1) - v_up;
    %v2
    grid(:, 3) = v0 + (l/3)*x + l*y;
    grid(:, 4) = grid(:, 3) + v_up;
    %v3
    grid(:, 5) = v0 + (2*l/3)*x + l*y + v_up;
    grid(:, 6) = grid(:, 5) - v_up;
    %v4
    grid(:, 7) = v0 + (2*l/3)*x;
    grid(:, 8) = grid(:, 7) + v_up;
    %v5
    grid(:, 9) = v0 + l*x + (l/3)*y + v_up;
    grid(:, 10) = grid(:, 9) - v_up;
    %v6
    grid(:, 11) = v0 + (l/3)*y;
    grid(:, 12) = grid(:, 11) + v_up;
    %v7
    grid(:, 13) = v0 + (2*l/3)*y + v_up;
    grid(:, 14) = grid(:, 13) - v_up;
    %v8
    grid(:, 15) = v0 + l*x + (2*l/3)*y;
    grid(:, 16) = grid(:, 15) + v_up;
    %plot3(grid(1, :), grid(2, :), grid(3,:));
end 