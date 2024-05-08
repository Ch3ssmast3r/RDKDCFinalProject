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
    grid(:, 1) = + v_up;
    grid(:, 2) = grid(:, 1) - v_up;
    grid(:, 3) = ;
    grid(:, 4) = grid(:, 3) + v_up;
    grid(:, 5) =  + v_up;
    grid(:, 6) = grid(:, 5) - v_up;
    grid(:, 7) = ;
    grid(:, 8) = grid(:, 7) + v_up;
    grid(:, 9) = + v_up;
    grid(:, 10) = grid(:, 9) - v_up;
    grid(:, 11) = ;
    grid(:, 12) = grid(:, 12) + v_up;
    grid(:, 13) =  + v_up;
    grid(:, 14) = grid(:, 13) - v_up;
    grid(:, 15) = ;
    grid(:, 16) = grid(:, 16) + v_up;
end 