%% RDKDC Final Project - Calc grid centers
%
% Written by Aabhas Jain
% Math by Aabhas Jain

% Inputs:
% l: length of 1 side of the bounding box
% v1: first corner of the box
% v2: second corner of the box

% Purpose:
% calculates the list of points needed to draw the grid in order
% Outputs:
% list of points: x,y,z
function centers = calc_centers(v0, x, y, l)
centers = zeros(3, 9);
coeffs = [(1*l/6) (5*l/6);
          (3*l/6) (5*l/6);
          (5*l/6) (5*l/6);
          (1*l/6) (3*l/6);
          (3*l/6) (3*l/6);
          (5*l/6) (3*l/6);
          (1*l/6) (1*l/6);
          (3*l/6) (1*l/6);
          (5*l/6) (1*l/6)];
for i = 1:9
centers(:,i) = v0 + coeffs(i,1)*x + coeffs(i,2)*y;
end