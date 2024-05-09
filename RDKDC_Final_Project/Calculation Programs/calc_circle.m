%% RDKDC Final Project - Calc Circle
%
% Written by Aabhas Jain

% Inputs:
% center: center of circle: (x,y)
% r: circle radius

% Purpose:
% calculates the list of points needed to draw a circle.
% Outputs:
% list of points: x,y

function circle = calc_circle(center, r)
thetas = linspace(0, 2*pi, 30);
up_amount = 2/100; % 2cm
v_up = [0;0;up_amount];

circle(1, :) = center(1) + r*cos(thetas);
circle(2, :) = center(2) + r*sin(thetas);
circle(3, :) = center(3);
initial_point = circle(:, 1) + v_up;
end_point = circle(:, end) + v_up;
circle = [initial_point, circle, end_point];
% plot(circle(1,:), circle(2,:));
end