function plotp3(p0, varargin)
% Function accepts a single 3x1 vector and plots the vector as a point,
% with an optional argument for coloring.

% p0: a 3x1 vector
% varargin: (optional) a color specification, either a MATLAB built-in
% character (e.g. 'r') or a RBG triplet of values between 0 and 1

% usage example:  
% plotp3([1; 0; 0]);
% plotp3([1; 0; 0], 'r');
% plotp3([1; 0; 0], rand(1, 3));

% Aug 15 1997  Louis Whitcomb Created and Written
% 5 Mar 2022 modified Yanzhou Wang

[rows, cols] = size(p0);
if ((rows ~= 3) || (cols ~= 1))
  error('plotp3 requires a 3x1 vector argument. Check your dimensions.');
end

if nargin == 2
    colorspec = varargin{1};
else
    % if user does not specify a colorspec, set it to black.
    colorspec = [0, 0, 0];
end

plot3(p0(1), p0(2), p0(3), "Color", colorspec, "Marker", 'o');