% ur5_calcgridparameters
% Andrew Palacio

% much like ur5_calculate_lines, this function is going to calculate the
% parameters necessary to draw the initial tic tac toe box. 

% Necessary outputs: x,y,L,gOrigin, where:
% x and y are orthonormal unit vectors for the box
% L is the length of the box
% gOrigin is the rigid body transformation of the pen tip to the bottom
% left hand quarter of the grid

function [x,y,L,gOrigin] = ur5_calcgridparameters(gstart,gfinish)

    % gOrigin is simply the starting frame.
    gOrigin = gstart;

    %vectors v1 and v2 will represent the position of the PEN TIP at the
    %two corners of the box. This will be calculated using the translation
    %vectors in gstart and gfinish, as well as the pen tip transformation.

    %pen tip transformation goes from frame 6 (tool0) to 7 (pen tip).
    p67 = [0; -49/1000; 115/1000];

    %extract rotations and vectors from the frames
    vstart = gstart(1:3,4); 
    vfinish = gfinish(1:3,4);
    Rstart = gstart(1:3,1:3);
    Rfinish = gfinish(1:3,1:3);

    %calculate pen tip vectors using this formula: v_tool + R_tool*p67
    v1 = vstart + Rstart*p67;
    v2 = vfinish + Rfinish*p67;

    %calculate the translation from v1 to v2 IN THE XY PLANE.
    v12 = v2-v1;
    v12(3,1) = 0;
    
    %v12 is the diagonal of the box, so we can calculate the length like
    %this:
    L = norm(v12)/sqrt(2);

    %make a unit vector of v12. let's call it vcorner.
    vcorner = v12/norm(v12);

    %to find x and y, rotate the unit vector vcorner by +/- pi/4 radians
    %about the z axis. 
    x = ROTZ(-pi/4)*vcorner;
    y = ROTZ(pi/4)*vcorner;

    if L < 75/1000 %Set mininum size for grid
        error_message = "ERROR - Grid is too small. Try again.";
        assert(false, 'MyException:EndOK', error_message);
    end
end
