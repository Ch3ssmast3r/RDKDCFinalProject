function frames = ur5_calculate_lines(gstart,gfinish)
    %Andrew Palacio
    
    %this function inputs the frame transformations ALREADY INCLUDING the pen
    %tip transformation of the points 1 and 4 for drawing two parallel
    %straight lines. This will then output the frame transformations STILL
    %INCLUDING the pen tip transformation for the points 2 and 3.
    
    %we'll need this vector
    e3 = [0;0;1];

    %pen tip transformation goes from frame 6 (tool0) to 7 (pen tip).
    R67 = eye(3);
    p67 = [0; -49/1000; 115/1000];
    t67 = [R67, p67; 0, 0, 0, 1];

    %if an error occurs, the 4 frames will be set to the starting config.
    starting_config = [0.8996; -1.7011; 2.4081; -2.2778; -1.5627; 0.1232];
    gerror = ur5FwdKinDH(starting_config);
    
    %The orientation of the first trained point will be used
    %for frames 1 and 2. The orientation of the second trained point will
    %be used for frames 3 and 4. 

    %extract rotation matrices for the starting configuration.
    R12 = gstart(1:3,1:3); % this is in base -> tool0
    R34 = gfinish(1:3,1:3); % this in base -> tool0

    %extract the vectors from the homogeneous frames
    v1 = gstart(1:3,4); 
    v4 = gfinish(1:3,4);

    %v1 and v4 are the points of the TOOL frame when the robot is taught,
    %but we need v1 and v4 of the PEN TIP, expressed in spacial frame.

    %we use the equation vpen = v_tool + R_tool*p67
    v1pen = v1 + R12*p67;
    v4pen = v4 + R34*p67;

    %To calculate points 2 and 3, we will consider the z-position and
    %configuration of the starting trained point, to guarantee that the
    %pen will stay on the paper.
    %Then the xy positions of points 2 and 3 will be considered
    %using purely the x-y coordinates of vectors v1pen and v4en.
    
    v1_xy = [v1pen(1:2,1);0];
    v4_xy = [v4pen(1:2,1);0];
    
    %compute the distance between v1 and v4, projected in the xy plane.
    v14 = v4_xy - v1_xy;
    
    %make sure that points 1 and 4 aren't the same. If so, break the
    %function.
    if norm(v14)< .01 %threshold set to 1cm
        disp('Error - Start and end positions are too close. Moving to starting configuration.');
        g1 = gerror;
        g2 = gerror;
        g3 = gerror;
        g4 = gerror;
        
        frames(:,:,1) = g1;
        frames(:,:,2) = g2;
        frames(:,:,3) = g3;
        frames(:,:,4) = g4;
        return
    end
    
    %make it a unit vector
    v14 = v14/norm(v14);
    
    %compute v12, vector that goes from point 1 to 2, using vector cross
    %product with e3, so that v12 is orthogonal with v14.
    v12 = cross(v14,e3);
    
    %calculate the position of v2 and v3 using the 5cm distance
    %v2 is calculated wrt v1 because the tool will be translated in
    %the same orientation as in g1.
    %v3 is calculated wrt v4 because the tool will be translated in the
    %same orientation as in g4.

    v2 = v1 + (v12)*5/100; %distance is expressed in meters
    v3 = v4 + (v12)*5/100;
    
    %output the frames g1-g4. 1 and 2 will be in starting training config.
    %3 and 4 will be in final training config. 

    g1 = [R12,v1;0,0,0,1];
    g2 = [R12,v2;0,0,0,1];
    g3 = [R34,v3;0,0,0,1];
    g4 = [R34,v4;0,0,0,1];

    
    %plot is for troubleshooting purposes.
    % figure (1)
    % hold on
    % plotp3(v1,'b')
    % plotp3(v2,'b')
    % plotp3(v3,'b')
    % plotp3(v4,'b')
    % plotp3(v1pen(1:3),'r')
    % plotp3(v4pen(1:3),'r')
    % xlim([.5 1.5])
    % ylim([0 1])
    % hold off
    
    accept = input('Is this the desired position? Input 1 for YES, 0 for NO. ');
    if accept ~= 1 %check whether desired position was rejected.
        disp('Error - User denied the proposed positions. Moving to starting configuration.');
        g1 = gerror;
        g2 = gerror;
        g3 = gerror;
        g4 = gerror;
    end
    
    frames(:,:,1) = g1;
    frames(:,:,2) = g2;
    frames(:,:,3) = g3;
    frames(:,:,4) = g4;
    close all
end