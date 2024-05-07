function frames = ur5_calculate_lines(gstart,gfinish)
    %Andrew Palacio
    
    %this function inputs the frame transformations ALREADY INCLUDING the pen
    %tip transformation of the points 1 and 4 for drawing two parallel
    %straight lines. This will then output the frame transformations STILL
    %INCLUDING the pen tip transformation for the points 2 and 3.
    
    %NECESSARY CHANGES
    %ask the user to confirm that they like the two selected points
    
    %we'll need this vector
    e3 = [0;0;1];

    %if an error occurs, the 4 frames will be set to the starting config.
    starting_config = [0.8996; -1.7011; 2.4081; -2.2778; -1.5627; 0.1232];
    gerror = ur5FwdKinDH(starting_config);
    
    %Go to this starting configuration in RPY form:
    %Rx = -180
    %Ry = 0
    %Rz = 135
    %the order is Rzyx
    
    % R = ROTZ(3*pi/4)*ROTY(0)*ROTX(-pi);
    
    %JUST KIDDING. The orientation of the first trained point will be used
    %for frames 1 and 2. The orientation of the second trained point will
    %be used for frames 3 and 4. 

    %extract rotation matrices
    R12 = gstart(1:3,1:3);
    R34 = gfinish(1:3,1:3);

    %extract the vectors from the homogeneous frames
    v1 = gstart(1:3,4); 
    v4 = gfinish(1:3,4);
    
    %To calculate points 2 and 3, we will consider the average z-position
    %of the existing points 1 and 4, to guarantee that the pen remains on
    %the paper. Then the xy positions of points 2 and 3 will be considered
    %using purely the x-y coordinates of vectors v1 and v2.
    
    zavg = (v1(3,1)+v4(3,1))/2;
    v1_xy = [v1(1:2,1);zavg];
    v4_xy = [v4(1:2,1);zavg];
    
    %compute the distance between v1 and v4, projected in the xy plane.
    v14 = v4_xy - v1_xy;
    
    %make sure that points 1 and 4 aren't the same. If so, break the
    %function.
    if norm(v14)< .01 %threshold set to 1cm
        disp('Error - Start and end positions are too close. Moving to starting configuration.');
        g1 = gerror(4);
        g2 = gerror(4);
        g3 = gerror(4);
        g4 = gerror(4);
        
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
    
    %calculate the position of v2 and v4 using the 5cm distance.
    v2 = v1_xy + (v12)*5/100; %distance is expressed in meters
    v3 = v4_xy + (v12)*5/100;
    
    %output the frames g2 and g3, using the orientation expressed in g4.
    g2 = [R12,v2;0,0,0,1];
    g3 = [R34,v3;0,0,0,1];
    
    %frames g1 and g4 will be re-expressed with the average z height.
    g1 = [R12,v1_xy;0,0,0,1];
    g4 = [R34,v4_xy;0,0,0,1];
    
    %plot the proposed points and ask if the user likes them
    figure (1)
    hold on
    plotp3(v1_xy)
    plotp3(v2)
    plotp3(v3)
    plotp3(v4_xy)
    hold off
    
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