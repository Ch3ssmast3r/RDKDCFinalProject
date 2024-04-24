function[g] = XiToG(xi)
    %this function takes a 6-dimensional vector xi in the form of [v;w] and
    %calculates the homogeneous transformation matrix. 
    
    %first extract v and w from xi
    v = xi(1:3);
    w = xi(4:6);
    
    %there is a translational case and a general case.
    
    %Case 1: Purely Translational
    if norm(w) == 0
        g = [eye(3),v;0,0,0,1];
    else
    
    %Case 2: General
        w_hat = SKEW3(w);%calculate omega-Hat

        exponential = EXPCR(w); %calculate the rotation matrix R
        g = [exponential, (eye(3) - exponential)*(w_hat*v) + w*transpose(w)*v;
            zeros(1,3) 1]; %this is the homogeneous transformation matrix
    end
end