function[g] = RPtoG(R,p)
    %this function takes rotation matrix R and translation vector p and
    %puts them in the homogeneous transformation matrix. I'm lazy.
    g = [R,p;0,0,0,1];
end