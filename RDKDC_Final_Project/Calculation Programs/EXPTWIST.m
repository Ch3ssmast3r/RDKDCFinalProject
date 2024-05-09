%% Function to calculate the exponential revolute twist coordinates
function exp_twist = EXPTWIST(w, q, theta)
    twist = [-cross(w,q); w]; %v, w
    v = twist(1:3);
    w_hat = SKEW3(w);
    exponential = EXPCR(w*theta);
    exp_twist = [exponential, (eye(3) - exponential)*(w_hat*v) + w*transpose(w)*v*theta;
        zeros(1,3) 1];

end