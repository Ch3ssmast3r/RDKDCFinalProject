
function  R = MatrixExp(skewMat)
% Takes a 3x3 so(3) representation of exponential coordinates.
% Returns R in SO(3) that is achieved by rotating about omghat by theta 
% from an initial orientation R = I.


omgtheta = [skewMat(3,2); skewMat(1,3); skewMat(2,1)];
if norm(omgtheta) < 1e-6
    R = eye(3);
else
    theta = norm(omgtheta);
    omega_hat = skewMat / theta;
    R = eye(3) + sin(theta) * omega_hat + (1 - cos(theta)) * omega_hat * omega_hat;
end
end
