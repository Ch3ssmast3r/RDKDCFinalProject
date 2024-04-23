function[xi] = getXi(g)
%g is the homogeneous transformation matrix
R = g(1:3,1:3);
theta = acos((trace(R)-1)/2);
p = g(1:3,4);
if theta == 0
    omega = [0;0;0];
    v = p/norm(p);
    theta = norm(p);
else
    diff = R-transpose(R);
    differenceV = [diff(3,2);diff(1,3);diff(2,1)];
    omega = differenceV/(2*sin(theta));
    omegaHat = SKEW3(omega);
    A = eye(3)*theta + (1-cos(theta))*omegaHat +...
        (theta - sin(theta))*omegaHat*omegaHat;
    v = inv(A)*p;
en
xi = theta*[v;omega];