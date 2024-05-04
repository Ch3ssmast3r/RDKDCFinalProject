function[xi] = getXi(g)
%g is the homogeneous transformation matrix in SE(3)
R = g(1:3,1:3); %extract the rotation matrix from g
theta = acos((trace(R)-1)/2); %theta is dependent upon the rotation matrix
% fprintf('Theta: %6.4f \n',theta);
p = g(1:3,4); %extract the position vector from g

%We must account for two cases: 1) purely translational and 2) general case
if abs(theta) < 0.0001 %purely translational case. v is simply dependent on the p vector.
    omega = [0;0;0];
    v = p/norm(p);
    theta = norm(p);
else %more general case. 
    difference = R-transpose(R); %This is needed to calculate omega, using the general equation
    differenceV = [difference(3,2);difference(1,3);difference(2,1)];
    omega = differenceV/(2*sin(theta));
    %next we compute v. the equation is summarized in this matrix (A) such
    %that vA = p.
    omegaHat = SKEW3(omega);
    R = EXPCR(omega*theta);
    A = (eye(3)-R)*omegaHat+omega*transpose(omega)*theta;
    
    %this part isn't working.
%     A = eye(3)*theta + (1-cos(theta))*omegaHat +...
%         (theta - sin(theta))*omegaHat*omegaHat;
    v = inv(A)*p; %solve for v
end
xi = theta*[v;omega]; %we are interested in a non-normalized xi, which is why we...
%multiply the unit twist by theta.