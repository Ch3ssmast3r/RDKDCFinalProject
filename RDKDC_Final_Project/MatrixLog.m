function so3 = MatrixLog(R)
% *** CHAPTER 3: RIGID-BODY MOTIONS ***
% Takes R (rotation matrix).
% Returns the corresponding so(3) representation of exponential 
% coordinates.

acosinput = (trace(R) - 1) / 2;
if acosinput >= 1
    so3 = zeros(3);

elseif acosinput <= -1
    if norm(1 + R(3, 3)) >= 1e-6 %if not zero/close to zero
        w = (1 / sqrt(2 * (1 + R(3, 3)))) * [R(1, 3); R(2, 3); 1 + R(3, 3)];

    elseif norm(1 + R(2, 2)) >= 1e-6
        w = (1 / sqrt(2 * (1 + R(2, 2)))) * [R(1, 2); 1 + R(2, 2); R(3, 2)];

    else
        w = (1 / sqrt(2 * (1 + R(1, 1)))) * [1 + R(1, 1); R(2, 1); R(3, 1)];

    end
    
    so3 = VecToso3(pi * w);

else
	theta = acos(acosinput);
    so3 = theta * (1 / (2 * sin(theta))) * (R - R');
end
end