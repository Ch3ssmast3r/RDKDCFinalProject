function finalerr = ur5_move_specified_control(gdesired, ur5, control_method)
if (control_method == 1)
    disp('using IK control');
    finalerr = ur5IKcontrol(gdesired, ur5);
elseif control_method == 2
    disp('using RR control');
    finalerr = ur5RRcontrol(gdesired, ur5);
elseif control_method == 3
    disp('using TJ control');
    finalerr = ur5TJcontrol(gdesired, ur5);
else
    error_message = "Invalid control method, please run the script again!";
    assert(false, 'MyException:EndOK', error_message);
end
R_desired = gdesired(1:3, 1:3);
p_desired = gdesired(1:3, 4);
gcurrent = ur5FwdKinDH(ur5.get_current_joints());
R_current = gcurrent(1:3, 1:3);
p_current = gcurrent(1:3, 4);
d_so3 = sqrt(trace((R_current-R_desired)*transpose(R_current-R_desired)));
d_r3 = norm(p_current - p_desired);
fprintf("SO3 error: %4.2f", d_so3);
fprintf("R3 error: %4.2f m", d_r3);
end