function yawed_matrix = rotz(yaw)
    yawed_matrix = [cos(yaw) -sin(yaw) 0;
                    sin(yaw) cos(yaw) 0;
                    0 0 1];
end 