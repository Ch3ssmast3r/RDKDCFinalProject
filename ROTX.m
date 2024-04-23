function rolled_matrix = rotx(roll)
    rolled_matrix = [1 0 0;
                      0 cos(roll) -sin(roll);
                      0 sin(roll) cos(roll)];
end 