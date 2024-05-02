function pitched_matrix = roty(pitch)
    pitched_matrix = [cos(pitch) 0 sin(pitch);
                      0 1 0;
                      -sin(pitch) 0 cos(pitch)];
end 