function size = calc_shape_size(L)
    %This function inputs the total side length of the tic tac toe grid, L,
    %and calculates the size needed for an X or an O.
    boxSize = L/3; %Size of a single box
    size = (0.7*boxSize)/2;
end