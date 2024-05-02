function expcr = EXPCR(x)
    % used equation on page 28.
    norm_x = norm(x);
    if norm_x == 0
        expcr = eye(3);
    else 
    skew_x = SKEW3(x);
    expcr = eye(3) + (skew_x/norm_x)*(sin(norm_x)) + ... 
            (skew_x^2/norm_x^2)*(1-cos(norm_x));
    end
end