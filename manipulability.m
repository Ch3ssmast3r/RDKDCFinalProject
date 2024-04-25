function[mu] = manipulability(J,measure)
    %the purpose of this function is to calculate the manipulability of the
    %inputted jacobian using one of three methods, selected by the user:
    %1) minimum singular value of J
    %2) inverse condition of the number of J
    %3) Determinant of J
    
    %J is a 6x6 matrix, measure is a string that must be
    %'sigmamin','detjac',or'invcond'
    
    if (ismember(measure,'sigmamin')|ismember(measure,'invcond'))
        %the "ismember" function checks whether the variable measure contains the string 'sigmamin'.
        %for both 'sigmamin' and 'invcondition', we must compute the
        %singular values of J.
       
        sigmas = svd(J); %array of singular values
        sigmaMin = min(sigmas); %smallest singular value
        mu = sigmaMin; %this is our output for the measurement 'sigmamin'.
        
        if ismember(measure,'invcond') %this measure takes the ratio of the min. and max. singular values.
            sigmaMax = max(sigmas); %largest singular value
            mu = sigmaMin/sigmaMax; 
        end
        
    elseif ismember(measure,'detjac') 
        %this measure of manipulability simply uses the determinant of the
        %jacobian, which is the product of the singular values of J.
        mu = det(J);
        
    else %final condition if user inputs an invalid manipulability method name.
        fprintf("Invalid desired manipulability input");
        mu = NaN;
    end
end