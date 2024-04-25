%% Part C - Testing manipulability measures
%This should be added to main lab3 code later.

%This part will calculate the manipulability near a singularity, more
%specifically when theta3 = 0.

q = [pi/3;-pi/3;NaN;pi/4;pi/6;-pi/6]; %joint space variables. Theta 3 will be left blank for now.
theta3 = (-pi/4:.01:pi/4); %array of values of theta3 near the singularity, theta3 = 0

Q = zeros(6,length(theta3)); %pre-define these arrays
muSigmaMin = zeros(length(theta3),1);
muDetJac = zeros(length(theta3),1);
muInvCond = zeros(length(theta3),1);
for i = 1:length(theta3) %we will calulate manipulability for each value of theta3.
    Q(:,i)=q; %start by placing the other 5 joint space variables.
    Q(3,i)=theta3(i); %fill in theta3
    
    %calculate the Jacobian for each qi
    J = ur5BodyJacobian(Q(:,i));
    
    %calculate each of the 3 manipulability measures: 'sigmamin','detjac',or'invcond'
    muSigmaMin(i) = manipulability(J,'sigmamin');
    muDetJac(i) = manipulability(J,'detjac');
    muInvCond(i) = manipulability(J,'invcond');
end

%generate plots of manipulability, mu, vs theta.
figure(1)
plot(theta3,muSigmaMin)
title("Singularity: Minimum Singular value")
xlabel('Theta3, (rad)')
ylabel('Manipulability, mu')

figure(2)
plot(theta3,muDetJac)
title("Signularity: Determinant of Jacobian")
xlabel('Theta3 (rad)')
ylabel('Manipulability, mu')

figure(3)
plot(theta3,muInvCond)
title("Singularity: Inverse Condition")
xlabel('Theta3 (rad)')
ylabel('Manipulability, mu')