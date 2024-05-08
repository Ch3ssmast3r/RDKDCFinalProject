function control_method = ask_control_method()
control_method_prompt = "Please select a control method: IK, RR, or TJ \n";
control_method = input(control_method_prompt, "s");
if strcmp(control_method, "IK")
    disp("Thank you for selecting Inverse Kinematic control");
    control_method = 1;
elseif strcmp(control_method, "RR")
    disp("Thank you for selecting Resolved Rate control");
    control_method =2;
elseif strcmp(control_method, "TJ")
    disp("Thank you for selecting Transpose Jacobian control");
    control_method = 3;
else
    error_message = "Invalid control method, please run the script again!";
    assert(false, 'MyException:EndOK', error_message);
end
end