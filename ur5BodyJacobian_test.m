%ur5BodyJacobian test
q = [0; 0; 0; 0; 0; 0]; % joint angles
Jb = ur5BodyJacobian(q); %calculate Body Jocabian

%define e_i's
e1 = [1 0 0 0 0 0]'; e2 = [0 1 0 0 0 0]'; e3 = [0 0 1 0 0 0]'; 
e4 = [0 0 0 1 0 0]'; e5 = [0 0 0 0 1 0]'; e6 = [0 0 0 0 0 1]';

e = [e1, e2, e3, e4, e5, e6];

eps = 0.0000001; %slight offset

gst = ur5FwdKin(q);
gst_inv = FINV(gst);

Japprox = zeros(6); %initialize Jacobian approximation

for i = 1:6
    upper = ur5FwdKin(q+eps*e(:,i));
    lower = ur5FwdKin(q-eps*e(:,i));
    dg_dqi = (1/(2*eps))*(upper-lower);
    Japprox(:,i) = twistify(gst_inv*dg_dqi);
end

%compute matrix norm of the error btn Japprox and Jb
err_norm = norm(Japprox-Jb);
disp(err_norm)
     
