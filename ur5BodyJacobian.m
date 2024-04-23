function Jb = ur5BodyJacobian(q)
%Function accepts q: 6 × 1 joint space variable vector = [θ1, θ2, θ3, θ4,
%θ5, θ6]' where θn is the angle of joint n for n = 1, · · · , 6.
%Returns J: Body Jacobian, Jb_st (6 × 6 matrix)

%call ur5FwdKin.m to get gst, then get the Adjoint matrix
Ad_g = adjoint(gst);


