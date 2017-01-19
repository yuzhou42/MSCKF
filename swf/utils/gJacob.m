function [gJacobMat] = gJacob(p_pc_c, calibParams)
%GJACOBMAT Returns the 4x3 Jacobian of the observation model with respect
%to the feature position in the camera frame

%Redefine local shorthand notation
x = p_pc_c(1);
y = p_pc_c(2);
z = p_pc_c(3);

c_u = calibParams.c_u;
c_v = calibParams.c_v;
f_u = calibParams.f_u;
f_v = calibParams.f_v;
b = calibParams.b;

%Evaluate the Jacobian
gJacobMat = [f_u/z, 0, -(1/z^2)*(f_u*x )
             0, f_v/z, -(1/z^2)*(f_v*y )
             f_u/z, 0, -(1/z^2)*(f_u*(x-b) )
             0, f_v/z, -(1/z^2)*(f_v*y )];
end