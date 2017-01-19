function [errorVec] = stereoCamError(yMeas, kState, vehicleCamTransform, rho_pi_i, calibParams)
%STEREOCAMERROR Compute the 4x1 error vector associated with a landmark
%feature

C_cv = vehicleCamTransform.C_cv;
rho_cv_v = vehicleCamTransform.rho_cv_v;
C_vi = kState.C_vi;
r_vi_i = kState.r_vi_i;

%Equation 3.4 in the assignment
p_pc_c = C_cv*(C_vi*(rho_pi_i - r_vi_i) - rho_cv_v); 

% Compute the g function
x = p_pc_c(1);
y = p_pc_c(2);
z = p_pc_c(3);

c_u = calibParams.c_u;
c_v = calibParams.c_v;
f_u = calibParams.f_u;
f_v = calibParams.f_v;
b = calibParams.b;

ySim = (1/z)*[f_u*x; f_v*y; f_u*(x-b); f_v*y] + [c_u; c_v; c_u; c_v];

errorVec = yMeas - ySim;

end

