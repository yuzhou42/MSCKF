function [G_x, G_x_f] = G_xfn(kState, vehicleCamTransform, rho_pi_i, calibParams, useMonoCamera)
%G_XFN Calculates 4x6 G_x matrix for the stereo camera model

C_cv = vehicleCamTransform.C_cv;
rho_cv_v = vehicleCamTransform.rho_cv_v;
C_vi = kState.C_vi;
r_vi_i = kState.r_vi_i;

%Calculate the nominal feature vector
p_pc_c_bar = C_cv*(C_vi*(rho_pi_i - r_vi_i) - rho_cv_v); 


%Caculate the Jacobian of the transformation (L) and then pass it through
%the model g

L = [-C_cv*C_vi, C_cv*crossMat(C_vi*(rho_pi_i - r_vi_i))];
J = gJacob(p_pc_c_bar, calibParams);

%Output the jacobian with respect to the state and the feature position
if useMonoCamera
    G_x = J(1:2,:)*L;
    G_x_f = J(1:2,:)*C_cv*C_vi;
else
    G_x = J*L;
    G_x_f = J*C_cv*C_vi;
end

end

