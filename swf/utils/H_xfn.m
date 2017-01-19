function [H_x] = H_xfn(kMinus1State, imuMeasurement, deltaT )
%H_XFN Compute the 6x6 matrix H_x_k  

%eTrans = kState.r_vi_i - (kMinus1State.r_vi_i + kMinus1State.C_vi'*d);
%eRotMat = kState.C_vi*(Phi*kMinus1State.C_vi)'

psiVec = imuMeasurement.omega*deltaT;
psiMag = norm(psiVec);
d = imuMeasurement.v*deltaT;

Psi = cos(psiMag)*eye(3) + (1 - cos(psiMag))*(psiVec/psiMag)*(psiVec/psiMag)' - sin(psiMag)*crossMat(psiVec/psiMag);

%Construct the H_x matrix
H_x = zeros(6,6);
H_x(1:3, 1:3) = eye(3);
H_x(1:3, 4:6) = -kMinus1State.C_vi'*crossMat(d);

H_x(4:6, 4:6) = Psi;



end

