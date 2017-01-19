function [ newState ] = propagateState(oldState, imuMeasurement, deltaT)
%PROPAGATESTATE Propagates the state forward in time

psiVec = imuMeasurement.omega*deltaT;
psiMag = norm(psiVec);
d = imuMeasurement.v*deltaT;

Phi = cos(psiMag)*eye(3) + (1 - cos(psiMag))*(psiVec/psiMag)*(psiVec/psiMag)' - sin(psiMag)*crossMat(psiVec/psiMag);

newState.C_vi = Phi*oldState.C_vi;
newState.r_vi_i = oldState.r_vi_i + oldState.C_vi'*(d);
newState.k = oldState.k + 1;

end

