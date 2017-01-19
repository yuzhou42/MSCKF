function [C] = Cfrompsi(psiVec)
%CFROMPSI Create a 3x3 rotation matrix from a 3x1 axis-angle parameters

psiMag = norm(psiVec);

C = cos(psiMag)*eye(3) + (1 - cos(psiMag))*(psiVec/psiMag)*(psiVec/psiMag)' - sin(psiMag)*crossMat(psiVec/psiMag);

end

