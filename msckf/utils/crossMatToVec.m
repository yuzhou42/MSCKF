function vec = crossMatToVec(crossMat)
%
% Extracts a 3x1 vector from a cross-product matrix
%
% crossMat           - 3x3 skew-symmetric matrix
%
    vec = [crossMat(3,2); crossMat(1,3); crossMat(2,1)];
end