function updateQuat = buildUpdateQuat(deltaTheta)
%
% Builds the update quaternion from the minimally parametrized update
% See Indirect Kalman Filter for 3D Attitude Estimation (Roumeliotis)
%

    deltaq = 0.5 * deltaTheta;
    
    checkNorm = deltaq' * deltaq;
    
    if checkNorm > 1
        updateQuat = [deltaq; 1];
        updateQuat = updateQuat / sqrt(1 + checkNorm);
    else
        updateQuat = [deltaq; sqrt(1 - checkNorm)];
    end
    
    updateQuat = updateQuat / norm(updateQuat);
end