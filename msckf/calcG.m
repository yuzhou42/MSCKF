function G = calcG(imuState_k)
% Multiplies the noise vector in the linearized continuous-time
% error state model

    G = zeros(12,12);
    
    C_IG = quatToRotMat(imuState_k.q_IG);
    
    G(1:3,1:3) = -eye(3);
    G(4:6,4:6) = eye(3);
    G(7:9,10:12) = eye(3);
    G(10:12,7:9) = -C_IG';
    
end