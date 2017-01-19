function F = calcF(imuState_k, measurements_k)
%误差状态更新矩阵
%对应论文公式23：“The Battle for Filter Supremacy: A Comparative Study of the
%      Multi-State Constraint Kalman Filter and the Sliding Window Filter”

% Multiplies the error state in the linearized continuous-time
% error state model

    F = zeros(12,12);
    
    omegaHat = measurements_k.omega - imuState_k.b_g;
    vHat = measurements_k.v - imuState_k.b_v;
    C_IG = quatToRotMat(imuState_k.q_IG);
    
    F(1:3,1:3) = -crossMat(omegaHat);
    F(1:3,4:6) = -eye(3);
    F(10:12,1:3) = -C_IG' * crossMat(vHat);
    F(10:12,7:9) = -C_IG';
  
end