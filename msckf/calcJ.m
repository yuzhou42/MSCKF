function J = calcJ(camera, imuState_k, camStates_k)
%函数功能：得到要增广的状态（相机位置、相机四元数）对之前已有状态的雅克比

% 参考文献：“The Battle for Filter Supremacy: A Comparative Study of the
%    Multi-State Constraint Kalman Filter and the Sliding Window Filter”

% Jacobian of feature observations w.r.t. feature locations

    C_CI = quatToRotMat(camera.q_CI);
    C_IG = quatToRotMat(imuState_k.q_IG);
    
    %参考文献 公式10
    %增广的相机位置与四元数，对增广后的状态的偏导（雅克比）
    % (camera_p_k+1 camera_q_k+1) = J * (imu_q_k, imu_b_w_k, imu_b_v_k, imu_p_k, camera_p_k, camera_q_k)
    J = zeros(6, 12 + 6*size(camStates_k,2));
    J(1:3,1:3) = C_CI;
    J(4:6,1:3) = crossMat(C_IG' * camera.p_C_I);
    J(4:6,10:12) = eye(3);

end