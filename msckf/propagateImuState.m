function imuState_prop = propagateImuState(imuState_k, measurements_k)
%函数功能：根据IMU测量值更新IMU的状态（四元数 陀螺仪零偏与速度零偏 位置）
%
%返回值：
%      imuState_prop：IMU部分的状态量更新后的结果
%输入值：
%      imuState_k：上一次的IMU状态
%      measurements_k：当次IMU测量值

% prop == propagated to k+1

    C_IG = quatToRotMat(imuState_k.q_IG);
    
    % Rotation state
    %IMU中陀螺仪测量值（去除零偏）积分得到角增量，psi = (w-bias)*dt
    psi = (measurements_k.omega - imuState_k.b_g) * measurements_k.dT;
    %用陀螺仪测量的角速度更新四元数
    %　|q1|    |q1|      ｜ 0    wz  -wy  wx ||q1|  
    %　|q2| := |q2| + 1/2｜-wz   0    wx  wy ||q2|dt
    %　|q3|    |q3|      ｜ wy  -wx   0   wz ||q3|
    %　|q0|    |q0|      ｜-wx  -wy  -wz   0 ||q0|
    imuState_prop.q_IG = imuState_k.q_IG + 0.5 * omegaMat(psi) * imuState_k.q_IG;
%     diffRot = axisAngleToRotMat(psi);
%     C_IG_prop = diffRot * C_IG;
%     imuState_prop.q_IG = rotMatToQuat(C_IG_prop);
    
    %Unit length quaternion
    %归一化四元数
    imuState_prop.q_IG = imuState_prop.q_IG/norm(imuState_prop.q_IG);
    
    % Bias states
    %陀螺仪零偏与速度零偏的更新矩阵为单位矩阵
    imuState_prop.b_g = imuState_k.b_g;
    imuState_prop.b_v = imuState_k.b_v;
    
    % Translation state
    %IMU中测量的速度通过积分更新位置（位置为global坐标系）
    d = (measurements_k.v - imuState_k.b_v) * measurements_k.dT;
    imuState_prop.p_I_G = C_IG' * d + imuState_k.p_I_G;
    
end