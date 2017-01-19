function msckfState_prop = propagateMsckfStateAndCovar(msckfState, measurements_k, noiseParams)
%函数功能：msckf预测更新，更新状态量，更新协方差矩阵
%返回值：
%      msckfState_prop：更新后的状态量
%输入值：
%      msckfState：更新前的状态量
%      measurements_k：测量值（IMU角速度与IMU速度）
%      noiseParams：噪声

    % Jacobians
    Q_imu = noiseParams.Q_imu;
    %步骤1：误差状态转移矩阵（对状态的求导）
    F = calcF(msckfState.imuState, measurements_k);
    %步骤2：误差状中噪声更新矩阵（对噪声部分的求导）
    G = calcG(msckfState.imuState);

    %Propagate State
    %步骤3：根据IMU测量值更新IMU的状态（四元数 陀螺仪零偏与速度零偏 位置）
    msckfState_prop.imuState = propagateImuState(msckfState.imuState, measurements_k);

    % State Transition Matrix
    %注意！ 状态转移协方差的更新矩阵与时间有关
    Phi = eye(size(F,1)) + F * measurements_k.dT; % Leutenegger 2013
    
    % IMU-IMU Covariance
%     msckfState_prop.imuCovar = msckfState.imuCovar + ...
%                                 ( F * msckfState.imuCovar ...
%                                 + msckfState.imuCovar * F' ...
%                                 + G * Q_imu * G' ) ...
%                                         * measurements_k.dT;

    %步骤4：更新协方差矩阵中IMU-IMU块部分
    %imuCover := P * imuCover * P' + G * Q_imu * G'
    %notation! 噪声协方差的更新矩阵与时间有关
    msckfState_prop.imuCovar = Phi * msckfState.imuCovar * Phi' ...
                                + G * Q_imu * G' * measurements_k.dT; % Leutenegger 2013
    
    % Enforce PSD-ness
    %步骤5：强制协方差变为对称矩阵：主对角线元素取绝对值，非对角线元素对称元素取平均值
    msckfState_prop.imuCovar = enforcePSD(msckfState_prop.imuCovar);
                                    
    % Camera-Camera Covariance
    %步骤6：更新协方差矩阵中Camera-Camera块部分，不变
    msckfState_prop.camCovar = msckfState.camCovar;
    
    % IMU-Camera Covariance
    %步骤7：更新协方差矩阵中IMU-Camera块部分
    %imuCamCovar := P * imuCamCovar
    msckfState_prop.imuCamCovar = Phi * msckfState.imuCamCovar;
    msckfState_prop.camStates = msckfState.camStates;
end