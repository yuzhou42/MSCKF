function [errorVec] = imuError(kState, kMinus1State, imuMeasurement, deltaT)
%函数功能： 计算kState时刻通过IMU预估状态与优化状态量之间的误差
% error_imu = ||IMU_propagation - state||
%
%返回值：
%      errorVec：误差向量[平移误差，旋转误差]
%输入值：
%      kState： 当前帧状态
%      kMinus1State： 上一帧状态
%      imuMeasurement： IMU测量值
%      deltaT：两帧间时间间隔
%IMUERROR Compute the 6x1 error vector associated with interoceptive measurement

%由IMU角速度测量的到的两帧间姿态角微小增量向量 psiVec = w * dt
psiVec = imuMeasurement.omega*deltaT;
%归一化姿态角微小增量向量 psiVec/norm(psiVec)
psiMag = norm(psiVec);
%由IMU速度测量值得到的两帧间位移增量 distance = v * dt
d = imuMeasurement.v*deltaT;

%Compute rotational error (See Lecture8-10)
%由姿态角微小增量向量计算得到旋转矩阵
Phi = cos(psiMag)*eye(3) + (1 - cos(psiMag))*(psiVec/psiMag)*(psiVec/psiMag)' - sin(psiMag)*crossMat(psiVec/psiMag);
%步骤1：得到kState时刻预估值与状态量之间的残差
%R_kState_hat = Phi*kMinus1State.C_vi得到kState时刻的姿态预估值
%R_kState * R_kState_hat'得到kState时刻预估值与状态量之间的角度残差
%例如：R2 = deltaR * R1，则deltaR = R2 * R1'
%                                  |    1      -theta_z     theta_y |
%                                = | theta_z      1        -theta_x |
%                                  |-theta_y    theta_x        1    |
eRotMat = kState.C_vi*(Phi*kMinus1State.C_vi)';
eRot = [eRotMat(2,3); eRotMat(3,1); eRotMat(1,2)];

%Compute translational error
%步骤2：得到kState时刻预估值与状态量之间的位移残差
eTrans = kState.r_vi_i - (kMinus1State.r_vi_i + kMinus1State.C_vi'*d);

errorVec = [eTrans; eRot];
end

