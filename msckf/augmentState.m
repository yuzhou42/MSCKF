function msckfState_aug = augmentState(msckfState, camera, state_k)
%函数功能：msckf状态量中增广相机的状态，增广雅克比，增广协方差矩阵，并更新协方差矩阵
%
%返回值：
%      msckfState_aug：增广后的状态量
%输入值：
%      msckfState：增广前的状态量
%      camera：包含了相机与IMU之间的位姿关系
%      state_k：该帧ID号

% Augments the MSCKF state with a new camera pose    

    %将imu状态中的四元数转化为旋转矩阵，globle到imu
    C_IG = quatToRotMat(msckfState.imuState.q_IG);
    
    % Compute camera pose from current IMU pose
    %步骤1：由IMU以及IMU与camera的固连关系得到相机的位置和姿态
    %得到global到相机的变换四元数
    q_CG = quatLeftComp(camera.q_CI) * msckfState.imuState.q_IG;
    %得到相机在世界坐标系下的位置
    p_C_G = msckfState.imuState.p_I_G + C_IG' * camera.p_C_I;

    % Build MSCKF covariance matrix
    %步骤2：构造增广前的协方差矩阵：
    % |imu协方差      imu与相机协方差|
    % |相机与imu协方差     相机协方差|
    P = [msckfState.imuCovar, msckfState.imuCamCovar;
        msckfState.imuCamCovar', msckfState.camCovar];
    
    % Camera state Jacobian
    %步骤3：增广状态以后，需要得到增广状态（相机位置、相机四元数）对msckf状态（增广前以后的状态）的雅克比
    %camera中包含了camera与IMU之间的变换关系
    %msckfState.imuState为状态中的IMU相关部分
    %msckfState.camStates为状态中的相机部分
    J = calcJ(camera, msckfState.imuState, msckfState.camStates);
    
    %现有状态中和camera有关的状态量个数
    N = size(msckfState.camStates,2);
    
    %步骤4：构造增广后的协方差矩阵：
    %步骤4.1：增广后相机状态后所有状态的雅克比矩阵
    tempMat = [eye(12+6*N); J];
    
    % Augment the MSCKF covariance matrix
    %步骤4.2：更新增广后相机状态后所有状态的协方差矩阵
    P_aug = tempMat * P * tempMat';
    
    % Break everything into appropriate structs
    %得到增广后的状态量向量
    %[之前的状态，camState{N+1}]
    %camState[P_C_G,q_CG,state_k,trackedFeatureIds]
    msckfState_aug = msckfState;
    msckfState_aug.camStates{N+1}.p_C_G = p_C_G;
    msckfState_aug.camStates{N+1}.q_CG = q_CG;
    msckfState_aug.camStates{N+1}.state_k = state_k;
    msckfState_aug.camStates{N+1}.trackedFeatureIds = [];
    msckfState_aug.imuCovar = P_aug(1:12,1:12);
    msckfState_aug.camCovar = P_aug(13:end,13:end);
    msckfState_aug.imuCamCovar = P_aug(1:12, 13:end);
end