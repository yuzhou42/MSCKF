function imuStates_up = updateStateHistory(imuStates, msckfState, camera, state_k)
%函数功能：从MSCKF状态中更新IMU的历史状态，用相机的状态更新对应时刻imu的位姿状态
%
%返回值：
%      imuStates_up：将IMU的状态用MSCKF的状态替代
%输入值：
%      imuStates：IMU状态
%      msckfState：当前帧（state_k）MSCKF状态
%      camera：记录了相机与IMU之间的变换关系
%      state_k：记录该帧ID号

% updateStateHistory -- updates IMU state history from current msckfState
%
% INPUTS:   imuStates -- all the IMU states
%           msckfState -- the current msckfState (at timestep state_k)
%           camera -- camera parameters (needed for IMU-to-camera transform)
%           state_k -- the current timestep
%
% OUTPUTS: imuStates_up -- all the IMU states, replaced
%                           by the msckfState versions where available
%

    % Platitude of the day: Everything that hasn't changed must stay the same
    %将msckf中imu的历史状态给imuStates_up
    imuStates_up = imuStates;

    % Update the current IMU state
    %用msckf中最新的（state_k）imu的状态添加到imuStates_up
    imuStates_up{state_k}.q_IG = msckfState.imuState.q_IG;
    imuStates_up{state_k}.p_I_G = msckfState.imuState.p_I_G;
    imuStates_up{state_k}.b_g = msckfState.imuState.b_g;
    imuStates_up{state_k}.b_v = msckfState.imuState.b_v;
    imuStates_up{state_k}.covar = msckfState.imuCovar;
    
    % Update IMU states corresponding to active camera poses
    
    % Camera to IMU transformation
    %得到相机到IMU的平移和四元数
    C_CI = quatToRotMat(camera.q_CI);
    q_IC = rotMatToQuat(C_CI');
    p_I_C = - C_CI' * camera.p_C_I;
    
    %遍历所有msckf中所有相机的状态
    for camIdx = 1:size(msckfState.camStates, 2)
        %得到global到imu的四元数
        q_IG = quatLeftComp(q_IC) * msckfState.camStates{camIdx}.q_CG;
        C_IG = quatToRotMat(q_IG);
        p_C_G = msckfState.camStates{camIdx}.p_C_G;
        %得到global到imu的位置，即imu在global坐标系下的坐标
        p_I_G = p_C_G + C_IG'*C_CI'*p_I_C;
        
        cam_state_k = msckfState.camStates{camIdx}.state_k;
        
        %用相机的状态更新对应时刻imu的位姿状态
        imuStates_up{cam_state_k}.q_IG = q_IG;
        imuStates_up{cam_state_k}.p_I_G = p_I_G;
    end
end