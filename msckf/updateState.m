function msckfState_up = updateState(msckfState, deltaX)
%函数功能：根据误差状态更新MSCKF状态，x_true := x_nominal + detx
%
%返回值：
%      msckfState_up：更新后的MSCKF状态
%输入值：
%      msckfState：未更新前的状态
%      deltaX：滤波后得到的误差状态量

% Updates MSCKF state with deltaX

    % Initialize updated state with current state
    msckfState_up = msckfState;

    % Update IMU State
    % 误差状态变量赋值
    deltatheta_IG = deltaX(1:3);
    deltab_g = deltaX(4:6);
    deltab_v = deltaX(7:9);
    deltap_I_G = deltaX(10:12);
    % 将姿态角误差转换为增量四元数 q = [deltaq; 1]
    deltaq_IG = buildUpdateQuat(deltatheta_IG);
    %步骤1：更新IMU状态 x_true := x_nominal + detx
    msckfState_up.imuState.q_IG = quatLeftComp(deltaq_IG) * msckfState.imuState.q_IG;
    msckfState_up.imuState.b_g = msckfState.imuState.b_g + deltab_g;
    msckfState_up.imuState.b_v = msckfState.imuState.b_v + deltab_v;
    msckfState_up.imuState.p_I_G = msckfState.imuState.p_I_G + deltap_I_G;
    
    % Update camera states
    %步骤2：更新相机状态 x_true := x_nominal + detx
    for i = 1:size(msckfState.camStates, 2)
        qStart = 12 + 6*(i-1) + 1;
        pStart = qStart+3;
        
        deltatheta_CG = deltaX(qStart:qStart+2);
        deltap_C_G = deltaX(pStart:pStart+2);
        
        deltaq_CG = buildUpdateQuat(deltatheta_CG);
        
        msckfState_up.camStates{i}.q_CG = quatLeftComp(deltaq_CG) * msckfState.camStates{i}.q_CG;
        msckfState_up.camStates{i}.p_C_G = msckfState.camStates{i}.p_C_G + deltap_C_G;
    end
    
end