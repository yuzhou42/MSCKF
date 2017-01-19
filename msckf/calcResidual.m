function [r_j] = calcResidual(p_f_G, camStates, measurements)
%函数功能：计算优化后特征3D坐标与相机匹配特征点之间的重投影残差
%
%返回值：
%      r_j：特征点对应的所有的观测和估计值的残差
%输入值：
%      p_f_G：特征点3D坐标
%      camStates：相机状态
%      measurements：特征点相机坐标系下坐标

%CALCRESIDUAL Calculates the residual for a feature position

% measurements is 2 x M_j
% camStates is a cell array of the camState structs for the states
%   included in measurements
    r_j = NaN(2*size(camStates,2), 1);
    for i = 1:size(camStates,2)
        %将global坐标系下3D点坐标转化到camera坐标系下
        C_CG = quatToRotMat(camStates{i}.q_CG);
        p_f_C = C_CG * (p_f_G - camStates{i}.p_C_G);
        %计算该特征点3D坐标与所有相机匹配点之间的重投影残差
        %得到特征点在camera坐标系下的齐次坐标
        zhat_i_j = p_f_C(1:2)/p_f_C(3);
        iStart = 2*(i-1)+1;
        iEnd = 2*i;
        r_j(iStart:iEnd) = measurements(:,i) - zhat_i_j;
    end
        
end