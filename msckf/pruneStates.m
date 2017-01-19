function [ prunedMsckfState, deletedCamStates ] = pruneStates( msckfState )
%函数功能：分别得到MSCKF中需要被保留和删除的状态与协方差
%
%返回值：
%      prunedMsckfState：MSCKF中需要被保留的状态和协方差
%      deletedCamStates：MSCKF中需要被删除的状态和协方差
%输入值：
%      msckfState：msckf未删除前状态

%PRUNESTATES Prunes any states that have no tracked features and updates
%covariances
    
    prunedMsckfState.imuState = msckfState.imuState;
    prunedMsckfState.imuCovar = msckfState.imuCovar;
    
    %Find all camStates with no tracked landmarks    
    %找到跟踪特征数为空的相机状态，添加到删除列表deleteIdx中
    deleteIdx = [];
    for c_i = 1:length(msckfState.camStates)
        if isempty(msckfState.camStates{c_i}.trackedFeatureIds)
            deleteIdx(end+1) = c_i;
        end
    end
    
    %Prune the damn states!
    
    %步骤1：删除msckf中需要删除的状态
    %取出msckf中要删除的相机状态
    deletedCamStates = msckfState.camStates(deleteIdx);
    %从msckf中删除需要删除的相机状态
    prunedMsckfState.camStates = removeCells(msckfState.camStates, deleteIdx);
    
    %协方差矩阵中相机协方差的行数
    statesIdx = 1:size(msckfState.camCovar,1);
    %keepCovarMask用于标记要保留的协方差行
    keepCovarMask = true(1, numel(statesIdx));
    for dIdx = deleteIdx
        %协方差中要删除的置为false
        keepCovarMask(6*dIdx - 5:6*dIdx) = false(6,1);
    end
    
    %给协方差相应位置标明是否保留状态，保留为true，删除为false
    keepCovarIdx = statesIdx(keepCovarMask);
    deleteCovarIdx = statesIdx(~keepCovarMask);

    %步骤2：删除msckf中需要删除的协方差
    %得到需要被保留的协方差，IMU-IMU协方差全部保留
    %得到camera-camera协方差矩阵块中需要被保留的部分
    prunedMsckfState.camCovar = msckfState.camCovar(keepCovarIdx, keepCovarIdx);
    %Keep rows, prune columns of upper right covariance matrix
    %得到imu-camera协防差矩阵块中需要被保留的部分
    prunedMsckfState.imuCamCovar = msckfState.imuCamCovar(:, keepCovarIdx);
    %得到camera-camera协方差矩阵块中要删除的部分
    deletedCamCovar = msckfState.camCovar(deleteCovarIdx, deleteCovarIdx);
    %得到要删除的camera-camera协方差对角线的元素均方根deletedCamSigma
    deletedCamSigma = sqrt(diag(deletedCamCovar));
    
    % Grab the variances of the deleted states for plotting
    %得到要删除的相机状态对应的协方差矩阵对角线元素的均方根（画图用）
    for c_i = 1:size(deletedCamStates, 2)
        deletedCamStates{c_i}.sigma = deletedCamSigma(6*c_i - 5 : 6*c_i);
    end
end

