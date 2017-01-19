function  [updatedMsckfState, featCamStates, camStateIndices] = removeTrackedFeature(msckfState, featureId)
%函数功能：从所有相机状态中剔除该特征点（已证明观测到的该特征点足够好），并将涉及到的相机状态添加到状态待优化列表中
%（相机状态只包含位置和四元数，但是同样会记录跟踪到的特征点这个属性）
%返回值：
%      updatedMsckfState：状态待优化列表（msckf状态）
%      featCamStates：待优化的相机状态（该相机观测的特征点超出视野或长期被观测到（超出允许的增广相机状态上限））
%      camStateIndices：待优化的相机索引
%输入值：
%      msckfState：msckf状态
%      featureId：要剔除的特征ID

%REMOVETRACKEDFEATURE Remove tracked feature from camStates and extract all
%camera states that include it

    updatedCamStates = msckfState.camStates;
    featCamStates = {};
    camStateIndices = [];
    %遍历msckf的相机状态
    for c_i = 1:length(updatedCamStates)
        %相机状态中记录了跟踪到的特征点，判断要剔除的特征点是否在其内
        featIdx = find(featureId == updatedCamStates{c_i}.trackedFeatureIds);
        %如果要剔除的特征点是否在当前相机追踪到的特征列表中，则将其剔除（置为空）
        %将该相机状态添加到将优化的状态列表中
        if ~isempty(featIdx)
            updatedCamStates{c_i}.trackedFeatureIds(featIdx) = [];
            camStateIndices(end + 1) = c_i;
            featCamStates{end +1} = updatedCamStates{c_i};
        end
    end
    
    updatedMsckfState = msckfState;
    updatedMsckfState.camStates = updatedCamStates;
end

