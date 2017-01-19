function [msckfState, featureTracks, trackedFeatureIds] = initializeMSCKF(firstImuState, firstMeasurements, camera, state_k, noiseParams)
%函数功能：初始化状态，初始化跟踪到的特征点，第一帧所有的特征点认为都被跟踪上
%
%返回值：
%      msckfState：增广后的的状态，IMU相关状态加上一个相机位姿
%      featureTracks：记录跟踪到的特征点，初始化时，所有的特征点认为都被跟踪到了
%      trackedFeatureIds：记录跟踪到的特征点的ID号
%输入值：
%      firstImuState：IMU初始状态
%      firstMeasurements：初始帧（state_k）的所有测量数据，
%                         [dt,y,omega,v],dt:时间间隔 y:特征点在相机坐标系下的位置 omega是机体角速度测量值 v是线速度
%      camera：记录了相机与IMU之间的变换关系
%      state_k：记录该帧ID号
%      noiseParams：用于初始化协方差矩阵

%INITIALIZEMSCKF Initialize the MSCKF with tracked features and ground
%truth


%Compute the first state
%firstImuState:1、b_g:陀螺仪零偏
%              2、b_v:速度零偏
%msckfState:1、imuState:imu状态（p,q,b_g,b_v）
%           2、imuCovar:imu状态自身协方差 
%           3、camCovar:相机自身协方差
%           4、imuCamCovar:imu与相机之间的协方差
%           5、camStates:相机状态（p,q）
firstImuState.b_g = zeros(3,1);
firstImuState.b_v = zeros(3,1);
msckfState.imuState = firstImuState;
msckfState.imuCovar = noiseParams.initialIMUCovar;
msckfState.camCovar = [];
msckfState.imuCamCovar = [];
msckfState.camStates = {};

%函数功能：通过当前IMU的位姿得到当前相机的位姿，并将相机的位姿增广到状态量中，求取雅克比，并更新协方差矩阵
msckfState = augmentState(msckfState, camera, state_k);

%Compute all of the relevant feature tracks
%记录所有跟踪到的特征点坐标以及特征点ID号，初始化时，所有的特征点以及ID号都压入到featureTracks
featureTracks = {};
%记录所有跟踪到的特征点ID号，初始化时，所有特征点认为都被跟踪到了
trackedFeatureIds = [];

%遍历该帧（state_k）所有的特征点
 for featureId = 1:size(firstMeasurements.y,2)
        %取出特征点坐标
        meas_k = firstMeasurements.y(:, featureId);
        %如果特征点有效
        if ~isnan(meas_k(1,1))
                %Track new feature

                %定义结构体track[featureId,相机坐标下特征点坐标]
                track.featureId = featureId;
                track.observations = meas_k;
                %定义结构体featureTracks[track,featureId]
                featureTracks{end+1} = track;
                trackedFeatureIds(end+1) = featureId;
                %Add observation to current camera
                msckfState.camStates{end}.trackedFeatureIds(end+1) = featureId;
        end
 end
 
end

