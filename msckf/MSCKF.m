%% ============================Notation============================ %%
% X_sub_super
% q_ToFrom
% p_ofWhat_expressedInWhatFrame


%% =============================Setup============================== %%
clear;
close all;
clc;
addpath('utils');

tic
dataDir = '../datasets';

% fileName = 'dataset3';
% fileName = 'dataset3_fresh_10noisy';
% fileName = 'dataset3_fresh_10lessnoisy';
% fileName = 'dataset3_fresh_20lessnoisy';
% fileName = 'dataset3_fresh_40lessnoisy';
% fileName = 'dataset3_fresh_60lessnoisy';
% fileName = 'dataset3_fresh_80lessnoisy';
% fileName = 'dataset3_fresh_100lessnoisy';
% fileName = 'dataset3_fresh_500lessnoisy';
% fileName = '2011_09_26_drive_0035_sync_KLT';
% fileName = '2011_09_26_drive_0005_sync_KLT';
% fileName = '2011_09_30_drive_0020_sync_KLT';
% fileName = '2011_09_26_drive_0027_sync_KLT';
% fileName = '2011_09_30_drive_0020_sync_KLT'; kStart = 2; kEnd = 900;

% Good KITTI runs
 fileName = '2011_09_26_drive_0001_sync_KLT'; kStart = 2; kEnd = 98;
%fileName = '2011_09_26_drive_0036_sync_KLT'; kStart = 2; kEnd = 239;
% fileName = '2011_09_26_drive_0051_sync_KLT'; kStart = 2; kEnd = 114;
% fileName = '2011_09_26_drive_0095_sync_KLT'; kStart = 2; kEnd = 139;

%步骤1：加载数据
load(sprintf('%s/%s.mat',dataDir,fileName));

% r_i_vk_i = p_vi_i;

%Dataset window bounds
% kStart = 2; kEnd = 177;
% kStart = 1215; kEnd = 1715;

%Set constant
%总共为4*frames*features，得到特征点的个数
numLandmarks = size(y_k_j,3);

%Set up the camera parameters
%相机内参，IMU到相机的变换 Camera = q_CI * IMU + p_C_I
camera.c_u      = cu;                   % Principal point [u pixels]
camera.c_v      = cv;                   % Principal point [v pixels]
camera.f_u      = fu;                   % Focal length [u pixels]
camera.f_v      = fv;                   % Focal length [v pixels]
camera.b        = b;                    % Stereo baseline [m]
camera.q_CI     = rotMatToQuat(C_c_v);  % 4x1 IMU-to-Camera rotation quaternion
camera.p_C_I    = rho_v_c_v;            % 3x1 Camera position in IMU frame

%Set up the noise parameters
%初始化相机测量噪声11个像素点，[Pleft_u, Pleft_v, Pright_u, Pright_v]
y_var = 11^2 * ones(1,4);               % pixel coord var 121
noiseParams.u_var_prime = y_var(1)/camera.f_u^2;
noiseParams.v_var_prime = y_var(2)/camera.f_v^2;

%步骤2：初始化imu预测协方差和测量协方差矩阵
%步骤2.1：初始化测量协方差矩阵
%角速度测量协方差
w_var = 4e-2 * ones(1,3);              % rot vel var0.04
%速度协方差
v_var = 4e-2 * ones(1,3);              % lin vel var
%陀螺仪零偏协方差
dbg_var = 1e-6 * ones(1,3);            % gyro bias change var
%速度零偏协方差
dbv_var = 1e-6 * ones(1,3);            % vel bias change var
%构造所有状态量的初始协方差矩阵
noiseParams.Q_imu = diag([w_var, dbg_var, v_var, dbv_var]);

%步骤2.2：初始化状态量预测协方差矩阵
q_var_init = 1e-6 * ones(1,3);         % init rot var
p_var_init = 1e-6 * ones(1,3);         % init pos var
bg_var_init = 1e-6 * ones(1,3);        % init gyro bias var
bv_var_init = 1e-6 * ones(1,3);        % init vel bias var
noiseParams.initialIMUCovar = diag([q_var_init, bg_var_init, bv_var_init, p_var_init]);
   
% MSCKF parameters
% 设置MSCKF参数
% 特征点至少被跟踪10帧以上
msckfParams.minTrackLength = 10;        % Set to inf to dead-reckon only
msckfParams.maxTrackLength = Inf;      % Set to inf to wait for features to go out of view
msckfParams.maxGNCostNorm  = 1e-2;     % Set to inf to allow any triangulation, no matter how bad
msckfParams.minRCOND       = 1e-12;
msckfParams.doNullSpaceTrick = true;
msckfParams.doQRdecomp = true;


% IMU state for plotting etc. Structures indexed in a cell array
imuStates = cell(1,numel(t));
prunedStates = {};

% imuStates{k}.q_IG         4x1 Global to IMU rotation quaternion
% imuStates{k}.p_I_G        3x1 IMU Position in the Global frame
% imuStates{k}.b_g          3x1 Gyro bias
% imuStates{k}.b_v          3x1 Velocity bias
% imuStates{k}.covar        12x12 IMU state covariance

% We don't really need these outside of msckfState, do we?
% camState = cell(1,numel(t));
% camStates{k}.q_CG        4x1 Global to camera rotation quaternion
% camStates{k}.p_C_G       3x1 Camera Position in the Global frame
% camStates{k}.trackedFeatureIds  1xM List of feature ids that are currently being tracked from that camera state
% camStates{k}.state_k

%msckfState.imuState
%msckfState.imuCovar
%msckfState.camCovar
%msckfState.imuCamCovar
%msckfState.camStates


% Measurements as structures all indexed in a cell array
%diff函数通过微分t向量得到{0，dT1,dT2,dT3...}
dT = [0, diff(t)];
%得到向量t的元素个数
measurements = cell(1,numel(t));
% groundTruthStates = cell(1,numel(t));
% groundTruthMap = rho_i_pj_i;

% Important: Because we're idealizing our pixel measurements and the
% idealized measurements could legitimately be -1, replace our invalid
% measurement flag with NaN
%遍历y_k_j中所有元素，如果数据中有-1，则改为无穷大
y_k_j(y_k_j == -1) = NaN;

%步骤3：导入测量数据和参考值：measurements与groundTruthStates
%measurements：[dt,y,omega,v],dt:时间间隔 y:特征点在相机坐标系下的位置 omega是机体角速度测量值 v是线速度
%groundTruthStates:[imuState:q_IG,p_I_G
%                   camState:q_IG,p_I_G]
for state_k = kStart:kEnd 
    measurements{state_k}.dT    = dT(state_k);                      % sampling times
    %y为左相机state_k帧所有的特征点，y为2*features维的矩阵
    measurements{state_k}.y     = squeeze(y_k_j(1:2,state_k,:));    % left camera only 三维压缩成二维得到一帧的特征点
    %Omega为state_k帧的所有角速度，omega为3*1维的列向量
    measurements{state_k}.omega = w_vk_vk_i(:,state_k);             % ang vel
    %v为state_k帧的所有线速度，v为3*1维的列向量
    measurements{state_k}.v     = v_vk_vk_i(:,state_k);             % lin vel
    
    %Idealize measurements
    %判断state_k帧的所有特征点是否有效，有效为1，无效为0
    validMeas = ~isnan(measurements{state_k}.y(1,:));
    %将该帧所有有效的特征点，由图像坐标系反投影到相机坐标系
    measurements{state_k}.y(1,validMeas) = (measurements{state_k}.y(1,validMeas) - camera.c_u)/camera.f_u;
    measurements{state_k}.y(2,validMeas) = (measurements{state_k}.y(2,validMeas) - camera.c_v)/camera.f_v;
    
    %Ground Truth
    %theta_vk_i是姿态角参考值，globle到imu，将姿态角转化为旋转矩阵，再转为四元数
    q_IG = rotMatToQuat(axisAngleToRotMat(theta_vk_i(:,state_k)));
    %r_i_vk_i是位置参考值,globle到imu
    p_I_G = r_i_vk_i(:,state_k);
    
    %定义参考值结构体：[imuState:q_IG,p_I_G
    %                  camState:q_IG,p_I_G]
    groundTruthStates{state_k}.imuState.q_IG = q_IG;
    groundTruthStates{state_k}.imuState.p_I_G = p_I_G;
    
    % Compute camera pose from current IMU pose
    %得到globle到imu的旋转矩阵，旋转矩阵用C表示
    C_IG = quatToRotMat(q_IG);
    %得到globle到camera的四元数变换
    q_CG = quatLeftComp(camera.q_CI) * q_IG;
    %得到globle到camera的位置变换，即相机在世界坐标系下的位置
    p_C_G = p_I_G + C_IG' * camera.p_C_I;
    
    %定义参考值结构体：[imuState:q_IG,p_I_G
    %                  camState:q_IG,p_I_G]
    groundTruthStates{state_k}.camState.q_CG = q_CG;
    groundTruthStates{state_k}.camState.p_C_G = p_C_G;
    
end


%Struct used to keep track of features
%记录追踪到的所有特征点在不同相机中的观测以及特征ID号 
featureTracks = {};
trackedFeatureIds = [];

% featureTracks = {track1, track2, ...}
% track.featureId 
% track.observations



%% ==========================Initial State======================== %%
%Use ground truth for first state and initialize feature tracks with
%feature observations
%Use ground truth for the first state

%步骤4：初始化MSCKF
%步骤4.1：获取第一个四元数参考值作为IMU状态的初始值
firstImuState.q_IG = rotMatToQuat(axisAngleToRotMat(theta_vk_i(:,kStart)));
%步骤4.2：获取第一个位置参考值作为位置初始值
firstImuState.p_I_G = r_i_vk_i(:,kStart);
% firstImuState.q_IG = [0;0;0;1];
% firstImuState.q_IG = rotMatToQuat(rotx(90));
% firstImuState.p_I_G = [0;0;0];

%步骤4.3：初始化msckf的状态，目前只有IMU的状态，初始化跟踪到的特征点，第一帧所有的特征点认为都被跟踪上
[msckfState, featureTracks, trackedFeatureIds] = initializeMSCKF(firstImuState, measurements{kStart}, camera, kStart, noiseParams);
%从MSCKF状态中更新IMU的历史状态，用相机的状态更新对应时刻imu的位姿状态
imuStates = updateStateHistory(imuStates, msckfState, camera, kStart);
%画图调试使用
msckfState_imuOnly{kStart} = msckfState;

%% ============================MAIN LOOP========================== %%

numFeatureTracksResidualized = 0;
map = [];

%遍历所有帧（每一帧图像对应一个imu数据）
for state_k = kStart:(kEnd-1)
    fprintf('state_k = %4d\n', state_k);
    
    %% ==========================STATE PROPAGATION======================== %%
    %貌似默认IMU和Camera同步更新
    %Propagate state and covariance
    %步骤5：msckf预测更新，更新状态量，更新协方差矩阵
    msckfState = propagateMsckfStateAndCovar(msckfState, measurements{state_k}, noiseParams);
    %msckf预测更新，更新状态量，更新协方差矩阵，但是msckfState_imuOnly状态量中只有IMU的状态（用于对比测试）
    msckfState_imuOnly{state_k+1} = propagateMsckfStateAndCovar(msckfState_imuOnly{state_k}, measurements{state_k}, noiseParams);
    %Add camera pose to msckfState
    %步骤6：msckf状态量中增广相机的状态，增广雅克比，增广协方差矩阵，并更新协方差矩阵
    msckfState = augmentState(msckfState, camera, state_k+1);
    %% ==========================FEATURE TRACKING======================== %%
    % Add observations to the feature tracks, or initialize a new one
    % If an observation is -1, add the track to featureTracksToResidualize
    featureTracksToResidualize = {};
    
    %步骤7：遍历当前帧所有特征点，更新featureTracks
    %说明：msckf中用featureTracks记录了目前被跟踪到的特征点。
    %       featureTracks中包含每个特征点ID和观测值（即在所有能观测到该特征点的相机坐标系下的齐次坐标）
           
    %遍历该帧所有的特征点
    for featureId = 1:numLandmarks
        %IMPORTANT: state_k + 1 not state_k
        %取一个特征点坐标（特征点为相机坐标系，即已经过反投影坐标）
        meas_k = measurements{state_k+1}.y(:, featureId);
        
        %判断该特征点是否有效
        outOfView = isnan(meas_k(1,1));
        
        %步骤7.1：遍历当前帧所有特征点，判断是否属于featureTracks
        if ismember(featureId, trackedFeatureIds)

            %步骤7.2：如果该特征点在视野范围内：将特征点在相机坐标系下的齐次坐标添加到featureTracks中对应特征的观测中。
            if ~outOfView
                %Append observation and append id to cam states
                %将该特征点以及ID号添加到featureTracks
                %trackedFeatureIds记录了跟踪到的特征点ID，featureTracks记录了跟踪到的特征点坐标
                featureTracks{trackedFeatureIds == featureId}.observations(:, end+1) = meas_k;
                
                %Add observation to current camera
                %将跟踪到的特征点ID号添加到相机状态的属性trackedFeatureIds中
                %（相机状态只包含位置和四元数，但是同样会记录跟踪到的特征点这个属性）
                msckfState.camStates{end}.trackedFeatureIds(end+1) = featureId;
            end
            
            %步骤7.3：如果该特征点超出视野范围或者能够观测到该特征的相机数目大于上限值
            %取出该特征点元（特征点+特征点ID+跟踪到该特征点的相机状态）
            track = featureTracks{trackedFeatureIds == featureId};
            
            %如果特征不在视野范围内或者特征点的观测数大于设定的上限值，则判断该特征点是否足够好
            %则将观测到该特征点的相机取出进行优化
            if outOfView ...
                    || size(track.observations, 2) >= msckfParams.maxTrackLength ...
                    || state_k+1 == kEnd
                                
                %Feature is not in view, remove from the tracked features
                %从所有相机状态中剔除该特征点，并将涉及到的相机状态添加到状态待优化列表中
                %updatedMsckfState：状态待优化列表（msckf状态）
                %featCamStates：待优化的相机状态（该相机观测的特征点超出视野或长期被观测到）
                %camStateIndices：待优化的相机索引
                [msckfState, camStates, camStateIndices] = removeTrackedFeature(msckfState, featureId);
                
                %Add the track, with all of its camStates, to the
                %residualized list
                %待优化的相机状态超过最小跟踪长度（10），则将其添加到列表中用于优化（featureTracksToResidualize）
                if length(camStates) >= msckfParams.minTrackLength
                    track.camStates = camStates;
                    track.camStateIndices = camStateIndices;
                    featureTracksToResidualize{end+1} = track;
                end
               
                %Remove the track
                %若已使用完给特征，则从featureTracks中剔除该特征点
                featureTracks = featureTracks(trackedFeatureIds ~= featureId);
                %若已使用完给特征，则从trackedFeatureIds中剔除该特征点ID
                trackedFeatureIds(trackedFeatureIds == featureId) = []; 
            end
        
        %步骤7.4：跟踪到新的特征点（该特征点之前没有被跟踪到，并且在视野范围内），将该特征点添加到跟踪特征点列表
        %同时，观测到该特征点的相机记录该特征点已被跟踪到（添加到相机状态中）
        elseif ~outOfView && state_k+1 < kEnd % && ~ismember(featureId, trackedFeatureIds)
            %Track new feature
            track.featureId = featureId;
            track.observations = meas_k;
            featureTracks{end+1} = track;
            trackedFeatureIds(end+1) = featureId;

            %Add observation to current camera
            msckfState.camStates{end}.trackedFeatureIds(end+1) = featureId;
        end
    end
     
    %步骤8：MSCKF测量更新。遍历所有用于优化的特征点,构造观测模型（特征点重投影误差），更新MSCKF状态
    %% ==========================FEATURE RESIDUAL CORRECTIONS======================== %%
    %如果用于优化的特征点（记录了能观测到这些特征点的相机）不为空
    if ~isempty(featureTracksToResidualize)
        H_o = [];
        r_o = [];
        R_o = [];
        %步骤8.1：通过特征点所有观测估计出该特征点的3D空间坐标位置
        %遍历所有用于优化的特征点
        for f_i = 1:length(featureTracksToResidualize)
            %取出其中一个特征点元(该特征对应的所有观测)
            track = featureTracksToResidualize{f_i};     
            %Estimate feature 3D location through Gauss Newton inverse depth
            %optimization
            %使用逆深度参数构造重投影误差函数，用高斯牛顿优化的方法估计特征点3D坐标p_f_G
            [p_f_G, Jcost, RCOND] = calcGNPosEst(track.camStates, track.observations, noiseParams);
            % Uncomment to use ground truth map instead
%              p_f_G = groundTruthMap(:, track.featureId); Jcost = 0; RCOND = 1;
%              p_f_C = triangulate(squeeze(y_k_j(:, track.camStates{1}.state_k, track.featureId)), camera); Jcost = 0; RCOND = 1;
            %获取跟踪到该特征点的相机个数
            nObs = size(track.observations,2);
            %计算所有重投影误差平均到每个相机的投影误差
            JcostNorm = Jcost / nObs^2;
            fprintf('Jcost = %f | JcostNorm = %f | RCOND = %f\n',...
                Jcost, JcostNorm,RCOND);
            
            %投影误差太大，或者迭代时Hessian矩阵求逆不可靠
            if JcostNorm > msckfParams.maxGNCostNorm ...
                    || RCOND < msckfParams.minRCOND
%                     || norm(p_f_G) > 50
                
                break;
            else
                %将3D点加入到地图
                map(:,end+1) = p_f_G;
                numFeatureTracksResidualized = numFeatureTracksResidualized + 1;
                fprintf('Using new feature track with %d observations. Total track count = %d.\n',...
                    nObs, numFeatureTracksResidualized);
            end
            %步骤8.2：通过特征3D坐标与相机匹配特征点之间的重投影残差构造观测模型，包括重投影误差对MSCKF状态量的雅克比矩阵的求解 
            %Calculate residual and Hoj
            %计算优化后特征3D坐标与相机匹配特征点之间的重投影残差
            [r_j] = calcResidual(p_f_G, track.camStates, track.observations);
            %构造重投影残差的噪声向量（2*camState,2*camState）大小矩阵
            R_j = diag(repmat([noiseParams.u_var_prime, noiseParams.v_var_prime], [1, numel(r_j)/2]));
            %计算重投影误差对MSCKF状态量的雅克比矩阵（MSCKF观测模型）
            [H_o_j, A_j, H_x_j] = calcHoj(p_f_G, msckfState, track.camStateIndices);

            % Stacked residuals and friends
            %判断是否要对重投影误差和噪声左乘H_f_j的零空间变换矩阵，详见calcHoj函数最后部分
            if msckfParams.doNullSpaceTrick
                %增广H矩阵
                H_o = [H_o; H_o_j];

                if ~isempty(A_j)
                    r_o_j = A_j' * r_j;
                    r_o = [r_o ; r_o_j];

                    R_o_j = A_j' * R_j * A_j;
                    R_o(end+1 : end+size(R_o_j,1), end+1 : end+size(R_o_j,2)) = R_o_j;
                end
                
            else
                H_o = [H_o; H_x_j];
                r_o = [r_o; r_j];
                R_o(end+1 : end+size(R_j,1), end+1 : end+size(R_j,2)) = R_j;
            end
        end
        
        if ~isempty(r_o)
            % Put residuals into their final update-worthy form
            if msckfParams.doQRdecomp
                %QR分解H_o矩阵，并且剔除Q中的零行，以及R中对应的列
                [T_H, Q_1] = calcTH(H_o);
                %error = H * x + n  (观测模型，n为噪声协方差)
                %      = Q_1 * T_H * x + n
                %==> Q_1' * error = Q_1' * Q_1 * T_H * x + Q_1' * n * Q_1
                %                 = T_H * x + Q_1' * n * Q_1
                r_n = Q_1' * r_o;
                R_n = Q_1' * R_o * Q_1;
            else
                T_H = H_o;
                r_n = r_o;
                R_n = R_o;
            end           
            %步骤8.3：计算卡尔曼增益，更新误差状态
            % Build MSCKF covariance matrix
            P = [msckfState.imuCovar, msckfState.imuCamCovar;
                   msckfState.imuCamCovar', msckfState.camCovar];

            % Calculate Kalman gain
            K = (P*T_H') / ( T_H*P*T_H' + R_n );

            % State correction
            deltaX = K * r_n;
            %步骤8.4：根据误差状态更新MSCKF状态，x_true := x_nominal + detx
            msckfState = updateState(msckfState, deltaX);

            % Covariance correction
            %步骤8.5：协方差测量更新
            tempMat = (eye(12 + 6*size(msckfState.camStates,2)) - K*T_H);
%             tempMat = (eye(12 + 6*size(msckfState.camStates,2)) - K*H_o);

            P_corrected = tempMat * P * tempMat' + K * R_n * K';

            msckfState.imuCovar = P_corrected(1:12,1:12);
            msckfState.camCovar = P_corrected(13:end,13:end);
            msckfState.imuCamCovar = P_corrected(1:12, 13:end);
           
%             figure(1); clf; imagesc(deltaX); axis equal; axis ij; colorbar;
%             drawnow;
            
        end
        
    end
    
        %% ==========================STATE HISTORY======================== %%
        %步骤9：历史状态更新。从MSCKF状态中更新IMU的历史状态，通过相机的状态更新对应时刻imu的位姿状态
        imuStates = updateStateHistory(imuStates, msckfState, camera, state_k+1);
        
        
        %% ==========================STATE PRUNING======================== %%
        %步骤10：状态剔除。剔除MSCKF中需要被删除的状态和对应的协方差矩阵块
        %Remove any camera states with no tracked features
        %分别得到MSCKF中需要被保留和删除的状态与协方差
        [msckfState, deletedCamStates] = pruneStates(msckfState);

        %绘制删掉的相机位置
        if ~isempty(deletedCamStates)
            prunedStates(end+1:end+length(deletedCamStates)) = deletedCamStates;
        end    
        
%         if max(max(msckfState.imuCovar(1:12,1:12))) > 1
%             disp('omgbroken');
%         end
        
        plot_traj;
%     figure(1); imagesc(msckfState.imuCovar(1:12,1:12)); axis equal; axis ij; colorbar;
%     drawnow;
end %for state_K = ...

toc


%% ==========================PLOT ERRORS======================== %%
kNum = length(prunedStates);
p_C_G_est = NaN(3, kNum);
p_I_G_imu = NaN(3, kNum);
p_C_G_imu = NaN(3, kNum);
p_C_G_GT = NaN(3, kNum);
theta_CG_err = NaN(3,kNum);
theta_CG_err_imu = NaN(3,kNum);
err_sigma = NaN(6,kNum); % cam state is ordered as [rot, trans]
err_sigma_imu = NaN(6,kNum);
% 
tPlot = NaN(1, kNum);
% 
for k = 1:kNum
    state_k = prunedStates{k}.state_k;
    
    p_C_G_GT(:,k) = groundTruthStates{state_k}.camState.p_C_G;
    p_C_G_est(:,k) = prunedStates{k}.p_C_G;
    q_CG_est  = prunedStates{k}.q_CG;    
    
    theta_CG_err(:,k) = crossMatToVec( eye(3) ...
                    - quatToRotMat(q_CG_est) ...
                        * ( C_c_v * axisAngleToRotMat(theta_vk_i(:,kStart+k-1)) )' );
      
    err_sigma(:,k) = prunedStates{k}.sigma;
    imusig = sqrt(diag(msckfState_imuOnly{state_k}.imuCovar));
    err_sigma_imu(:,k) = imusig([1:3,10:12]);
    
    p_I_G_imu(:,k) = msckfState_imuOnly{state_k}.imuState.p_I_G;
    C_CG_est_imu = C_CI * quatToRotMat(msckfState_imuOnly{state_k}.imuState.q_IG);
    theta_CG_err_imu(:,k) = crossMatToVec( eye(3) ...
                    - C_CG_est_imu ...
                        * ( C_CI * axisAngleToRotMat(theta_vk_i(:,kStart+k-1)) )' );
                    
    tPlot(k) = t(state_k);
end

% p_I_G_GT = p_vi_i(:,kStart:kEnd);
p_I_G_GT = r_i_vk_i(:,kStart:kEnd);
p_C_G_GT = p_I_G_GT + repmat(rho_v_c_v,[1,size(p_I_G_GT,2)]);
p_C_G_imu = p_I_G_imu + repmat(rho_v_c_v,[1,size(p_I_G_imu,2)]);

rotLim = [-0.5 0.5];
transLim = [-0.5 0.5];

% Save estimates
msckf_trans_err = p_C_G_est - p_C_G_GT;
msckf_rot_err = theta_CG_err;
imu_trans_err = p_C_G_imu - p_C_G_GT;
imu_rot_err = theta_CG_err_imu;
save(sprintf('../KITTI Trials/msckf_%s', fileName));

armse_trans_msckf = mean(sqrt(sum(msckf_trans_err.^2, 1)/3));
armse_rot_msckf = mean(sqrt(sum(msckf_rot_err.^2, 1)/3));
final_trans_err_msckf = norm(msckf_trans_err(:,end));

armse_trans_imu = mean(sqrt(sum(imu_trans_err.^2, 1)/3));
armse_rot_imu = mean(sqrt(sum(imu_rot_err.^2, 1)/3));
final_trans_err_imu = norm(imu_trans_err(:,end));

fprintf('Trans ARMSE: IMU %f, MSCKF %f\n',armse_trans_imu, armse_trans_msckf);
fprintf('Rot ARMSE: IMU %f, MSCKF %f\n',armse_rot_imu, armse_rot_msckf);
fprintf('Final Trans Err: IMU %f, MSCKF %f\n',final_trans_err_imu, final_trans_err_msckf);

% Translation Errors
figure
subplot(3,1,1)
plot(tPlot, p_C_G_est(1,:) - p_C_G_GT(1,:), 'LineWidth', 2)
hold on
plot(tPlot, 3*err_sigma(4,:), '--r')
plot(tPlot, -3*err_sigma(4,:), '--r')
% ylim(transLim)
xlim([tPlot(1) tPlot(end)])
title('Translational Error')
ylabel('\delta r_x')


subplot(3,1,2)
plot(tPlot, p_C_G_est(2,:) - p_C_G_GT(2,:), 'LineWidth', 2)
hold on
plot(tPlot, 3*err_sigma(5,:), '--r')
plot(tPlot, -3*err_sigma(5,:), '--r')
% ylim(transLim)
xlim([tPlot(1) tPlot(end)])
ylabel('\delta r_y')

subplot(3,1,3)
plot(tPlot, p_C_G_est(3,:) - p_C_G_GT(3,:), 'LineWidth', 2)
hold on
plot(tPlot, 3*err_sigma(6,:), '--r')
plot(tPlot, -3*err_sigma(6,:), '--r')
% ylim(transLim)
xlim([tPlot(1) tPlot(end)])
ylabel('\delta r_z')
xlabel('t_k')

% Rotation Errors
figure
subplot(3,1,1)
plot(tPlot, theta_CG_err(1,:), 'LineWidth', 2)
hold on
plot(tPlot, 3*err_sigma(1,:), '--r')
plot(tPlot, -3*err_sigma(1,:), '--r')
ylim(rotLim)
xlim([tPlot(1) tPlot(end)])
title('Rotational Error')
ylabel('\delta \theta_x')


subplot(3,1,2)
plot(tPlot, theta_CG_err(2,:), 'LineWidth', 2)
hold on
plot(tPlot, 3*err_sigma(2,:), '--r')
plot(tPlot, -3*err_sigma(2,:), '--r')
ylim(rotLim)
xlim([tPlot(1) tPlot(end)])
ylabel('\delta \theta_y')

subplot(3,1,3)
plot(tPlot, theta_CG_err(3,:), 'LineWidth', 2)
hold on
plot(tPlot, 3*err_sigma(3,:), '--r')
plot(tPlot, -3*err_sigma(3,:), '--r')
ylim(rotLim)
xlim([tPlot(1) tPlot(end)])
ylabel('\delta \theta_z')
xlabel('t_k')