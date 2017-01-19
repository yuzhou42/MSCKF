% Sliding Window Gauss Newton Optimization
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clc
clear
close all
addpath('utils')
%fileName = '100noisy';
%load(['../datasets/dataset3_fresh_' fileName '.mat'])
fileName = '2011_09_26_drive_0001_sync_KLT.mat';
load(['../datasets/' fileName]);

tic
%Set number of landmarks
%获取数据集特征点个数
numLandmarks = size(y_k_j,3);

%Set up appropriate structs
%设置相机内参结构体
calibParams.c_u = cu;
calibParams.c_v = cv;
calibParams.f_u = fu;
calibParams.f_v = fv;
calibParams.b = b;

%设置机体到相机的旋转平移变换结构体以及对应的SE(3)矩阵
vehicleCamTransform.C_cv = C_c_v;
vehicleCamTransform.rho_cv_v = rho_v_c_v;
T_cv = [C_c_v -C_c_v*rho_v_c_v; 0 0 0 1];

%Set up the noise parameters for MSCKF calcGNPosEst
%设置MSCKF calcGNPosEst中的噪声参数
v_var = 0.1*ones(3,1);
w_var = 0.1*ones(3,1);
%设置像素噪声方差
y_var = 5^2*ones(2,1);                 % pixel coord var
noiseParams.u_var_prime = y_var(1)/fu^2;
noiseParams.v_var_prime = y_var(2)/fv^2;

%Set up sliding window
%设置LM优化中的参数
LMLambda = 1e-4;
lineLambda = 0.25;
JcostThresh = 0.1e-2;
useMonoCamera = true; %If true, only left camera will be used

%设置滑动窗口尺寸和最大迭代次数
kappa = 10; %Sliding window size
maxOptIter = 5;

kStart = 1;
kEnd = size(y_k_j,2) - kappa - 1 


k1 = kStart;
k2 = k1+kappa;
%滑动窗口中状态个数
K = k2 - k1;  %There are K + 1 total states, since x0 is the k1th state

%% Setup
%根据使用的相机类型确定像素测量维数，单目为2，双目为4
if useMonoCamera
    pixMeasDim = 2;
else
    pixMeasDim = 4;
end

initialStateStruct = {};
    
% Extract noise values
%初始化测量噪声协方差
Q = diag([v_var; w_var]);
%根据使用的相机类型初始化观测噪声，单目为2*2矩阵，双目为4*4单位阵
if useMonoCamera
    R = diag(y_var);
    %R = diag(y_var(1:2));
else
    R = 1*eye(4);
    %R = diag(y_var);
end

%% First create the initial guess using dead reackoning

%Use ground truth for the first state
%根据参考值获得初始化状态C_vi，r_vi_i，k(旋转，位置，ID)
firstState.C_vi = Cfrompsi(theta_vk_i(:,k1));
if isnan(firstState.C_vi(1,1))
    firstState.C_vi = eye(3);
end
firstState.r_vi_i = r_i_vk_i(:,k1);
firstState.k = k1;
initialStateStruct{1} = firstState;


%There are K + 1 states (there is a '0' state)
%通过IMU的测量值（角速度和速度）更新第一个滑动窗口中的所有状态（K=10）
for kIdx = 1:K
    k = kIdx + k1;
    imuMeasurement.omega = w_vk_vk_i(:, k-1);
    imuMeasurement.v = v_vk_vk_i(:, k-1);
    deltaT = t(k) - t(k-1);
    %Propagate the state forward
    initialStateStruct{kIdx+1} = propagateState(initialStateStruct{kIdx}, imuMeasurement, deltaT);
end


%% IMU Only
%通过IMU的测量值（角速度和速度）进行状态更新，用于绘图对比
imuOnlyStateStruct{1} = firstState;
%There are K + 1 states (there is a '0' state)
for k = kStart+1:kEnd+1
    imuMeasurement.omega = w_vk_vk_i(:, k-1);
    imuMeasurement.v = v_vk_vk_i(:, k-1);
    deltaT = t(k) - t(k-1);
    %Propagate the state forward
    imuOnlyStateStruct{k} = propagateState(imuOnlyStateStruct{k-1}, imuMeasurement, deltaT);
end

%%

%Slide the window along
stateVecHistStruct = {};
stateVecHistStruct{1} = firstState;

%stateSigmaHistMat = [];
%动态绘制平面轨迹图，绘制第一个点
figure
plot(-firstState.r_vi_i(2),firstState.r_vi_i(1), '*b')
hold on;
grid on;
for k1 = kStart:kEnd        
k2 = k1+kappa;

% %How many exteroceptive measurements do we have?
% %NOTE: k1 is the 0th state
% totalLandmarkObs = 0;
% observedBinaryFlags = zeros(numLandmarks, 1);
% lmObsVec = zeros(1, K);
% 
% for k = (k1+1):k2
%     validObs = squeeze(y_k_j(1, k, :) > -1);
%     lmObsVec(k-k1) = sum(validObs);
%     observedBinaryFlags(validObs') = ones(1, sum(validObs==1));
%     totalLandmarkObs = totalLandmarkObs + sum(y_k_j(1, k, :) > -1);
% end
% totalUniqueObservedLandmarks = sum(observedBinaryFlags);
% observedLandmarkIds = find(observedBinaryFlags);


%To initialize G-N, we propagate all the states for the first window
%and then only propagate the most recent state, re-using the rest
%初始化高斯牛顿迭代
%如果是第一帧，则用前N(window size)个IMU的预测状态初始化第一个滑动窗口，否则预测最近的一帧状态添加到滑动窗口中
%使用IMU数据更新当前窗口中的状态
if k1 == kStart
        %对于第一帧数据，直接用初始化窗口（initialStateStruct）初始化当前窗口（currentStateStruct）
        currentStateStruct = initialStateStruct;
        %初始化3D点坐标矩阵
        rho_i_pj_i_est = NaN(3, numLandmarks);
else
        %窗口往后滑动一个状态
        currentStateStruct = currentStateStruct(2:end);

        %Extract the measurement
        imuMeasurement.omega = w_vk_vk_i(:, k2-1);
        imuMeasurement.v = v_vk_vk_i(:, k2-1);
        deltaT = t(k2) - t(k2-1);

        %Propagate the state forward
        %用IMU测量数据对窗口中的状态进行预测更新
        currentStateStruct{end+1} = propagateState(currentStateStruct{end}, imuMeasurement, deltaT);
end

% Initialize the landmark positions (start fresh each window)
%初始化特征点位置向量
rho_i_pj_i_est = NaN(3, numLandmarks);
%初始化特征点对应的观测结构体camStates，observations（相机状态，特征点观测）
observedLandmarkStructs = {};
for l_i = 1:numLandmarks
       observedLandmarkStructs{l_i}.camStates = {};
       observedLandmarkStructs{l_i}.observations = [];
end
for kIdx = 1:K
        k = kIdx + k1;
        %获得有效的特征点ID
        validLmObsId = find(y_k_j(1, k, :) > -1);
        kState = currentStateStruct{kIdx+1};
        T_vi = [kState.C_vi -kState.C_vi*kState.r_vi_i; 0 0 0 1];
        T_ci = T_cv*T_vi;
        T_ic = inv(T_ci);
        % 
        % camStates{k}.q_CG        4x1 Global to camera rotation quaternion
        % camStates{k}.p_C_G       3x1 Camera Position in the Global frame
        % camStates{k}.trackedFeatureIds  1xM List of feature ids that are currently being tracked from that camera state
        %
        for lmId = validLmObsId'
            yMeas = y_k_j(:, k, lmId);
            camState = {};
            camState.C_CG = T_ci(1:3,1:3);
            camState.p_C_G = T_ic(1:3,4);
            observedLandmarkStructs{lmId}.camStates{end+1} = camState;
            observedLandmarkStructs{lmId}.observations(:, end+1) = [(yMeas(1) - calibParams.c_u)/calibParams.f_u; (yMeas(2) - calibParams.c_v)/calibParams.f_v];

            %Find the ground truth position of the observed landmark
            %rho_pi_i_check = rho_i_pj_i(:, lmId);

%             if (isnan(rho_i_pj_i_est(1, lmId)))
%                 %Use triangulation to find the position of the landmark
%                  rho_pc_c = triangulate(yMeas, calibParams);
%                  
%                  rho_pi_i = kState.C_vi'*(vehicleCamTransform.C_cv'*rho_pc_c + vehicleCamTransform.rho_cv_v) +  kState.r_vi_i;
%                  rho_i_pj_i_est(:, lmId) = rho_pi_i;
%                  
%                  %Use ground truth for now
%                  %rho_i_pj_i_est(:, lmId) = rho_i_pj_i(:, lmId);
%             end
        end
end

%Triangulate all landmarks and keep track of which ones we are
%triangulating
totalLandmarkObs = 0;
observedLandmarkIds = [];
totalUniqueObservedLandmarks = 0;
for lmId = 1:length(observedLandmarkStructs)
    if length(observedLandmarkStructs{lmId}.camStates) > 1
        camStates = observedLandmarkStructs{lmId}.camStates;
        observations = observedLandmarkStructs{lmId}.observations;
        [rho_pi_i, Jcost, RCOND] = calcGNPosEst(camStates, observations, noiseParams);
        if Jcost < JcostThresh*length(camStates)^2
            rho_i_pj_i_est(:, lmId) = rho_pi_i;
            totalLandmarkObs = totalLandmarkObs + length(observedLandmarkStructs{lmId}.camStates);
            totalUniqueObservedLandmarks = totalUniqueObservedLandmarks + 1;
            observedLandmarkIds(end+1) = lmId;
        end
    end
end


%Define the optimal state
optimalStateStruct = currentStateStruct;
Jbest = Inf;
dx = Inf;

for optIdx = 1:maxOptIter+1

%Error Vector
errorVector = NaN(6*K+pixMeasDim*totalLandmarkObs, 1);
%This helper index will keep track of where we need to insert our next
%errors
errorVectorHelperIdx = 1;

%H and T
H = sparse(6*K+pixMeasDim*totalLandmarkObs, 6*(K+1) + 3*totalUniqueObservedLandmarks);
T = sparse(6*K+pixMeasDim*totalLandmarkObs, 6*K+pixMeasDim*totalLandmarkObs);

%Helper indices that keep track of the row number of the last block entry
%into H and T
HHelperIdx = 1;
THelperIdx = 1;


for kIdx = 1:K
    k = kIdx + k1;
    imuMeasurement.omega = w_vk_vk_i(:, k-1);
    imuMeasurement.v = v_vk_vk_i(:, k-1);
    deltaT = t(k) - t(k-1);
    
    %==== Build the interoceptive error and Jacobians=====%
    %Note that there are K+1 states (the 0th state is the 1st element)
    kState = currentStateStruct{kIdx+1};
    kMinus1State = currentStateStruct{kIdx};
    
    intErrorVec = imuError(kState, kMinus1State, imuMeasurement, deltaT);
    H_x_k =  H_xfn(kMinus1State, imuMeasurement, deltaT );
    H_w_k = H_wfn(kMinus1State);
    
    
    %==== Build the exteroceptive error and Jacobians=====%
    validLmObsId = intersect(find(y_k_j(1, k, :) > -1), observedLandmarkIds);
    
    if kIdx == 1 && optIdx == 1
        fprintf('Tracking %d features. \n', length(validLmObsId));
    end

    if ~isempty(validLmObsId)
        
        extErrorVec = NaN(pixMeasDim*length(validLmObsId), 1);
        G_x_k = NaN(pixMeasDim*length(validLmObsId),6);
        %Jacobians wrt feature position
        G_x_f_k = NaN(pixMeasDim*length(validLmObsId), 3);
        
        
        idx = 1;
        for lmId = validLmObsId'
            
            yMeas = y_k_j(:, k, lmId);
           
            rho_pi_i = rho_i_pj_i_est(:,lmId);
            
            stereoError = stereoCamError(yMeas, kState, vehicleCamTransform, rho_pi_i, calibParams);
            
            [G_x_k_state, G_x_k_feat] = G_xfn(kState, vehicleCamTransform, rho_pi_i, calibParams, useMonoCamera);

            %Use stereo or monocular errors
            if useMonoCamera
                extErrorVec(idx:idx+1, 1) = stereoError(1:2);
                
                G_x_k(idx:idx+1, :) = G_x_k_state;
                G_x_f_k(idx:idx+1, :) = G_x_k_feat;
                idx = idx + 2;
            else
                extErrorVec(idx:idx+3, 1) = stereoError;
                
                G_x_k(idx:idx+3, :) = G_x_k_state;
                G_x_f_k(idx:idx+3, :) = G_x_k_feat;
                idx = idx + 4;
            end
        end
    else
        extErrorVec = [];
    end
    
    %Update matrices 
    %==== Error vector =====
    combinedErrorVec = [intErrorVec; extErrorVec];
    errorVector(errorVectorHelperIdx:(errorVectorHelperIdx + length(combinedErrorVec) - 1) ,1) = combinedErrorVec;
    errorVectorHelperIdx = errorVectorHelperIdx + length(combinedErrorVec);
    
    %==== H matrix =====    
    Hblock = zeros(6+pixMeasDim*length(validLmObsId), 12);
    Hblock(1:6,1:6) = -H_x_k;
    Hblock(1:6,7:12) = eye(6);
    
    if ~isempty(validLmObsId)
        Hblock(7:(7+pixMeasDim*length(validLmObsId) - 1), 7:12) = -G_x_k;
    end
    Hblockrows = size(Hblock, 1);
    
    H(HHelperIdx:(HHelperIdx + Hblockrows - 1), 1+6*(kIdx-1):12+6*(kIdx-1) ) = Hblock;
    
    %Add the feature Jacobians
    lmNum = 1;
    for lmId = validLmObsId'
        rowIdx = pixMeasDim*(lmNum-1)+1;
        
        colLmId = find(observedLandmarkIds == lmId);
        colIdx = 6*(K+1)+3*colLmId-2;
        
        H(HHelperIdx+5+rowIdx:HHelperIdx+rowIdx+5+(pixMeasDim-1), colIdx:colIdx+2) = -G_x_f_k(rowIdx:rowIdx+pixMeasDim-1, :);
        lmNum = lmNum + 1;
    end
    
    HHelperIdx = HHelperIdx + Hblockrows;
    
    %==== T matrix =====
    T_k = zeros(6+pixMeasDim*length(validLmObsId), 6+pixMeasDim*length(validLmObsId));
    T_k(1:6, 1:6) = H_w_k*Q*deltaT^2*H_w_k';
    %Here, G_n_k is identity, so we can just repeat the variances along the
    %diagonal
    obsVar = diag(R);
    T_k(7:end, 7:end) = diag(repmat(obsVar, [length(validLmObsId),1]));
    Tksize = size(T_k, 1);
    T(THelperIdx:(THelperIdx + Tksize - 1), THelperIdx:(THelperIdx + Tksize - 1)) = T_k;
    THelperIdx = THelperIdx + Tksize;
end

    H = H(:, 7:end);

    %Calculate scalar objective
    Jnew = 0.5*errorVector'*(T\errorVector);
    
    
    if Jnew < Jbest 
        optimalStateStruct = currentStateStruct;
        Jbest = Jnew;
    end
    
    %Check for convergence
    if norm(dx) < 1e-3
        disp('Converged!')
        break;
    end

    % Solve for the optimal step size!
    if optIdx <= maxOptIter
        dx = (H'*(T\H) + LMLambda*diag(diag(H'*(T\H))))\(-H'*(T\errorVector));
        [currentStateStruct, rho_i_pj_i_est] = updateStateStruct(currentStateStruct, observedLandmarkIds, rho_i_pj_i_est,  lineLambda*dx);
    end
   
end %End optimization iterations

% error = 0;
% for i=1:numLandmarks
%     if ~isnan(rho_i_pj_i_est(1,i))
%         error = error + norm(rho_i_pj_i_est(:,i) - rho_i_pj_i(:,i));
%     end
% end
% error = error/numLandmarks
currentStateStruct = optimalStateStruct;


if optIdx == maxOptIter
    fprintf('Warning: Failed to converge! \n');
end

fprintf('%d done. J = %.5f. %d iterations. \n', k1, Jbest, optIdx)

%Extract variance of states
%stateCov = inv(H'*(T\H) + LMLambda*eye(size(H,2)));
%stateVar = diag(stateCov);

%Keep track of the first state in the window
% if ~all(stateVar > 0)
%     warning('Variances not positive');
% end
stateVecHistStruct{end+1} = currentStateStruct{2};
plot(-currentStateStruct{2}.r_vi_i(2),currentStateStruct{2}.r_vi_i(1), '*b')
plot(-imuOnlyStateStruct{k1}.r_vi_i(2),imuOnlyStateStruct{k1}.r_vi_i(1), '*r')
plot(-r_i_vk_i(2,k1),r_i_vk_i(1,k1), '*k');


drawnow;
%stateSigmaHistMat(:,k1 - kStart + 1) = sqrt(abs(stateVar(1:6)));

end %End Sliding window

toc

% sigma_x = (stateSigmaHistMat(1,:));
% sigma_y = (stateSigmaHistMat(2,:));
% sigma_z = (stateSigmaHistMat(3,:));
% sigma_th1 = (stateSigmaHistMat(4,:));
% sigma_th2 = (stateSigmaHistMat(5,:));
% sigma_th3 = (stateSigmaHistMat(6,:));



%% Plot error and variances
%addpath('/Users/valentinp/Research/MATLAB/export_fig'); %Use Oliver Woodford's awesome export_fig package to get trimmed PDFs
addpath('../msckf/utils')
rotErrVec = zeros(3, length(stateVecHistStruct));
transErrVec = zeros(3, length(stateVecHistStruct));
transErrVecImu = zeros(3, length(stateVecHistStruct));
rotErrVecImu = zeros(3, length(stateVecHistStruct));

translation = zeros(3, length(stateVecHistStruct));
translation_imuonly = zeros(3, length(stateVecHistStruct));
 
for stIdx = 1:length(stateVecHistStruct)
    translation(:, stIdx) = stateVecHistStruct{stIdx}.r_vi_i;
    translation_imuonly(:,stIdx) = imuOnlyStateStruct{stIdx}.r_vi_i;
    transErrVec(:,stIdx) = stateVecHistStruct{stIdx}.r_vi_i - r_i_vk_i(:,stIdx);
    transErrVecImu(:,stIdx) = imuOnlyStateStruct{stIdx}.r_vi_i - r_i_vk_i(:,stIdx);
    eRotMat = eye(3) - stateVecHistStruct{stIdx}.C_vi*axisAngleToRotMat(theta_vk_i(:,stIdx))';
    rotErrVec(:, stIdx) = [eRotMat(3,2); eRotMat(1,3); eRotMat(2,1)];
    
    eRotMat = eye(3) - imuOnlyStateStruct{stIdx}.C_vi*axisAngleToRotMat(theta_vk_i(:,stIdx))';
    rotErrVecImu(:, stIdx) = [eRotMat(3,2); eRotMat(1,3); eRotMat(2,1)];
end



figure
plot3(-translation(2,:),translation(1,:),translation(3,:), '-b');
hold on
plot3(-translation_imuonly(2,:),translation_imuonly(1,:),translation_imuonly(3,:), '-r');
plot3(-r_i_vk_i(2,1:length(stateVecHistStruct)),r_i_vk_i(1,1:length(stateVecHistStruct)),r_i_vk_i(3,1:length(stateVecHistStruct)), '-k');
legend('Opt', 'IMU', 'Ground Truth')
armse_imu = mean(sqrt(sum(transErrVecImu.^2,1)/3))
armse_imu_rot = mean(sqrt(sum(rotErrVecImu(:,2:end).^2,1)/3))

armse = mean(sqrt(sum(transErrVec.^2,1)/3))
armse_rot = mean(sqrt(sum(rotErrVec(:,2:end).^2,1)/3))


norm(transErrVecImu(:,end))
norm(transErrVec(:,end))

save(sprintf('../KITTI Trials/SWF_%s', fileName));
%%
% numFeat = NaN(1, 100);
% for f_i = 1:100
%    numFeat(f_i) = sum(y_k_j(1,f_i,:) > -1);
% end
% plot(numFeat)
% Save estimates
% swf_trans_err = transErrVec;
% swf_rot_err = rotErrVec;
% save(sprintf('swf_%d_%d_%d_%s.mat',kStart,kEnd, kappa, fileName), 'swf_trans_err', 'swf_rot_err', 'stateSigmaHistMat');


% transLim = 0.5;
% rotLim = 0.5;
% recycleStates = 'Yes';
% 
% figure
% subplot(3,1,1)
% plot(t(kStart:kEnd), transErrVec(1,:), 'LineWidth', 1.2)
% hold on
% plot(t(kStart:kEnd), 3*sigma_x, '--r')
% plot(t(kStart:kEnd), -3*sigma_x, '--r')
% ylim([-transLim transLim])
% title(sprintf('Translational Error (\\kappa = %d, Recycle States? %s)', kappa, recycleStates))
% ylabel('\delta r_x')
% 
% 
% subplot(3,1,2)
% plot(t(kStart:kEnd), transErrVec(2,:), 'LineWidth', 1.2)
% hold on
% plot(t(kStart:kEnd), 3*sigma_y, '--r')
% plot(t(kStart:kEnd), -3*sigma_y, '--r')
% ylim([-transLim transLim])
% ylabel('\delta r_y')
% 
% subplot(3,1,3)
% plot(t(kStart:kEnd), transErrVec(3,:), 'LineWidth', 1.2)
% hold on
% plot(t(kStart:kEnd), 3*sigma_z, '--r')
% plot(t(kStart:kEnd), -3*sigma_z, '--r')
% ylim([-transLim transLim])
% ylabel('\delta r_z')
% xlabel('t_k')
% %set(gca,'FontSize',12)
% %set(findall(gcf,'type','text'),'FontSize',12)
% 
% figure
% subplot(3,1,1)
% plot(t(kStart:kEnd), rotErrVec(1,:), 'LineWidth', 1.2)
% hold on
% plot(t(kStart:kEnd), 3*sigma_th1, '--r')
% plot(t(kStart:kEnd), -3*sigma_th1, '--r')
% ylim([-rotLim rotLim])
% title(sprintf('Rotational Error (\\kappa = %d, Recycle States? %s)', kappa, recycleStates))
% ylabel('\delta\theta_x')
% 
%  
% subplot(3,1,2)
% plot(t(kStart:kEnd), rotErrVec(2,:), 'LineWidth', 1.2)
% hold on
% plot(t(kStart:kEnd), 3*sigma_th2, '--r')
% plot(t(kStart:kEnd), -3*sigma_th2, '--r')
% ylim([-rotLim rotLim])
% ylabel('\delta\theta_y')
% 
% subplot(3,1,3)
% plot(t(kStart:kEnd), rotErrVec(3,:), 'LineWidth', 1.2)
% hold on
% plot(t(kStart:kEnd), 3*sigma_th3, '--r')
% plot(t(kStart:kEnd), -3*sigma_th3, '--r')
% ylim([-rotLim rotLim])
% ylabel('\delta\theta_z')
% xlabel('t_k')