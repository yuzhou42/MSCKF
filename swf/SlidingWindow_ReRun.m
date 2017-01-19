% Sliding Window Gauss Newton Optimization
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clc
clear
close all
load('../KITTI Trials/SWF_2011_09_26_drive_0051_sync_KLT.mat');
%k1,k2为窗口首尾
k1 = kStart;
k2 = k1+kappa;
%K+1为窗口状态
K = k2 - k1;  %There are K + 1 total states, since x0 is the k1th state
% v_var = 0.1*ones(3,1);
% w_var = 0.1*ones(3,1);
% y_var = 0.5^2*ones(2,1);                 % pixel coord var
% JcostThresh = 0.1e-2;

%% Setup
if useMonoCamera
    pixMeasDim = 2;
else
    pixMeasDim = 4;
end

initialStateStruct = {};
    
% Extract noise values
Q = diag([v_var; w_var]);
if useMonoCamera
    R = diag(y_var);
    %R = diag(y_var(1:2));
else
    R = 1*eye(4);
    %R = diag(y_var);
end

%% First create the initial guess using dead reackoning

%Use ground truth for the first state
firstState.C_vi = Cfrompsi(theta_vk_i(:,k1));
if isnan(firstState.C_vi(1,1))
    firstState.C_vi = eye(3);
end
firstState.r_vi_i = r_i_vk_i(:,k1);
firstState.k = k1;
initialStateStruct{1} = firstState;


%There are K + 1 states (there is a '0' state)
for kIdx = 1:K
    k = kIdx + k1;
    imuMeasurement.omega = w_vk_vk_i(:, k-1);
    imuMeasurement.v = v_vk_vk_i(:, k-1);
    deltaT = t(k) - t(k-1);
    %Propagate the state forward
    initialStateStruct{kIdx+1} = propagateState(initialStateStruct{kIdx}, imuMeasurement, deltaT);
end


%% IMU Only
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
if k1 == kStart
        currentStateStruct = initialStateStruct;
        rho_i_pj_i_est = NaN(3, numLandmarks);
else
        currentStateStruct = currentStateStruct(2:end);

        %Extract the measurement
        imuMeasurement.omega = w_vk_vk_i(:, k2-1);
        imuMeasurement.v = v_vk_vk_i(:, k2-1);
        deltaT = t(k2) - t(k2-1);

        %Propagate the state forward
        currentStateStruct{end+1} = propagateState(currentStateStruct{end}, imuMeasurement, deltaT);
end

% Initialize the landmark positions (start fresh each window)
rho_i_pj_i_est = NaN(3, numLandmarks);
observedLandmarkStructs = {};
for l_i = 1:numLandmarks
       observedLandmarkStructs{l_i}.camStates = {};
       observedLandmarkStructs{l_i}.observations = [];
end
for kIdx = 1:K
        k = kIdx + k1;
        %获得第k帧相机中的有效特征点
        validLmObsId = find(y_k_j(1, k, :) > -1);
        %
        kState = currentStateStruct{kIdx+1};
        T_vi = [kState.C_vi -kState.C_vi*kState.r_vi_i; 0 0 0 1];
        T_ci = T_cv*T_vi;
        T_ic = inv(T_ci);
            
        % camStates{k}.q_CG        4x1 Global to camera rotation quaternion
        % camStates{k}.p_C_G       3x1 Camera Position in the Global frame
        % camStates{k}.trackedFeatureIds  1xM List of feature ids that are currently being tracked from that camera state
        %更新特征点观测结构体observedLandmarkStructs
        for lmId = validLmObsId'
            %获取第k帧的有效所有有效特征观测
            yMeas = y_k_j(:, k, lmId);
            %获取Global坐标系下当前帧相机状态C_CG，p_C_G（旋转，平移）
            camState = {};
            camState.C_CG = T_ci(1:3,1:3);
            camState.p_C_G = T_ic(1:3,4);
            %将当前帧相机状态添加到特征点观测结构体observedLandmarkStructs{lmId}.camStates中
            observedLandmarkStructs{lmId}.camStates{end+1} = camState;
            %将所有特征观测像素坐标转为相机坐标系下的齐次坐标添加到特征点观测结构体observedLandmarkStructs{lmId}.observations中
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
%能观测到所有特征的全部相机数目
totalLandmarkObs = 0;
%可以被观测到的特征ID
observedLandmarkIds = [];
%特征点个数
totalUniqueObservedLandmarks = 0;
%遍历观测特征点结构体observedLandmarkStructs中的每个特征点,恢复特征点3D空间坐标，并获取相应参数
for lmId = 1:length(observedLandmarkStructs)
    %如果能观测到当前特征的相机数大于1
    if length(observedLandmarkStructs{lmId}.camStates) > 1
        %得到能观测到该特征的所有相机状态和特征点在对应相机坐标系下的齐次坐标
        camStates = observedLandmarkStructs{lmId}.camStates;
        observations = observedLandmarkStructs{lmId}.observations;
        %三角化恢复特征点3D空间坐标（高斯牛顿优化方法）
        [rho_pi_i, Jcost, RCOND] = calcGNPosEst(camStates, observations, noiseParams);
        if Jcost < JcostThresh*length(camStates)^2
            %将特征点3D空间坐标位置添加到特征点列表rho_i_pj_i_est中
            rho_i_pj_i_est(:, lmId) = rho_pi_i;
            %获取能观测到所有特征的全部相机数目
            totalLandmarkObs = totalLandmarkObs + length(observedLandmarkStructs{lmId}.camStates);
            %获取特征点个数
            totalUniqueObservedLandmarks = totalUniqueObservedLandmarks + 1;
            %将特征点ID号添加到observedLandmarkIds中
            observedLandmarkIds(end+1) = lmId;
        end
    end
end


%Define the optimal state
%定义最优状态
optimalStateStruct = currentStateStruct;
Jbest = Inf;
dx = Inf;

for optIdx = 1:maxOptIter+1

%Error Vector
%定义误差向量errorVector[滑动窗口内K个相机位姿（6*K）；M个特征点位置(2*M)]，参考文献1：公式1
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
stateCov = inv(H'*(T\H) + LMLambda*eye(size(H,2)));
stateVar = diag(stateCov);

%Keep track of the first state in the window
% if ~all(stateVar > 0)
%     warning('Variances not positive');
% end
stateVecHistStruct{end+1} = currentStateStruct{2};
plot(-currentStateStruct{2}.r_vi_i(2),currentStateStruct{2}.r_vi_i(1), '*b')
plot(-imuOnlyStateStruct{k1}.r_vi_i(2),imuOnlyStateStruct{k1}.r_vi_i(1), '*r')
plot(-r_i_vk_i(2,k1),r_i_vk_i(1,k1), '*k');


drawnow;
stateSigmaHistMat(:,k1 - kStart + 1) = sqrt(abs(stateVar(1:6)));

end %End Sliding window

toc

sigma_x = (stateSigmaHistMat(1,:));
sigma_y = (stateSigmaHistMat(2,:));
sigma_z = (stateSigmaHistMat(3,:));
sigma_th1 = (stateSigmaHistMat(4,:));
sigma_th2 = (stateSigmaHistMat(5,:));
sigma_th3 = (stateSigmaHistMat(6,:));



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

save(sprintf('../KITTI Trials/SWF_RERUN_%s', fileName));
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