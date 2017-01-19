%% Clean up and import
clc;
close all;
clear;
addpath('utils');
addpath('utils/devkit');


dataBaseDir = '/Users/valentinp/Desktop/KITTI/2011_09_26/2011_09_26_drive_0019_sync';
dataCalibDir = '/Users/valentinp/Desktop/KITTI/2011_09_26';

%% Get ground truth and import data
frameRange = 1:250;
%Image data
leftImageData = loadImageData([dataBaseDir '/image_00'], frameRange);
rightImageData = loadImageData([dataBaseDir '/image_01'], frameRange);
%IMU data
[imuData, imuFrames] = loadImuData(dataBaseDir, leftImageData.timestamps);
%Ground Truth
T_wIMU_GT = getGroundTruth(dataBaseDir, imuFrames);

skipFrames = 1;
minObsNum = 3;
ransacMaxDistance = 2;

%% Load calibration
[T_camvelo_struct, P_rect_cam1] = loadCalibration(dataCalibDir);
T_camvelo = T_camvelo_struct{1}; 
T_veloimu = loadCalibrationRigid(fullfile(dataCalibDir,'calib_imu_to_velo.txt'));
T_camimu = T_camvelo*T_veloimu;

%Add camera ground truth

T_wCam_GT = T_wIMU_GT;

for i = 1:size(T_wIMU_GT, 3)
    T_wCam_GT(:,:,i) = T_wIMU_GT(:,:,i)*inv(T_camimu);
end

%% Process images

seenFeatureStructs = {};
currentlyObservedIds = [];
lastLeftPoints = [];
lastRightPoints = [];
oldFeatureIdx = [];


numFrames = size(leftImageData.rectImages, 3);
detectNewPoints = false;

for image_i=1:numFrames
    fprintf('Processing image %d \n', image_i);
    %Extract stereo images
    viLeftImage = uint8(leftImageData.rectImages(:,:,image_i));
    viRightImage = uint8(rightImageData.rectImages(:,:,image_i));
   
    
   if image_i == 1
       %Extract features and initialize points
       [ inlierPointsLeft, inlierPointsRight ] = extractBinnedFeatures( viLeftImage, viRightImage);
       pointTrackerL = vision.PointTracker('MaxBidirectionalError', 5);
       initialize(pointTrackerL, inlierPointsLeft, viLeftImage);

       pointTrackerR = vision.PointTracker('MaxBidirectionalError', 5);
       initialize(pointTrackerR, inlierPointsRight, viLeftImage);
    
       lastLeftPoints = inlierPointsLeft;
       lastRightPoints = inlierPointsRight;
       
       %Initialize structs
       for obs_i = 1:size(inlierPointsLeft,1)
            seenFeatureStructs{obs_i}.leftPixels = inlierPointsLeft(obs_i, :)';
            seenFeatureStructs{obs_i}.rightPixels = inlierPointsRight(obs_i, :)';
            seenFeatureStructs{obs_i}.imageIndex = image_i;
            currentlyObservedIds(end+1) = obs_i;
       end
       
   elseif detectNewPoints
        detectNewPoints = false; 
        
        %Step all of the old points through
        [validLeftPoints, isFoundL] = step(pointTrackerL, viLeftImage);
        [validRightPoints, isFoundR] = step(pointTrackerR, viRightImage);

        %RANSAC
        [~,inlierPtsPastLeft,inlierPtsCurrentLeft] = estimateGeometricTransform(lastLeftPoints,validLeftPoints,'similarity', 'MaxDistance', ransacMaxDistance);
        ransacInliers = ismember(validLeftPoints,inlierPtsCurrentLeft);
        ransacInliers = ransacInliers(:,1);
        
        trackedIdx = find(ransacInliers & isFoundL & isFoundR & validLeftPoints(:,1) > 0 & validLeftPoints(:,2) > 0 ...
             & validRightPoints(:,1) > 0 & validRightPoints(:,2) > 0 ...
             & abs(validLeftPoints(:, 2) - validRightPoints(:, 2)) <= 1);
        
        
         trackedLeftPoints = validLeftPoints(trackedIdx,:);
         trackedRightPoints = validRightPoints(trackedIdx,:);
         currentlyObservedIds = currentlyObservedIds(trackedIdx);
         
         %Add observations of old points
            j = 1;
            for struct_i = currentlyObservedIds
                seenFeatureStructs{struct_i}.leftPixels(:, end+1) = trackedLeftPoints(j, :)';
                seenFeatureStructs{struct_i}.rightPixels(:, end+1) = trackedRightPoints(j, :)';
                seenFeatureStructs{struct_i}.imageIndex(end+1) = image_i;
                j = j+1;
            end
            
         %Add new points
         [newPointsLeft, newPointsRight ] = extractBinnedFeatures( viLeftImage, viRightImage);
          
         lastFeatureId = length(seenFeatureStructs);
         newFeatureIds = lastFeatureId+1:lastFeatureId+size(newPointsLeft,1);
         j = 1;
          for obs_i = newFeatureIds
            seenFeatureStructs{obs_i}.leftPixels = newPointsLeft(j, :)';
            seenFeatureStructs{obs_i}.rightPixels = newPointsRight(j, :)';
            seenFeatureStructs{obs_i}.imageIndex = image_i;
            j = j +1;
          end
          
          %Add ids of new points
          currentlyObservedIds = [currentlyObservedIds newFeatureIds];
         
        %Track old points and new points  
        setPoints(pointTrackerL,[trackedLeftPoints; newPointsLeft]);
        setPoints(pointTrackerR, [trackedRightPoints; newPointsRight]); 
        
        lastLeftPoints = [trackedLeftPoints; newPointsLeft];
        lastRightPoints = [trackedRightPoints; newPointsRight];
     

   else
       
         %Step all of the old points through
        [validLeftPoints, isFoundL] = step(pointTrackerL, viLeftImage);
        [validRightPoints, isFoundR] = step(pointTrackerR, viRightImage);
        
        %RANSAC
        [~,inlierPtsPastLeft,inlierPtsCurrentLeft] = estimateGeometricTransform(lastLeftPoints,validLeftPoints,'similarity', 'MaxDistance', ransacMaxDistance);
        ransacInliers = ismember(validLeftPoints,inlierPtsCurrentLeft);
        ransacInliers = ransacInliers(:,1);

        trackedIdx = find(ransacInliers & isFoundL & isFoundR & validLeftPoints(:,1) > 0 & validLeftPoints(:,2) > 0 ...
             & validRightPoints(:,1) > 0 & validRightPoints(:,2) > 0 ...
             & abs(validLeftPoints(:, 2) - validRightPoints(:, 2)) <= 1);
        
         trackedLeftPoints = validLeftPoints(trackedIdx, :);
         trackedRightPoints = validRightPoints(trackedIdx, :);
         currentlyObservedIds = currentlyObservedIds(trackedIdx);
         
         %Add observations of old points
            j = 1;
            for struct_i = currentlyObservedIds
                seenFeatureStructs{struct_i}.leftPixels(:, end+1) = trackedLeftPoints(j, :)';
                seenFeatureStructs{struct_i}.rightPixels(:, end+1) = trackedRightPoints(j, :)';
                seenFeatureStructs{struct_i}.imageIndex(end+1) = image_i;
                j = j+1;
            end
            
         if length(trackedIdx) < 50
             detectNewPoints = true;
         end
         
         fprintf('Tracking %d features. \n', length(trackedIdx));

           setPoints(pointTrackerL, trackedLeftPoints);
           setPoints(pointTrackerR, trackedRightPoints); 

            lastLeftPoints = trackedLeftPoints;
            lastRightPoints = trackedRightPoints;
     
           showMatchedFeatures(viLeftImage, viRightImage, trackedLeftPoints, trackedRightPoints);
   end
    %=====Temporal tracking======
end

%% Prune points with low observation count

pruneIds = [];
totObs = 0;
for p_i = 1:length(seenFeatureStructs)
    totObs = totObs + length(seenFeatureStructs{p_i}.imageIndex);
    trackLength = norm(seenFeatureStructs{p_i}.leftPixels(:, 1) - seenFeatureStructs{p_i}.leftPixels(:, end));
    
    lp = abs(diff(sum(seenFeatureStructs{p_i}.leftPixels, 1)));
    rp = abs(diff(sum(seenFeatureStructs{p_i}.leftPixels, 1)));
    
    maxPixDiff = max([lp rp]);
    minPixDiff = min([lp rp]);

    if length(seenFeatureStructs{p_i}.imageIndex) < minObsNum  %|| trackLength < 5 || maxPixDiff > 50 || minPixDiff < 1
        pruneIds(end+1) = p_i;
    end
end

fprintf('%d features to prune. \n',length(pruneIds));
fprintf('%d remaining features. \n', length(seenFeatureStructs) - length(pruneIds));

seenFeatureStructsPruned = removeCells(seenFeatureStructs, pruneIds);

%% Plot feature tracks

figure
subplot(2,1,1);
plot(0,0);
hold on;
for f_i = 1:length(seenFeatureStructsPruned)
    plot(seenFeatureStructsPruned{f_i}.leftPixels(1,:), seenFeatureStructsPruned{f_i}.leftPixels(2,:));
end
subplot(2,1,2);
plot(0,0);
hold on;
for f_i = 1:length(seenFeatureStructsPruned)
    plot(seenFeatureStructsPruned{f_i}.rightPixels(1,:), seenFeatureStructsPruned{f_i}.rightPixels(2,:));
end

%%

% y_k_j, 4 x K x 20
y_k_j = -1*ones(4, length(1:skipFrames:numFrames), length(seenFeatureStructsPruned));

data_i = 1;
for image_i = 1:skipFrames:numFrames
    fprintf('Pruning features from image %d \n', image_i);
    for p_i = 1:length(seenFeatureStructsPruned)
        [isObs, idx] = ismember(image_i, seenFeatureStructsPruned{p_i}.imageIndex);
        if isObs
            y_k_j(:, data_i, p_i) = [seenFeatureStructsPruned{p_i}.leftPixels(:,idx); seenFeatureStructsPruned{p_i}.rightPixels(:,idx)];
        end
    end
    data_i = data_i + 1;
end

%% Export variables
w_vk_vk_i = imuData.measOmega;
v_vk_vk_i = imuData.measVel;

K= P_rect_cam1(:,1:3);
b_pix = P_rect_cam1(1,4);

cu = K(1,3);
cv = K(2,3);
fu = K(1,1);
fv = K(2,2);
b = -b_pix/fu; %The KITTI calibration supplies the baseline in units of pixels


R_ci = T_camimu(1:3,1:3);
T_imucam = inv(T_camimu);
p_ci_i = T_imucam(1:3,4)


C_c_v = R_ci;
rho_v_c_v = p_ci_i;

f = strsplit(dataBaseDir, '/');
f = strsplit(char(f(end)), '.');
fileName = [char(f(1)) '_KLT.mat'];

size(w_vk_vk_i)
size(v_vk_vk_i)
size(y_k_j)

r_i_vk_i = NaN(3, size(T_wIMU_GT,3));
theta_vk_i = NaN(3, size(T_wIMU_GT,3));

for j = 1:size(T_wIMU_GT,3)
r_i_vk_i(:,j) = T_wIMU_GT(1:3, 4, j);
R_wIMU = T_wIMU_GT(1:3,1:3,j);
theta = acos((trace(R_wIMU)-1)*0.5);
if theta < eps
    w = zeros(3,1);
else
    w = 1/(2*sin(theta))*[R_wIMU(3,2) - R_wIMU(2,3); R_wIMU(1,3) - R_wIMU(3,1); R_wIMU(2,1) - R_wIMU(1,2)];
end
theta_vk_i(:,j) = theta*w;
end

t = leftImageData.timestamps;
save(['../datasets/' fileName], 'r_i_vk_i','theta_vk_i','w_vk_vk_i','v_vk_vk_i', 'cu','cv','fu','fv','b', 'y_k_j', 'C_c_v', 'rho_v_c_v', 't');

%%
for i = 1:size(y_k_j,2)
    numValidFeatures(i) = sum(y_k_j(1,i,:) ~= -1);
%     find(y_k_j(1,i,:) ~= -1)
end
figure; plot(numValidFeatures(numValidFeatures >= 3),'*g'); hold on;
plot(numValidFeatures(numValidFeatures < 3),'*r');

%% Plot Stuff!

% figure 
% subplot(2,2,1)
% hist(imuSanitizedData(4, :), 50)
% subplot(2,2,2)
% hist(imuSanitizedData(5, :), 50)
% subplot(2,2,3)
% hist(imuSanitizedData(6, :),50)

% figure
% plot(imuData(4, :))
% hold on
% plot(imuData(5, :))
% plot(imuData(6, :))

% figure 
% subplot(2,2,1)
% hist(imuData(1, :), 50)
% subplot(2,2,2)
% hist(imuData(2, :), 50)
% subplot(2,2,3)
% hist(imuData(3, :),50)
% 
% 
% mean(imuData(1, :))
% mean(imuData(2, :))
% mean(imuData(3, :))
