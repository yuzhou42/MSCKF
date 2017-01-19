%% Clean up and import
clc;
close all;
addpath('utils');
addpath('utils/devkit');


dataBaseDir = '~/Desktop/KITTI/2011_09_26/2011_09_26_drive_0095_sync';
dataCalibDir = '~/Desktop/KITTI/2011_09_26';

%% Get ground truth and import data
frameRange = 1:265;
%Image data
leftImageData = loadImageData([dataBaseDir '/image_00'], frameRange);
rightImageData = loadImageData([dataBaseDir '/image_01'], frameRange);
%IMU data
[imuData, imuFrames] = loadImuData(dataBaseDir, leftImageData.timestamps);
%Ground Truth
T_wIMU_GT = getGroundTruth(dataBaseDir, imuFrames);

skipFrames = 1;
minObsNum = 2;

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

meanErrorHist = [];
errorVecHist = [];
bmHist = [];
imuDataHist = [];

seenFeatureVectors = [];
seenFeatureStructs = {};

numFrames = size(leftImageData.rectImages, 3);

for i=1:skipFrames:numFrames
    fprintf('Processing image %d \n', i);
    %Extract stereo images
    
    viLeftImage = uint8(leftImageData.rectImages(:,:,i));
    viRightImage = uint8(rightImageData.rectImages(:,:,i));
    
    %Detect strongest corners
    leftPoints = detectSURFFeatures(viLeftImage);
    rightPoints = detectSURFFeatures(viRightImage);
    
    leftPoints = leftPoints.selectStrongest(100);
    rightPoints = rightPoints.selectStrongest(100);
    
    %Extract features and stereo match
   [featuresLeft, validLeftPoints] = extractFeatures(viLeftImage, leftPoints);
   [featuresRight, validRightPoints] = extractFeatures(viRightImage, rightPoints);

 
   
    indexPairs = matchFeatures(featuresLeft, featuresRight);
    matchedPointsLeft = validLeftPoints(indexPairs(:, 1), :);
    featuresLeft = featuresLeft(indexPairs(:, 1), :);
    matchedPointsRight = validRightPoints(indexPairs(:, 2), :);
    
    inliers = abs((matchedPointsLeft.Location(:, 2) - matchedPointsRight.Location(:, 2))) <= 0.25;
    
    matchedPointsLeft = matchedPointsLeft(inliers, :);
    matchedPointsRight = matchedPointsRight(inliers, :);
    matchedFeatureVectors = featuresLeft(inliers, :);
    
    %Use ransac to remove bad matches, then draw the matches
    try
%        [~,matchedPointsLeft,matchedPointsRight] = estimateGeometricTransform(matchedPointsLeft,matchedPointsRight,'similarity', 'MaxDistance', 2);
%        [matchedFeatureVectors, ~] = extractFeatures(viLeftImage, matchedPointsLeft);
%         
       showMatchedFeatures(viLeftImage, viRightImage, matchedPointsLeft, matchedPointsRight);
        drawnow;
    catch
        break;
    end    
    
    %=====Temporal tracking=======
    
    %Have any of these features been seen before (compare to seenFeatureVectors)?
    if ~isempty(seenFeatureVectors)
        indexGlobalPairs = matchFeatures(single(seenFeatureVectors), matchedFeatureVectors);
        newFeatureIdx = setdiff(1:size(matchedFeatureVectors,1),indexGlobalPairs(:,2));
    else
        indexGlobalPairs = [];
        newFeatureIdx = 1:size(matchedFeatureVectors,1);
    end
    
    %For all previously seen features, update the feature struct to account
    %for new observation
    if ~isempty(indexGlobalPairs) > 0
        for f_i = 1:length(indexGlobalPairs(:,1))
            struct_i = indexGlobalPairs(f_i,1);
            obs_i = indexGlobalPairs(f_i,2);
            seenFeatureStructs{struct_i}.leftPixels(:, end+1) = matchedPointsLeft.Location(obs_i, :)';
            seenFeatureStructs{struct_i}.rightPixels(:, end+1) = matchedPointsRight.Location(obs_i, :)';
            seenFeatureStructs{struct_i}.imageIndex(end+1) = i;
        end
    end

    %For all new features,add a new struct
    fCount = length(seenFeatureStructs) + 1;
    if size(newFeatureIdx, 1) > 0
        for obs_i = newFeatureIdx
            seenFeatureStructs{fCount}.leftPixels = matchedPointsLeft.Location(obs_i, :)';
            seenFeatureStructs{fCount}.rightPixels = matchedPointsRight.Location(obs_i, :)';
            seenFeatureStructs{fCount}.imageIndex = i;
            fCount = fCount + 1;
            seenFeatureVectors(end+1, :) = matchedFeatureVectors(obs_i, :);
        end
    end
end

%% Prune points with low observation count

pruneIds = [];
for p_i = 1:length(seenFeatureStructs)
    if length(seenFeatureStructs{p_i}.imageIndex) < minObsNum
        pruneIds(end+1) = p_i;
    end
end
fprintf('%d points with less than %d observations. \n',  minObsNum,length(pruneIds));
fprintf('%d remaining points. \n', length(seenFeatureStructs) - length(pruneIds));

seenFeatureStructsPruned = removeCells(seenFeatureStructs, pruneIds);

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
fileName = [char(f(1)) '.mat'];

size(w_vk_vk_i)
size(v_vk_vk_i)
size(y_k_j)

p_vi_i = NaN(3, size(T_wIMU_GT,3));
for j = 1:size(T_wIMU_GT,3)
p_vi_i(:,j) = T_wIMU_GT(1:3, 4, j);
end

t = leftImageData.timestamps;
save(fileName, 'p_vi_i','w_vk_vk_i','v_vk_vk_i', 'cu','cv','fu','fv','b', 'y_k_j', 'C_c_v', 'rho_v_c_v', 't');

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
