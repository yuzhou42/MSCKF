function [poseMat] = getGroundTruth (base_dir, imuFrames)
% Based on KITTI RAW DATA DEVELOPMENT KIT
% 
% Outputs ground truth poses
%
% Input arguments:
% base_dir .... absolute path to sequence base directory (ends with _sync)
% sequence base directory

% load oxts data
oxts = loadOxtsliteData(base_dir);

% transform to poses
pose = convertOxtsToPose(oxts);

poseMat = zeros(4,4);

for i = 1:length(imuFrames)
    if i == 1
        firstPose = pose{imuFrames(i)};
    end
    poseMat(:,:,i) = inv(firstPose)*pose{imuFrames(i)};
end