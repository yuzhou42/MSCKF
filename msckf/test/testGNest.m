%% Test feature Gauss Newton triangulation
%Generate a forward trajectory
addpath('../utils');
addpath('../');
clc
i = 1;
T_wCam_GT = [];
for t = 0:0.2:2
    T_wCam_GT(:,:,i) = [eye(3) [t 0 2]'; 0 0 0 1];
    i=i+1;
end
%Generate the true location
landmarks_w = [1 0 3]';
simSetup.pixelNoiseStd = 5; %pixels
simSetup.cameraResolution = [640, 480]; %pixels

%Set the camera intrinsics
focalLength = 600; 
c_u = 640;
c_v = 480;

K  = [focalLength 0 c_u;
    0 focalLength c_v;
    0 0 1];


imageMeasurements = genFeatureMeasurements(T_wCam_GT, landmarks_w, K, simSetup);
validMeasurements = (imageMeasurements(1,:) ~= -1);


observations = imageMeasurements(:, validMeasurements);
T_wCam_GT = T_wCam_GT(:,:, validMeasurements);

observations(1,:) = (observations(1,:) - c_u)/focalLength;
observations(2,:) = (observations(2,:) - c_v)/focalLength; 

camStates = {};
% Extract all viewable measurements
for step_i = 1:size(T_wCam_GT,3)
    camStates{step_i}.q_CG  = rotMatToQuat(T_wCam_GT(1:3,1:3, step_i)');
    camStates{step_i}.p_C_G = T_wCam_GT(1:3,4, step_i);
end
noiseParams.z_1 = 1;
noiseParams.z_2 = 1;

[p_f_G] = calcGNPosEst(camStates, observations, noiseParams)
