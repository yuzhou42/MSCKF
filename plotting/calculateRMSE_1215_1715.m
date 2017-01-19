msckf5_inf = load('msckf_1215_1715_min5_maxInf.mat');
msckf10_20 = load('msckf_1215_1715_min10_max20.mat');
msckf10_50 = load('msckf_1215_1715_min10_max50.mat');
msckf20_100 = load('msckf_1215_1715_min20_max100.mat');
swf10 = load('swf_1215_1715_10_dataset3.mat');
swf50 = load('swf_1215_1715_50_dataset3.mat');
swf100 = load('swf_1215_1715_100_dataset3.mat');
imu = load('imu_1215_1715.mat');

%% Calculate Average RMSE (Root-Mean-Squared Error)

% IMU Only RMSE
imu_trans = sqrt(sum(imu.msckf_trans_err.^2, 1)/3);
imu_rot = sqrt(sum(imu.msckf_rot_err.^2, 1)/3);

% MSCKF RMSE
msckf5_inf_trans = sqrt(sum(msckf5_inf.msckf_trans_err.^2, 1)/3);
msckf10_20_trans = sqrt(sum(msckf10_20.msckf_trans_err.^2, 1)/3);
msckf10_50_trans = sqrt(sum(msckf10_50.msckf_trans_err.^2, 1)/3);
msckf20_100_trans = sqrt(sum(msckf20_100.msckf_trans_err.^2, 1)/3);
disp('===============')
msckf5_inf_rot = sqrt(sum(msckf5_inf.msckf_rot_err.^2, 1)/3);
msckf10_20_rot = sqrt(sum(msckf10_20.msckf_rot_err.^2, 1)/3);
msckf10_50_rot = sqrt(sum(msckf10_50.msckf_rot_err.^2, 1)/3);
msckf20_100_rot = sqrt(sum(msckf20_100.msckf_rot_err.^2, 1)/3);

% SWF RMSE
swf10_trans = sqrt(sum(swf10.swf_trans_err.^2, 1)/3);
swf50_trans = sqrt(sum(swf50.swf_trans_err.^2, 1)/3);
swf100_trans = sqrt(sum(swf100.swf_trans_err.^2, 1)/3);

swf10_rot = sqrt(sum(swf10.swf_rot_err.^2, 1)/3);
swf50_rot = sqrt(sum(swf50.swf_rot_err.^2, 1)/3);
swf100_rot = sqrt(sum(swf100.swf_rot_err.^2, 1)/3);