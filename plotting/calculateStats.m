swf40 = load('swf_1215_1715_25_40lessnoisy.mat');
swf60 = load('swf_1215_1715_25_60lessnoisy.mat');
swf100 = load('swf_1215_1715_25_100lessnoisy.mat');
msckf40 = load('msckf_1215_1715_min20_max100_40lessnoisy.mat');
msckf60 = load('msckf_1215_1715_min20_max100_60lessnoisy.mat');
msckf100 = load('msckf_1215_1715_min20_max100_100lessnoisy.mat');
imu = load('imu_1215_1715.mat');


%Calculate Average RMSE (Root-Mean-Squared Error)
clc
disp('IMU Only RMSE')
mean(sqrt(sum(imu.msckf_trans_err.^2, 1)/3))
mean(sqrt(sum(imu.msckf_rot_err.^2, 1)/3))

disp('MSCKF RMSE')
mean(sqrt(sum(msckf40.msckf_trans_err.^2, 1)/3))
mean(sqrt(sum(msckf60.msckf_trans_err.^2, 1)/3))
mean(sqrt(sum(msckf100.msckf_trans_err.^2, 1)/3))
disp('===============')
mean(sqrt(sum(msckf40.msckf_rot_err.^2, 1)/3))
mean(sqrt(sum(msckf60.msckf_rot_err.^2, 1)/3))
mean(sqrt(sum(msckf100.msckf_rot_err.^2, 1)/3))

disp('SWF RMSE')
mean(sqrt(sum(swf40.swf_trans_err.^2, 1)/3))
mean(sqrt(sum(swf60.swf_trans_err.^2, 1)/3))
mean(sqrt(sum(swf100.swf_trans_err.^2, 1)/3))
disp('===============')
mean(sqrt(sum(swf40.swf_rot_err.^2, 1)/3))
mean(sqrt(sum(swf60.swf_rot_err.^2, 1)/3))
mean(sqrt(sum(swf100.swf_rot_err.^2, 1)/3))

%Calculate Average NEES
stateErr = [imu.msckf_trans_err; imu.msckf_rot_err];
stateVar = imu.err_sigma.^2;
avgNEES = 0;
stepNum = size(stateErr, 2);
for i = 1:stepNum
    avgNEES = avgNEES + 1/stepNum*stateErr(:,i)'*inv(diag(stateVar(:,i)))*stateErr(:,i);
end
avgNEES

stateErr = [msckf40.msckf_trans_err; msckf40.msckf_rot_err];
stateVar = msckf40.err_sigma.^2;
avgNEES = 0;
stepNum = size(stateErr, 2);
for i = 1:stepNum
    avgNEES = avgNEES + 1/stepNum*stateErr(:,i)'*inv(diag(stateVar(:,i)))*stateErr(:,i);
end
avgNEES

stateErr = [msckf60.msckf_trans_err; msckf60.msckf_rot_err];
stateVar = msckf60.err_sigma.^2;
avgNEES = 0;
stepNum = size(stateErr, 2);
for i = 1:stepNum
    avgNEES = avgNEES + 1/stepNum*stateErr(:,i)'*inv(diag(stateVar(:,i)))*stateErr(:,i);
end
avgNEES

stateErr = [msckf100.msckf_trans_err; msckf100.msckf_rot_err];
stateVar = msckf100.err_sigma.^2;
avgNEES = 0;
stepNum = size(stateErr, 2);
for i = 1:stepNum
    avgNEES = avgNEES + 1/stepNum*stateErr(:,i)'*inv(diag(stateVar(:,i)))*stateErr(:,i);
end
avgNEES


stateErr = [swf40.swf_trans_err; swf40.swf_rot_err];
stateVar = swf40.stateSigmaHistMat.^2;
stepNum = size(stateErr, 2);

avgNEES = NaN(stepNum, 1);
for i = 1:stepNum
    avgNEES(i) = stateErr(:,i)'*inv(diag(stateVar(:,i)))*stateErr(:,i);
end

mean(avgNEES)


stateErr = [swf60.swf_trans_err; swf60.swf_rot_err];
stateVar = swf60.stateSigmaHistMat.^2;
avgNEES = 0;
stepNum = size(stateErr, 2);
for i = 1:stepNum
    avgNEES = avgNEES + 1/stepNum*stateErr(:,i)'*inv(diag(stateVar(:,i)))*stateErr(:,i);
end
avgNEES

stateErr = [swf100.swf_trans_err; swf100.swf_rot_err];
stateVar = swf100.stateSigmaHistMat.^2;
avgNEES = 0;
stepNum = size(stateErr, 2);
for i = 1:stepNum
    avgNEES = avgNEES + 1/stepNum*stateErr(:,i)'*inv(diag(stateVar(:,i)))*stateErr(:,i);
end
avgNEES