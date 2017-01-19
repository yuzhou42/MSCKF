%% Load data
clear; close all; clc;

% 500-1000
% swf10 = load('swf_500_1000_10_dataset3.mat');
% swf50 = load('swf_500_1000_50_dataset3.mat');
% swf100 = load('swf_500_1000_100_dataset3.mat');
% msckf5 = load('msckf_500_1000_min5_maxInf');
% msckf10 = load('msckf_500_1000_min10_max50');
% msckf20 = load('msckf_500_1000_min20_max100');
% imu = load('imu_500_1000.mat');
% kStart = 500; kEnd = 1000;

% 1215-1715
swf10 = load('swf_1215_1715_10_dataset3.mat');
swf50 = load('swf_1215_1715_50_dataset3.mat');
swf100 = load('swf_1215_1715_100_dataset3.mat');
msckf5 = load('msckf_1215_1715_min5_maxInf');
msckf10 = load('msckf_1215_1715_min10_max50');
msckf20 = load('msckf_1215_1715_min20_max100');
imu = load('imu_1215_1715.mat');
kStart = 1215; kEnd = 1715;

%% Compute RMSE
swf10_trans_rmse = sqrt(mean(swf10.swf_trans_err.^2,1));
swf10_rot_rmse = sqrt(mean(swf10.swf_rot_err.^2,1));

swf50_trans_rmse = sqrt(mean(swf50.swf_trans_err.^2,1));
swf50_rot_rmse = sqrt(mean(swf50.swf_rot_err.^2,1));

swf100_trans_rmse = sqrt(mean(swf100.swf_trans_err.^2,1));
swf100_rot_rmse = sqrt(mean(swf100.swf_rot_err.^2,1));

msckf5_trans_rmse = sqrt(mean(msckf5.msckf_trans_err.^2,1));
msckf5_rot_rmse = sqrt(mean(msckf5.msckf_rot_err.^2,1));

msckf10_trans_rmse = sqrt(mean(msckf10.msckf_trans_err.^2,1));
msckf10_rot_rmse = sqrt(mean(msckf10.msckf_rot_err.^2,1));

msckf20_trans_rmse = sqrt(mean(msckf20.msckf_trans_err.^2,1));
msckf20_rot_rmse = sqrt(mean(msckf20.msckf_rot_err.^2,1));

imu_trans_rmse = sqrt(mean(imu.msckf_trans_err.^2,1));
imu_rot_rmse = sqrt(mean(imu.msckf_rot_err.^2,1));

%% Plot stuff
figure(1); clf;
fontSize = 14;
lineWidth = 2;
pos = [200,200,640,400];
xLim = [kStart,kEnd];
k = kStart:kEnd;
xticks = linspace(kStart,kEnd,11);

set(gcf, 'Position', pos);

% Translational RMSE
subplot(2,1,1);
plot(k,imu_trans_rmse, '-k', 'LineWidth', lineWidth); hold on;
plot(k,msckf5_trans_rmse, '-b', 'LineWidth', lineWidth);
plot(k,msckf10_trans_rmse, '--b', 'LineWidth', lineWidth);
plot(k,msckf20_trans_rmse, '-.b', 'LineWidth', lineWidth);
plot(k,swf10_trans_rmse, '-g', 'LineWidth', lineWidth);
plot(k,swf50_trans_rmse, '--g', 'LineWidth', lineWidth);
plot(k,swf100_trans_rmse, '-.g', 'LineWidth', lineWidth);
xlim(xLim);
set(gca,'XTick',xticks);
legend('IMU Only','MSCKF 5-Inf', 'MSCKF 10-50', 'MSCKF 20-100', 'SWF 10', 'SWF 50', 'SWF 100','Location', 'northwest');
title('Window Size Comparison');
ylabel('Trans. RMSE (m)')
set(gca,'FontSize',fontSize)
grid minor; box on;

% Rotational RMSE
subplot(2,1,2);
plot(k,imu_rot_rmse, '-k', 'LineWidth', lineWidth); hold on;
plot(k,msckf5_rot_rmse, '-b', 'LineWidth', lineWidth);
plot(k,msckf10_rot_rmse, '--b', 'LineWidth', lineWidth);
plot(k,msckf20_rot_rmse, '-.b', 'LineWidth', lineWidth);
plot(k,swf10_rot_rmse, '-g', 'LineWidth', lineWidth);
plot(k,swf50_rot_rmse, '--g', 'LineWidth', lineWidth);
plot(k,swf100_rot_rmse, '-.g', 'LineWidth', lineWidth);
xlim(xLim);
set(gca,'XTick',xticks);
legend('IMU Only','MSCKF 5-Inf', 'MSCKF 10-50', 'MSCKF 20-100', 'SWF 10', 'SWF 50', 'SWF 100','Location', 'northwest');
ylabel('Rot. RMSE (Axis-Angle)');
xlabel('Timestep');
set(gca,'FontSize',fontSize)
grid minor; box on;

%% Export figure
fileName = sprintf('RMSE-Comparison-WindowSize-%d-%d.pdf',kStart,kEnd);
export_fig(gcf, fileName, '-transparent');