%% Load data
clear; close all; clc;

msckf40 = load('msckf_1215_1715_min20_max100_40lessnoisy.mat');
msckf60 = load('msckf_1215_1715_min20_max100_60lessnoisy.mat');
msckf100 = load('msckf_1215_1715_min20_max100_100lessnoisy.mat');

swf40 = load('swf_1215_1715_25_40lessnoisy.mat');
swf60 = load('swf_1215_1715_25_60lessnoisy.mat');
swf100 = load('swf_1215_1715_25_100lessnoisy.mat');

imu = load('imu_1215_1715.mat');

kStart = 1215; kEnd = 1715;

%% Compute RMSE
msckf40_trans_rmse = sqrt(mean(msckf40.msckf_trans_err.^2,1));
msckf40_rot_rmse = sqrt(mean(msckf40.msckf_rot_err.^2,1));

msckf60_trans_rmse = sqrt(mean(msckf60.msckf_trans_err.^2,1));
msckf60_rot_rmse = sqrt(mean(msckf60.msckf_rot_err.^2,1));

msckf100_trans_rmse = sqrt(mean(msckf100.msckf_trans_err.^2,1));
msckf100_rot_rmse = sqrt(mean(msckf100.msckf_rot_err.^2,1));

swf40_trans_rmse = sqrt(mean(swf40.swf_trans_err.^2,1));
swf40_rot_rmse = sqrt(mean(swf40.swf_rot_err.^2,1));

swf60_trans_rmse = sqrt(mean(swf60.swf_trans_err.^2,1));
swf60_rot_rmse = sqrt(mean(swf60.swf_rot_err.^2,1));

swf100_trans_rmse = sqrt(mean(swf100.swf_trans_err.^2,1));
swf100_rot_rmse = sqrt(mean(swf100.swf_rot_err.^2,1));

imu_trans_rmse = sqrt(mean(imu.msckf_trans_err.^2,1));
imu_rot_rmse = sqrt(mean(imu.msckf_rot_err.^2,1));

%% Plot Stuff
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
plot(k,msckf40_trans_rmse, '-b', 'LineWidth', lineWidth);
plot(k,msckf60_trans_rmse, '--b', 'LineWidth', lineWidth);
plot(k,msckf100_trans_rmse, '-.b', 'LineWidth', lineWidth);
plot(k,swf40_trans_rmse, '-g', 'LineWidth', lineWidth);
plot(k,swf60_trans_rmse, '--g', 'LineWidth', lineWidth);
plot(k,swf100_trans_rmse, '-.g', 'LineWidth', lineWidth);
xlim(xLim);
set(gca,'XTick',xticks);
legend('IMU Only','MSCKF 40', 'MSCKF 60', 'MSCKF 100', 'SWF 40','SWF 60','SWF 100', 'Location', 'northwest');
title('Feature Density Comparison');
ylabel('Trans. RMSE (m)')
set(gca,'FontSize',fontSize)
grid minor; box on;

% Rotational RMSE
subplot(2,1,2);
plot(k,imu_rot_rmse, '-k', 'LineWidth', lineWidth); hold on;
plot(k,msckf40_rot_rmse, '-b', 'LineWidth', lineWidth);
plot(k,msckf60_rot_rmse, '--b', 'LineWidth', lineWidth);
plot(k,msckf100_rot_rmse, '-.b', 'LineWidth', lineWidth);
plot(k,swf40_rot_rmse, '-g', 'LineWidth', lineWidth);
plot(k,swf60_rot_rmse, '--g', 'LineWidth', lineWidth);
plot(k,swf100_rot_rmse, '-.g', 'LineWidth', lineWidth);
xlim(xLim);
set(gca,'XTick',xticks);
legend('IMU Only','MSCKF 40', 'MSCKF 60', 'MSCKF 100', 'SWF 40','SWF 60','SWF 100', 'Location', 'northwest');
ylabel('Rot. RMSE (Axis-Angle)');
xlabel('Timestep');
set(gca,'FontSize',fontSize)
grid minor; box on;

%% Export figure
export_fig(gcf, 'RMSE-Comparison-Feat-Density-1215-1715-Noisy.pdf', '-transparent');