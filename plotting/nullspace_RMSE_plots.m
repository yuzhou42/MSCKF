%% Load data
clear all; close all; clc;

nst_on = load('msckf_NST_on.mat');
nst_off = load('msckf_NST_off.mat');

kStart = 500; kEnd = 1000;

%% Compute RMSE
nst_off_trans_rmse = sqrt(mean(nst_off.msckf_trans_err.^2,1));
nst_off_rot_rmse = sqrt(mean(nst_off.msckf_rot_err.^2,1));

nst_on_trans_rmse = sqrt(mean(nst_on.msckf_trans_err.^2,1));
nst_on_rot_rmse = sqrt(mean(nst_on.msckf_rot_err.^2,1));

%% Plot stuff
figure(1); clf;
fontSize = 14;
lineWidth = 2;
pos = [200,200,640,400];
xLim = [kStart, kEnd];
k = kStart:kEnd;
xticks = linspace(kStart,kEnd,11);

transLim = [0 0.6];
rotLim = [0 0.25];

set(gcf,'Position',pos);

% Translational RMSE
subplot(2,1,1);
plot(k,nst_off_trans_rmse, '-r','LineWidth',lineWidth); hold on;
plot(k,nst_on_trans_rmse, '-b','LineWidth',lineWidth);
xlim(xLim);
set(gca,'XTick',xticks);
% ylim(transLim)
legend('Noise Correlated to State', 'Noise De-correlated from State', ...
    'Location','northwest');
ylabel('Trans. RMSE (m)')
title('Effect of Nullspace Projection');
set(gca,'FontSize',fontSize);
grid minor;
box on;

% Rotational RMSE
subplot(2,1,2);
plot(k,nst_off_rot_rmse,'-r','LineWidth',lineWidth); hold on;
plot(k,nst_on_rot_rmse,'-b','LineWidth',lineWidth);
xlim(xLim);
set(gca,'XTick',xticks);
% ylim(rotLim)
legend('Noise Correlated to State', 'Noise De-correlated from State', ...
    'Location','northwest');
ylabel('Rot. RMSE (Axis-Angle)')
xlabel('Timestep')
set(gca,'FontSize',fontSize)
grid minor;
box on;

%% Export figure
export_fig(gcf, 'NST_RMSE.pdf', '-transparent');