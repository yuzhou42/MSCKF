clear all; close all; clc;
nst_on = load('msckf_NST_on.mat');
nst_off = load('msckf_NST_off.mat');

plotAbsErr = true;

msckf_trans_err = nst_off.msckf_trans_err;
msckf_rot_err = nst_off.msckf_rot_err;
msckf_err_sigma = nst_off.err_sigma;
swf_trans_err = nst_on.msckf_trans_err;
swf_rot_err = nst_on.msckf_rot_err;
swf_err_sigma = nst_on.err_sigma;
tPlot = nst_on.tPlot;

transLim = [-0.4 0.4];
rotLim = [-0.2 0.2];
fontSize = 14;

if plotAbsErr
    msckf_trans_err = abs(msckf_trans_err);
    msckf_rot_err = abs(msckf_rot_err);
    swf_trans_err = abs(swf_trans_err);
    swf_rot_err = abs(swf_rot_err);
    transLim = transLim - transLim(1);
    rotLim = rotLim - rotLim(1);
end

figure
subplot(3,1,1)
hold on
plot(tPlot, msckf_trans_err(1,:), '-r', 'LineWidth', 1.2)
% plot(tPlot, -3*msckf_err_sigma(4,:), '--r', 'LineWidth', 1.2)
% plot(tPlot, 3*msckf_err_sigma(4,:), '--r', 'LineWidth', 1.2)
plot(tPlot, swf_trans_err(1,:), '-b', 'LineWidth', 1.2)
% plot(tPlot, -3*swf_err_sigma(4,:), '--b', 'LineWidth', 1.2)
% plot(tPlot, 3*swf_err_sigma(4,:), '--b', 'LineWidth', 1.2)

xlim([tPlot(1) tPlot(end)]);
ylim(transLim)
legend('Noise Correlated to State', 'Noise De-correlated from State', 'Location','northwest');
title(sprintf('Absolute Translational Error'))
ylabel('\delta r_x [m]')
set(gca,'FontSize',fontSize)
grid on; grid minor;
box on;

subplot(3,1,2)
hold on
plot(tPlot, msckf_trans_err(2,:), '-r', 'LineWidth', 1.2)
% plot(tPlot, -3*msckf_err_sigma(5,:), '--r', 'LineWidth', 1.2)
% plot(tPlot, 3*msckf_err_sigma(5,:), '--r', 'LineWidth', 1.2)
plot(tPlot, swf_trans_err(2,:), '-b', 'LineWidth', 1.2)
% plot(tPlot, -3*swf_err_sigma(5,:), '--b', 'LineWidth', 1.2)
% plot(tPlot, 3*swf_err_sigma(5,:), '--b', 'LineWidth', 1.2)
xlim([tPlot(1) tPlot(end)]);
ylim(transLim)
ylabel('\delta r_y [m]')
set(gca,'FontSize',fontSize)
grid on; grid minor;
box on;

subplot(3,1,3)
hold on
plot(tPlot, msckf_trans_err(3,:), '-r', 'LineWidth', 1.2)
% plot(tPlot, -3*msckf_err_sigma(6,:), '--r', 'LineWidth', 1.2)
% plot(tPlot, 3*msckf_err_sigma(6,:), '--r', 'LineWidth', 1.2)
plot(tPlot, swf_trans_err(3,:), '-b', 'LineWidth', 1.2)
% plot(tPlot, -3*swf_err_sigma(6,:), '--b', 'LineWidth', 1.2)
% plot(tPlot, 3*swf_err_sigma(6,:), '--b', 'LineWidth', 1.2)
xlim([tPlot(1) tPlot(end)]);
ylim(transLim)
ylabel('\delta r_z [m]')
xlabel('t_k [s]')
%set(gca,'FontSize',12)
%set(findall(gcf,'type','text'),'FontSize',12)
set(gca,'FontSize',fontSize)
grid on; grid minor;
box on;
filename = sprintf('NST_trans.pdf');
export_fig(gcf, filename, '-transparent');

figure
subplot(3,1,1)
hold on
plot(tPlot, msckf_rot_err(1,:), '-r', 'LineWidth', 1.2)
% plot(tPlot, -3*msckf_err_sigma(1,:), '--r', 'LineWidth', 1.2)
% plot(tPlot, 3*msckf_err_sigma(1,:), '--r', 'LineWidth', 1.2)
plot(tPlot, swf_rot_err(1,:), '-b', 'LineWidth', 1.2)
% plot(tPlot, -3*swf_err_sigma(1,:), '--b', 'LineWidth', 1.2)
% plot(tPlot, 3*swf_err_sigma(1,:), '--b', 'LineWidth', 1.2)
xlim([tPlot(1) tPlot(end)]);
ylim(rotLim)
title(sprintf('Absolute Rotational Error'))
ylabel('\delta\theta_x')
legend('Noise Correlated to State', 'Noise De-correlated from State', 'Location','northwest');
set(gca,'FontSize',fontSize)
grid on; grid minor;
box on;
 
subplot(3,1,2)
hold on
plot(tPlot, msckf_rot_err(2,:), '-r', 'LineWidth', 1.2)
% plot(tPlot, -3*msckf_err_sigma(2,:), '--r', 'LineWidth', 1.2)
% plot(tPlot, 3*msckf_err_sigma(2,:), '--r', 'LineWidth', 1.2)
plot(tPlot, swf_rot_err(2,:), '-b', 'LineWidth', 1.2)
% plot(tPlot, -3*swf_err_sigma(2,:), '--b', 'LineWidth', 1.2)
% plot(tPlot, 3*swf_err_sigma(2,:), '--b', 'LineWidth', 1.2)
xlim([tPlot(1) tPlot(end)]);
ylim(rotLim)
ylabel('\delta\theta_y')
set(gca,'FontSize',fontSize)
grid on; grid minor;
box on;

subplot(3,1,3)
hold on
plot(tPlot, msckf_rot_err(3,:), '-r', 'LineWidth', 1.2)
% plot(tPlot, -3*msckf_err_sigma(3,:), '--r', 'LineWidth', 1.2)
% plot(tPlot, 3*msckf_err_sigma(3,:), '--r', 'LineWidth', 1.2)
plot(tPlot, swf_rot_err(3,:), '-b', 'LineWidth', 1.2)
% plot(tPlot, -3*swf_err_sigma(3,:), '--b', 'LineWidth', 1.2)
% plot(tPlot, 3*swf_err_sigma(3,:), '--b', 'LineWidth', 1.2)
xlim([tPlot(1) tPlot(end)]);
ylim(rotLim)
ylabel('\delta\theta_z')
xlabel('t_k [s]')
set(gca,'FontSize',fontSize)
grid on; grid minor;
box on;

filename = sprintf('NST_rot.pdf');
export_fig(gcf, filename, '-transparent');