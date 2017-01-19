function run_demoVehiclePath (base_dir)
% KITTI RAW DATA DEVELOPMENT KIT
% 
% Plots OXTS poses of a sequence
%
% Input arguments:
% base_dir .... absolute path to sequence base directory (ends with _sync)

% clear and close everything
clear all; close all; dbstop error; clc;
disp('======= KITTI DevKit Demo =======');

% sequence base directory
if nargin<1
  base_dir = '/Users/valentinp/Research/Datasets/Kitti/2011_09_29/2011_09_29_drive_0071_sync';
end

% load oxts data
oxts = loadOxtsliteData(base_dir);

% transform to poses
pose = convertOxtsToPose(oxts);

% plot every 10'th pose
figure; hold on; axis equal;
l = 3; % coordinate axis length
A = [0 0 0 1; l 0 0 1; 0 0 0 1; 0 l 0 1; 0 0 0 1; 0 0 l 1]';
for i=1:10:length(pose)
  B = pose{i}*A;
  plot3(B(1,1:2),B(2,1:2),B(3,1:2),'-r','LineWidth',2); % x: red
  plot3(B(1,3:4),B(2,3:4),B(3,3:4),'-g','LineWidth',2); % y: green
  plot3(B(1,5:6),B(2,5:6),B(3,5:6),'-b','LineWidth',2); % z: blue
end
xlabel('x');
ylabel('y');
zlabel('z');
