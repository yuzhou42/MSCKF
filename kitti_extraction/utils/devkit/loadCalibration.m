function [veloToCam, P_rect] = loadCalibration(dir)
% LOADCALIBRATION provides all needed coordinate system transformations
% returns the pre-computed velodyne to cam (gray and color) projection

% get the velodyne to camera calibration
Tr_velo_to_cam = loadCalibrationRigid(fullfile(dir,'calib_velo_to_cam.txt'));

% get the camera intrinsic and extrinsic calibration
calib = loadCalibrationCamToCam(fullfile(dir,'calib_cam_to_cam.txt'));

% create 4x4 matrix from rectifying rotation matrix 
R_rect00 = calib.R_rect{1};
R_rect00(4,4) = 1;

% compute extrinsics from first to i'th rectified camera
T0 = eye(4); T0(1,4) = calib.P_rect{1}(1,4)/calib.P_rect{1}(1,1);
T1 = eye(4); T1(1,4) = calib.P_rect{2}(1,4)/calib.P_rect{2}(1,1);
T2 = eye(4); T2(1,4) = calib.P_rect{3}(1,4)/calib.P_rect{3}(1,1);
T3 = eye(4); T3(1,4) = calib.P_rect{4}(1,4)/calib.P_rect{4}(1,1);

% transformation: velodyne -> rectified camera coordinates
veloToCam{1} = T0 * R_rect00 * Tr_velo_to_cam;
veloToCam{2} = T1 * R_rect00 * Tr_velo_to_cam;
veloToCam{3} = T2 * R_rect00 * Tr_velo_to_cam;
veloToCam{4} = T3 * R_rect00 * Tr_velo_to_cam;

% Rectification matrix
P_rect =calib.P_rect{2};
