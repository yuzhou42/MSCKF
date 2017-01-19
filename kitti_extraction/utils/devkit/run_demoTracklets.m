function run_demoTracklets(base_dir,calib_dir)
% KITTI RAW DATA DEVELOPMENT KIT
% 
% This tool displays the images and the object labels for the benchmark and
% provides an entry point for writing your own interface to the data set.
% Before running this tool, set root_dir to the directory where you have
% downloaded the dataset. 'root_dir' must contain the subdirectory
% 'training', which in turn contains 'image_2', 'label_2' and 'calib'.
% For more information about the data format, please look into readme.txt.
%
% Input arguments:
% base_dir .... absolute path to sequence base directory (ends with _sync)
% calib_dir ... absolute path to directory that contains calibration files
%
% Usage:
%   SPACE: next frame
%   '-':   last frame
%   'x':   +10 frames
%   'y':   -10 frames
%   'q':   quit
%
% Occlusion Coding:
%   green:  not occluded
%   yellow: partly occluded
%   red:    fully occluded
%   white:  unknown

% clear and close everything
clear all; close all; dbstop error; clc;
disp('======= KITTI DevKit Demo =======');

% options (modify this to select your sequence)
% the base_dir must contain:
%   - the data directories (image_00, image_01, ..)
%   - the tracklet file (tracklet_labels.xml)
% the calib directory must contain:
%   - calib_cam_to_cam.txt
%   - calib_velo_to_cam.txt
% cameras:
%   - 0 = left grayscale
%   - 1 = right grayscale
%   - 2 = left color
%   - 3 = right color
if nargin<1
  base_dir  = '/mnt/karlsruhe_dataset/2011_09_26/2011_09_26_drive_0009_sync';
end
if nargin<2
  calib_dir = '/mnt/karlsruhe_dataset/2011_09_26';
end
cam       = 2; % 0-based index

% get image sub-directory
image_dir = fullfile(base_dir, sprintf('/image_%02d/data', cam));

% get number of images for this dataset
nimages = length(dir(fullfile(image_dir, '*.png')));

% set up figure
gh = visualization('init',image_dir);

% read calibration for the day
[veloToCam, K] = loadCalibration(calib_dir);

% read tracklets for the selected sequence
tracklets = readTrackletsMex([base_dir '/tracklet_labels.xml']);

% extract tracklets
% LOCAL OBJECT COORDINATE SYSTEM:
%   x -> facing right
%   y -> facing forward
%   z -> facing up
for it = 1:numel(tracklets)
  
  % shortcut for tracklet dimensions
  w = tracklets{it}.w;
  h = tracklets{it}.h;
  l = tracklets{it}.l;

  % set bounding box corners
  corners(it).x = [l/2, l/2, -l/2, -l/2, l/2, l/2, -l/2, -l/2]; % front/back
  corners(it).y = [w/2, -w/2, -w/2, w/2, w/2, -w/2, -w/2, w/2]; % left/right
  corners(it).z = [0,0,0,0,h,h,h,h];
  
  % get translation and orientation
  t{it} = [tracklets{it}.poses(1,:); tracklets{it}.poses(2,:); tracklets{it}.poses(3,:)];
  rz{it} = wrapToPi(tracklets{it}.poses(6,:));
  occlusion{it} = tracklets{it}.poses(8,:);
end

% 3D bounding box faces (indices for corners)
face_idx = [ 1,2,6,5   % front face
             2,3,7,6   % left face
             3,4,8,7   % back face
             4,1,5,8]; % right face

% main loop (start at first image of sequence)
img_idx = 0;
while 1
  
  % visualization update for next frame
  visualization('update',image_dir,gh,img_idx,nimages);
  
  % compute bounding boxes for visible tracklets
  for it = 1:numel(tracklets)
    
    % get relative tracklet frame index (starting at 0 with first appearance; 
    % xml data stores poses relative to the first frame where the tracklet appeared)
    pose_idx = img_idx-tracklets{it}.first_frame+1; % 0-based => 1-based MATLAB index

    % only draw tracklets that are visible in current frame
    if pose_idx<1 || pose_idx>(size(tracklets{it}.poses,2))
      continue;
    end

    % compute 3d object rotation in velodyne coordinates
    % VELODYNE COORDINATE SYSTEM:
    %   x -> facing forward
    %   y -> facing left
    %   z -> facing up
    R = [cos(rz{it}(pose_idx)), -sin(rz{it}(pose_idx)), 0;
         sin(rz{it}(pose_idx)),  cos(rz{it}(pose_idx)), 0;
                             0,                      0, 1];

    % rotate and translate 3D bounding box in velodyne coordinate system
    corners_3D      = R*[corners(it).x;corners(it).y;corners(it).z];
    corners_3D(1,:) = corners_3D(1,:) + t{it}(1,pose_idx);
    corners_3D(2,:) = corners_3D(2,:) + t{it}(2,pose_idx);
    corners_3D(3,:) = corners_3D(3,:) + t{it}(3,pose_idx);
    corners_3D      = (veloToCam{cam+1}*[corners_3D; ones(1,size(corners_3D,2))]);
    
    % generate an orientation vector and compute coordinates in velodyneCS
    orientation_3D      = R*[0.0, 0.7*l; 0.0, 0.0; 0.0, 0.0];
    orientation_3D(1,:) = orientation_3D(1,:) + t{it}(1, pose_idx);
    orientation_3D(2,:) = orientation_3D(2,:) + t{it}(2, pose_idx);
    orientation_3D(3,:) = orientation_3D(3,:) + t{it}(3, pose_idx);
    orientation_3D      = (veloToCam{cam+1}*[orientation_3D; ones(1,size(orientation_3D,2))]);
    
    % only draw 3D bounding box for objects in front of the image plane
    if any(corners_3D(3,:)<0.5) || any(orientation_3D(3,:)<0.5) 
      continue;
    end

    % project the 3D bounding box into the image plane
    corners_2D     = projectToImage(corners_3D, K);
    orientation_2D = projectToImage(orientation_3D, K);
    drawBox3D(gh,occlusion{it}(pose_idx),corners_2D,face_idx,orientation_2D)
    
    % compute and draw the 2D bounding box from the 3D box projection
    box.x1 = min(corners_2D(1,:));
    box.x2 = max(corners_2D(1,:));
    box.y1 = min(corners_2D(2,:));
    box.y2 = max(corners_2D(2,:));
    drawBox2D(gh,box,occlusion{it}(pose_idx),tracklets{it}.objectType)
  end

  % force drawing and tiny user interface
  waitforbuttonpress; 
  key = get(gcf,'CurrentCharacter');
  switch lower(key)                         
    case 'q',  break;                                 % quit
    case '-',  img_idx = max(img_idx-1,  0);          % previous frame
    case 'x',  img_idx = min(img_idx+100,nimages-1);  % +100 frames
    case 'y',  img_idx = max(img_idx-100,0);          % -100 frames
    otherwise, img_idx = min(img_idx+1,  nimages-1);  % next frame
  end

end

% clean up
close all;
