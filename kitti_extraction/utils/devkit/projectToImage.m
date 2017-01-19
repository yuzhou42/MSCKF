function pts_2D = projectToImage(pts_3D, K)
% PROJECTTOIMAGE projects 3D points in given coordinate system in the image
% plane using the given calibration matrix K.

% project in image
pts_2D = K * pts_3D(1:3,:);

% scale projected points
pts_2D(1,:) = pts_2D(1,:)./pts_2D(3,:);
pts_2D(2,:) = pts_2D(2,:)./pts_2D(3,:);
pts_2D(3,:) = [];
