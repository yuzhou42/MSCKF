function pose = convertOxtsToPose(oxts)
% converts a list of oxts measurements into metric poses,
% starting at (0,0,0) meters, OXTS coordinates are defined as
% x = forward, y = right, z = down (see OXTS RT3000 user manual)
% afterwards, pose{i} contains the transformation which takes a
% 3D point in the i'th frame and projects it into the oxts
% coordinates of the first frame.

% compute scale from first lat value
scale = latToScale(oxts{1}(1));

% init pose
pose     = [];
Tr_0_inv = [];

% for all oxts packets do
for i=1:length(oxts)
  
  % if there is no data => no pose
  if isempty(oxts{i})
    pose{i} = [];
    continue;
  end

  % translation vector
  [t(1,1) t(2,1)] = latlonToMercator(oxts{i}(1),oxts{i}(2),scale);
  t(3,1) = oxts{i}(3);

  % rotation matrix (OXTS RT3000 user manual, page 71/92)
  rx = oxts{i}(4); % roll
  ry = oxts{i}(5); % pitch
  rz = oxts{i}(6); % heading 
  Rx = [1 0 0; 0 cos(rx) -sin(rx); 0 sin(rx) cos(rx)]; % base => nav  (level oxts => rotated oxts)
  Ry = [cos(ry) 0 sin(ry); 0 1 0; -sin(ry) 0 cos(ry)]; % base => nav  (level oxts => rotated oxts)
  Rz = [cos(rz) -sin(rz) 0; sin(rz) cos(rz) 0; 0 0 1]; % base => nav  (level oxts => rotated oxts)
  R  = Rz*Ry*Rx;
  
  % normalize translation and rotation (start at 0/0/0)
  if isempty(Tr_0_inv)
    Tr_0_inv = inv([R t;0 0 0 1]);
  end
      
  % add pose
  pose{i} = Tr_0_inv*[R t;0 0 0 1];
end

