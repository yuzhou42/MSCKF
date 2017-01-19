function R = getRnb( oxts )
% get rotation matrix of the body frame w.r.t navigation frame
rx = oxts(4); % roll
ry = oxts(5); % pitch
rz = oxts(6); % heading
Rx = [1 0 0; 0 cos(rx) -sin(rx); 0 sin(rx) cos(rx)]; % body frame => navigation frame
Ry = [cos(ry) 0 sin(ry); 0 1 0; -sin(ry) 0 cos(ry)]; % body frame => navigation frame
Rz = [cos(rz) -sin(rz) 0; sin(rz) cos(rz) 0; 0 0 1]; % body frame => navigation frame
R  = Rz*Ry*Rx;
end

