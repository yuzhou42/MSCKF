function [q] = quat_from_rotmat(R)
% QUAT_FROM_ROTMAT Quaternion from rotation matrix.
%
%   [q] = QUAT_FROM_ROTMAT(R) produces a unit quaternion from the rotation 
%   matrix 'R'.
%
%   Inputs:
%   -------
%    R  - 3x3 orthonormal rotation matrix.
%
%   Outputs:
%   --------
%    q  - 4x1 quaternion with unit norm.

q = zeros(4, 1);
q(1) = 0.5*sqrt(R(1, 1) + R(2, 2) + R(3, 3) + 1);

if q(1) > 1e-10
  q(2) = (R(3, 2) - R(2, 3))/(4*q(1));
  q(3) = (R(1, 3) - R(3, 1))/(4*q(1));
  q(4) = (R(2, 1) - R(1, 2))/(4*q(1));
  return;
end

% More complicated...
if R(1, 1) > R(2, 2) && R(1, 1) > R(3, 3)
  d = 2*sqrt(1 + R(1, 1) - R(2, 2) - R(3, 3));
  q(1) = (R(3, 2) - R(2, 3))/d;
  q(2) = d/4;
  q(3) = (R(2, 1) + R(1, 2) )/d;
  q(4) = (R(1, 3) + R(3, 1) )/d;
elseif R(2, 2) > R(3, 3)
  d = 2*sqrt(1 + R(2, 2) - R(1, 1) - R(3, 3));
  q(1) = (R(1, 3) - R(3, 1) )/d;
  q(2) = (R(2, 1) + R(1, 2) )/d;
  q(3) = d/4;
  q(4) = (R(3, 2) + R(2, 3))/d;
else
  d = 2*sqrt(1 + R(3, 3) - R(1, 1) - R(2, 2));
  q(1) = (R(2, 1) - R(1, 2) )/d;
  q(2) = (R(1, 3) + R(3, 1) )/d;
  q(3) = (R(3, 2) + R(2, 3) )/d;
  q(4) = d/4;
end
