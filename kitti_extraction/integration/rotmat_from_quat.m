function [R] = rotmat_from_quat(q)
% ROTMAT_FROM_QUAT Rotation matrix from unit quaternion.
%
%   [R] = ROTMAT_FROM_QUAT(q) produces a rotation matrix R from the unit 
%   quaternion q.
%
%   Inputs:
%   -------
%    q  - 4x1 quaternion with unit norm.
%
%   Outputs:
%   --------
%    R  - 3x3 orthonormal rotation matrix.

b2 = q(2)^2;
c2 = q(3)^2;
d2 = q(4)^2;

R = [          1 - 2*c2 - 2*d2, 2*q(2)*q(3) - 2*q(1)*q(4), 2*q(1)*q(3) + 2*q(2)*q(4);
     2*q(1)*q(4) + 2*q(2)*q(3),           1 - 2*b2 - 2*d2, 2*q(3)*q(4) - 2*q(1)*q(2);
     2*q(2)*q(4) - 2*q(1)*q(3), 2*q(1)*q(2) + 2*q(3)*q(4),          1 - 2*b2 - 2*c2];