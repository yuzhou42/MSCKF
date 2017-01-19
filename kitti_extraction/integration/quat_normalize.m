function [qn] = quat_normalize(q)
% QUAT_NORMALIZE Normalize quaternion.
%
%   [qn] = QUAT_NORMALIZE(q) returns the normalized verion of q, with unit
%   length.
%
%   Inputs:
%   -------
%    q  - 4x1 quaternion.
%
%   Outputs:
%   --------
%    qn  - 4x1 normalized quaternion.
%
%   See also QUAT_NORM.

n  = sqrt(q.'*q);
qn = q/n;