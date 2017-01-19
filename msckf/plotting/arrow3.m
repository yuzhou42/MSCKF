function [] = arrow3(O, D, color, linewidth)
%ARROW3  plots a 3d arrow
%
%	ARROW3(O, D)
%	ARROW3(O, D, COLOR, LINEWIDTH)
%
% Draws an arrow with the origin at O, and in direction D.  Thus, the head
% of the arrow will be at O + D.

% $Id: arrow3.m,v 1.1 2009-03-17 16:40:18 bradleyk Exp $
% Copyright (C) 2005, by Brad Kratochvil

if nargin < 3
    color = 'k';
end

if nargin < 4
    linewidth = 2;
end

if ~isequal([3 1], size(O)) || ~isequal([3 1], size(D))  
  error('SCREWS:arrow3', 'requires 3x1 input vector');
end

tail_length = 10;
tail_width = 2;

len = norm(D);
off = len/tail_length;

if 0 == len,
  return;
end

% the arrow points along the x-axis for now
points = [0 0 0; ...
          len 0 0; ...
          len-off -off/tail_width 0; ...
          len-off off/tail_width 0; ...
          len 0 0;]';

% build a rotation matrix to make our lives easier
R(:,1) = D/len;
R(:,2) = roty(pi/2)*D/len;
if (max(sum(R(:, 1:2).^2, 2)) > 1)
  R(:,2) = rotx(pi/2)*D/len;  
end
R(:,3) = sqrt(ones(3,1) - R(:, 1).^2 - R(:,2).^2) ;

% rotate the points
points = R * points + repmat(O, 1, size(points,2));

hchek = ishold;

% plot everything
plot3(points(1, 1:2), points(2, 1:2), points(3, 1:2), ...
      'color', color, 'linewidth', linewidth);
hold on;
h = patch(points(1, 2:end)', points(2, 2:end)', points(3, 2:end)', color);
set(h, 'LineStyle', 'none');

if 0 == hchek
   hold off
end

