function [] = drawframe(r, scale, alternate_color)
%DRAWFRAME  plots a graphical description of a coordinate frame
%
%	DRAWFRAME(T)
%	DRAWFRAME(T, SCALE)
%
% H is a homogeneous transformation.  R is a rotation matrix, P is a point.
%
% See also: DRAWFRAMEDIFF, DRAWFRAMETRAJ, ANIMATEFRAMETRAJ.

% $Id: drawframe.m,v 1.1 2009-03-17 16:40:18 bradleyk Exp $
% Copyright (C) 2005, by Brad Kratochvil

  if 1 == nargin,
    scale = 1;
  end
  
  if ~exist('alternate_color', 'var'),
    alternate_color = false;
  end

  p = r(1:3,4);
  r = r(1:3,1:3);

  plot3(p(1), p(2), p(3));
  
  hchek = ishold;
  hold on
  
  if (isequal(zeros(3,1), p')) && (isequal(eye(3),r)),
    % use gray for the base frame
    arrow3(p, scale*r(1:3,1), 'k');
    arrow3(p, scale*r(1:3,2), 'k');
    arrow3(p, scale*r(1:3,3), 'k');
    
    texts(r(1:3,1) + scale*0.05, 'x');
    texts(r(1:3,2) + scale*0.05, 'y');
    texts(r(1:3,3) + scale*0.05, 'z');    
  else    
    if alternate_color,
      arrow3(p, scale*r(1:3,1), 'c');
      arrow3(p, scale*r(1:3,2), 'm');
      arrow3(p, scale*r(1:3,3), 'k');
    else
      arrow3(p, scale*r(1:3,1), 'r');
      arrow3(p, scale*r(1:3,2), 'g');
      arrow3(p, scale*r(1:3,3), 'b');
    end
  end

  xlabel('x');
  ylabel('y');
  zlabel('z');
  
  if hchek == 0
     hold off
  end

end