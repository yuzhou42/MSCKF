function alpha = wrapToPi(alpha)

% wrap to [0..2*pi]
alpha = mod(alpha,2*pi);

% wrap to [-pi..pi]
idx = alpha>pi;
alpha(idx) = alpha(idx)-2*pi;
