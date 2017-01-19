function [yMeas] = stereoCamProject(p_fc_c, calibParams)
% triangulate Triangulates 3D point from stereo camera measurement

c_u = calibParams.c_u;
c_v = calibParams.c_v;
f_u = calibParams.f_u;
f_v = calibParams.f_v;
b = calibParams.b;

x = p_fc_c(1);
y = p_fc_c(2);
z = p_fc_c(3);


yMeas = (1/z)*[f_u*x; f_v*y; f_u*(x-b); f_v*y] + [c_u;c_v;c_u;c_v];
end
