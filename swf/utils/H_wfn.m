function [H_x] = H_wfn(kMinus1State)
%H_XFN Compute the 6x6 matrix H_w_k  
%Construct the H_x matrix
H_x = zeros(6,6);
H_x(1:3, 1:3) = kMinus1State.C_vi';
H_x(4:6, 4:6) = eye(3);
end

