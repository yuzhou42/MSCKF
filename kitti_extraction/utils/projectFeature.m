function [px] = projectFeature(p_fc_c, K)
%PROJECTFEATURE Summary of this function goes here
%   Detailed explanation goes here

px = homo2cart(K*p_fc_c);
end

