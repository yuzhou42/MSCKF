function homogeneous = cart2homo(cartesian)
% PURPOSE:      Converts a D x N array of cartesian coordinate vectors to
%               homogeneous coordinates
%                   D := dimension of the vectors
%                   N := number of vectors
%
% INPUT:        cartesian - D x N array of cartesian coordinate vectors
%
% OUTPUT:       homogeneous - (D+1) x N array of homogeneous coordinate vectors
%
% LAST EDIT:    Lee Clement, 2014-03-10

    homogeneous = [cartesian; ones(1,size(cartesian,2))];
end