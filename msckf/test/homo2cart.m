function cartesian = homo2cart(homogeneous)
% PURPOSE:      Converts a D x N array of homogeneous coordinate vectors to
%               cartesian coordinates
%                   D := dimension of the vectors
%                   N := number of vectors
%
% INPUT:        homogeneous - D x N array of homogeneous coordinate vectors
%
% OUTPUT:       cartesian - (D-1) x N array of cartesian coordinate vectors
%
% LAST EDIT:    Lee Clement, 2014-03-10

    scale = repmat( homogeneous(end,:), [size(homogeneous,1)-1, 1] );
    cartesian = homogeneous(1:end-1,:) ./ scale;
end