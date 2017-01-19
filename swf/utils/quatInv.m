function qInv = quatInv(quat)
%
% Computes the inverse (or conjugate) of a unit quaternion
% using the {i,j,k,1} convention
%
    if( size(quat,1) ~= 4 || size(quat,2) ~= 1 )
        error('Input quaternion must be 4x1');
    end
    
    if( abs(norm(quat) - 1) > eps )
        error('Input quaternion must be unit-length');
    end
    
    qInv(4,1) = quat(4);
    qInv(1:3,1) = -quat(1:3);
end