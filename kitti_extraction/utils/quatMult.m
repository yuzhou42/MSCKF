function quatProd = quatMult(quat1, quat2)
% 
% Multiplies two quaternions using the {i,j,k,1} convention
%
    if( size(quat1,1) ~= 4 || size(quat1,2) ~= 1 ...
        || size(quat2,1) ~= 4 || size(quat2,2) ~= 1 )
        error('Input quaternions must be 4x1');
    end
    
    quatProd = quatLeftComp(quat1) * quat2;
end