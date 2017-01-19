function qRC = quatRightComp(quat)
%
% Computes the right-hand compound operator form of a quaternion
% using the {i,j,k,1} convention (q^\oplus in Tim's book)
%
    if( size(quat,1) ~= 4 || size(quat,2) ~= 1 )
        error('Input quaternion must be 4x1');
    end
    
    vector = quat(1:3);
    scalar = quat(4);
    
    qRC = [ scalar*eye(3) + crossMat(vector),   vector;
            -vector',                           scalar  ];
end