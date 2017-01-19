function vecCross = crossMat(vec)
%
% Computes the cross-product matrix of a 3x1 vector
%	
    if(size(vec,1) ~= 3 || size(vec,2) ~= 1)
        error('Input vector must be 3x1');
    end

    vecCross = [ 0, 		-vec(3), 	vec(2);
				 vec(3),	0,			-vec(1);
				 -vec(2),	vec(1),		0        ];
end