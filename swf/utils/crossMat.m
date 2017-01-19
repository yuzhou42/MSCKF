function [vecCross] = crossMat(vec)
%CROSSMAT Returns the 3x3 skew symmetric matrix associated with the 3x1 vector
vecCross = [0 -vec(3) vec(2) 
            vec(3) 0 -vec(1) 
            -vec(2) vec(1) 0];
end

