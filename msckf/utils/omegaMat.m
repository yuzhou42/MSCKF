function bigOmega = omegaMat(omega)
%
% Computes the Omega matrix of a 3x1 vector, omega
%
    if(size(omega,1) ~= 3 || size(omega,2) ~= 1)
        error('Input vector must be 3x1');
    end

%　四元数ＪＰＬ坐标系,四元数实部在最后[q1 q2 q3 q0]
%　｜ 0    wz  -wy  wx |
%　｜-wz   0    wx  wy |
%　｜ wy  -wx   0   wz |
%　｜-wx  -wy  -wz   0 |
    bigOmega = [ -crossMat(omega),  omega;
                 -omega',            0 ];
end