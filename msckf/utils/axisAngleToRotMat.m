function Psi = axisAngleToRotMat(psi)
%
% Converts an axis-angle rotation vector into a rotation matrix
%
% psi               - axis-angle rotation vector
%
    np = norm(psi);
    cp = cos(np);
    sp = sin(np);
    pnp = psi / norm(psi);
    
    Psi = cp * eye(3) + (1 - cp) * (pnp * pnp') - sp * crossMat(pnp);
end