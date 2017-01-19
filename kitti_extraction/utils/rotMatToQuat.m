function [q] = rotMatToQuat(R)

[r,c] = size( R );
R=R';

if( r ~= 3 | c ~= 3 )
    fprintf( 'R must be a 3x3 matrix\n\r' );
    return;
end

Rxx = R(1,1); Rxy = R(1,2); Rxz = R(1,3);
Ryx = R(2,1); Ryy = R(2,2); Ryz = R(2,3);
Rzx = R(3,1); Rzy = R(3,2); Rzz = R(3,3);

w = sqrt( trace( R ) + 1 ) / 2;

% check if w is real. Otherwise, zero it.
if( imag( w ) > 0 )
     w = 0;
end

x = sqrt( 1 + Rxx - Ryy - Rzz ) / 2;
y = sqrt( 1 + Ryy - Rxx - Rzz ) / 2;
z = sqrt( 1 + Rzz - Ryy - Rxx ) / 2;

[~, i ] = max( [w,x,y,z] );

if( i == 1 )
    x = ( Rzy - Ryz ) / (4*w);
    y = ( Rxz - Rzx ) / (4*w);
    z = ( Ryx - Rxy ) / (4*w);
end

if( i == 2 )
    w = ( Rzy - Ryz ) / (4*x);
    y = ( Rxy + Ryx ) / (4*x);
    z = ( Rzx + Rxz ) / (4*x);
end

if( i == 3 )
    w = ( Rxz - Rzx ) / (4*y);
    x = ( Rxy + Ryx ) / (4*y);
    z = ( Ryz + Rzy ) / (4*y);
end

if( i == 4 )
    w = ( Ryx - Rxy ) / (4*z);
    x = ( Rzx + Rxz ) / (4*z);
    y = ( Ryz + Rzy ) / (4*z);
end

q = [x; y; z; w];

end