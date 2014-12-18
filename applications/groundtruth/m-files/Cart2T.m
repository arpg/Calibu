function T = Cart2T( pose )
x = pose(1);
y = pose(2);
z = pose(3);
r = pose(4);
p = pose(5);
q = pose(6);

cr = cos( r );
cp = cos( p );
cq = cos( q );

sr = sin( r );
sp = sin( p );
sq = sin( q );

T(1,1) = cp*cq;
T(1,2) = -cr*sq+sr*sp*cq;
T(1,3) = sr*sq+cr*sp*cq;
T(1,4) = 0;

T(2,1) = cp*sq;
T(2,2) = cr*cq+sr*sp*sq;
T(2,3) = -sr*cq+cr*sp*sq;
T(2,4) = 0;

T(3,1) = -sp;
T(3,2) = sr*cp;
T(3,3) = cr*cp;
T(3,4) = 0;

T(1,4) = x;
T(2,4) = y;
T(3,4) = z;
T(4,4) = 1;

end