function in = point_in_tag(corners, cs, pose)
p0 = corners(1, :);
l0 = pose(1, 1:3);
dx = corners(3, :) - corners(2, :);
dy = corners(1, :) - corners(2, :);
n = cross(dx, dy);
n = n / norm(n);

K = [476.772       0 337.554;
    0 474.912 276.485;
    0       0       1];
Ki = inv(K);

x0 = floor(min(cs(:, 1)))
x1 = ceil(max(cs(:, 1)))
y0 = floor(min(cs(:, 2)))
y1 = ceil(max(cs(:, 2)))

% count = 0;
% for i=x0:x1
%     for j = y0:y1
%         pt = [i j 1];
%         l = (Ki * pt')';
%         T = Cart2T( pose );
%         l_temp = [l 1];
%         lf = (T*l_temp')';
%         l = lf(1:3) / lf(4);
%         d = dot((p0 - l0), n) / dot(l, n);
%         I = d*l + l0;
%         plot3(I(1), I(2), I(3), 'ro');
%     end
% end
I = [];
T = Cart2T( pose )

R = T(1:3, 1:3)
rot = [1 0 0; 0 1 0; 0 0 -1];
% rot = [1 0 0; 0 1 0; 0 0 1];
corners(:, 1:3) = (rot*corners(:, 1:3)')';
plot3(corners(:, 1), corners(:, 2), corners(:, 3));
hold on;


R = rot'*R*rot
l0 = (rot*l0')'
T = [R(1, 1:3) l0(1); R(2, 1:3) l0(2); R(3, 1:3) l0(3); 0 0 0 1]

for count=1:size(cs, 1)
         pt = [cs(count, 1) cs(count, 2) 1];
         l = (Ki * pt')'; % this is the ray in the camera's reference frame
         l_temp = [l 1];
         lf = (T*l_temp')';
         l = lf(1:3) / lf(4); % this is the ray in the world reference frame
         d = dot((p0 - l0), n) / dot(l, n);
         I = [I; d*l + l0];
end;
pt = [320 240 1];
l = (Ki * pt')'; % this is the ray in the camera's reference frame
l_temp = [l 1];
lf = (T*l_temp')';
l = lf(1:3) / lf(4); % this is the ray in the world reference frame
d = dot((p0 - l0), n) / dot(l, n);
l1 = [l0; d*l + l0];
% plot3(l1(:, 1), l1(:, 2), l1(:, 3), 'g-');

% plot3(I(:, 1), I(:, 2), I(:, 3), 'r');
lkat = T*[0; 0; .25; 1];
lkat = lkat(1:3) / lkat(4)
l0 = [l0; lkat'];
plot3(l0(:, 1), l0(:, 2), l0(:, 3), 'k-');


nrm = [corners(1, 1:3); corners(1, 1:3) + n];
plot3(nrm(:, 1), nrm(:, 2), nrm(:, 3), '-k');
plot3(corners(1, 1), corners(1, 2), corners(1, 3), 'or');
plot3(corners(2, 1), corners(2, 2), corners(2, 3), 'og');
plot3(corners(3, 1), corners(3, 2), corners(3, 3), 'ob');
%frustum = [l0; I(1, 1:3); I(2, 1:3); l0; I(3, 1:3); I(2, 1:3); I(4, 1:3); l0];
%plot3(frustum(:, 1), frustum(:, 2), frustum(:, 3), 'b');
in = false;
end

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