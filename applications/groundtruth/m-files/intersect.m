function I = intersect( i, j, K, pose, corners )
    Ki = inv(K);
    p0 = corners(1, :);
    l0 = [pose(1) pose(2) pose(3)];
    dx = corners(3, :) - corners(2, :);
    dy = corners(1, :) - corners(2, :);
    n = cross(dx, dy);
%     n = n / norm(n);

    T = Cart2T(pose);
    pt = [Ki * [i j 1]'; 1];
    lf = T * pt;

    l = lf(1:3) / lf(4); % this is the ray in the world reference frame
    l = l - l0'; % equation of the line
    if (dot(l, n) == 0)
        d = 0;    
    else
        d = dot((p0 - l0), n) / dot(l, n);
    end;
    if (d < 0)
        d = 0;
    end;
    I = [l0; l0 + d*l'];
end