function in = inpoly(x, corners)
    x = x(2, :)
    o = corners(1, :) - x;
    t = corners(2, :) - x;
    r = corners(3, :) - x;
    f = corners(4, :) - x;
    o = o / norm(o);
    t = t / norm(t);
    r = r / norm(r);
    f = f / norm(f);
    
    theta = 0;
    theta = theta + acos( dot(o,t) );
    theta = theta + acos( dot(r,t) );
    theta = theta + acos( dot(r,f) );
    theta = theta + acos( dot(o,f) );
    
    
    in = 0;
    if (abs(theta - 6.28) < 0.01)
        in = 1;
    end;
end