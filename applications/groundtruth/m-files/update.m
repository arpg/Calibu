function update( im, w, h, K, corners, tag, pose, del )
    if (nargin < 8)
        del = 1e-3
    end
    dp = zeros(6);
    C = cost(im, w, h, pose, K, corners, tag);
    C_last = 1e10;
    gamma = 1e-9;
    while (C < C_last)
        % Construct Jacobian
        J = zeros(6, 1);
        for i =1:6
            J(i) = d1(im, w, h, pose, K, corners, tag, i, del);
        end;
        dp =  gamma*J;
        pose = pose - dp';
        C_last = C;
        C = cost(im, w, h, pose, K, corners, tag)
        im2 = project(w, h, pose, K, corners(1, :), corners(2, :), corners(3, :), corners(4, :), tag, false);
        im3 = abs(im2 - im);
        clf;
        imshow(im3);
    end;    
end

function d = d1(im, w, h, pose, K, corners, tag, i, step)
    temp_pose = pose;
    temp_pose(i) = temp_pose(i) + step;
    cp = cost(im, w, h, temp_pose, K, corners, tag);
    c0 = cost(im, w, h, pose, K, corners, tag);
    d = (cp - c0) / step;
end

function d = d2(im, w, h, pose, K, corners, tag, i, j, step)
    temp_pose = pose;
    temp_pose(j) = temp_pose(j) + step;
    cp = d1(im, w, h, temp_pose, K, corners, tag, i, step);
    c0 = d1(im, w, h, pose, K, corners, tag, i, step);
    d = (cp - c0) / step;
end

function c = cost(im, w, h, pose, K, corners, tag)
    test_im = project(w, h, pose, K, corners(1, :), corners(2, :), corners(3, :), corners(4, :), tag, false);
    C = abs(im - test_im);
    c = sum(C(:));
end