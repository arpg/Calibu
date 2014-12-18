% mex -I/opt/local/include/eigen3 project.cpp;
file = fopen('/Users/faradazerage/code/CoreDev/Calibu/build/applications/groundtruth/data.out', 'r');
pose = fscanf(file, '%f\t%f\t%f\t%f\t%f\t%f\n', 6);
pose = pose';
corners = fscanf(file, '%f', 12);
corners = reshape(corners, 3, 4);
corners = corners';
corners = [corners; corners(1, :)];

cs = fscanf(file, '%f', 8);
cs = reshape(cs, 2, 4);
cs = cs';
cs = [cs; cs(1, :)];

colors = fscanf(file, '%d', 2);
tg = fscanf(file, '%d', 36);
tg = reshape(tg, 6, 6);
tg = tg';
tag = zeros(8, 8);
tag(2:7, 2:7) = tg(:,:);

K = [476.772       0 337.554;
    0 474.912 276.485;
    0       0       1];
w = 640;
h = 480;

im = project(w, h, pose, K, corners(1, :), corners(2, :), corners(3, :), corners(4, :), tag);
imshow(im);

% figure;
% 
% plot3(corners(:, 1), corners(:, 2), corners(:, 3));
% hold on;
% 
% plot3(pose(1), pose(2), pose(3), 'ro');
% 
% l = intersect(cs(1, 1), cs(1, 2), K, pose, corners);
% l = [l; intersect(cs(2, 1), cs(2, 2), K, pose, corners);];
% l = [l; intersect(cs(3, 1), cs(3, 2), K, pose, corners);];
% l = [l; intersect(cs(4, 1), cs(4, 2), K, pose, corners);];
% l = [l; intersect(506, 159, K, pose, corners);];
% 
% t = intersect(506, 159, K, pose, corners);
% inpoly( t, corners);
% 
% tl = corners(4, :);
% tr = corners(1, :);
% bl = corners(3, :);
% br = corners(2, :);
% 
% dx = (tr - tl) / 8;
% dy = (bl - tl) / 8;
% 
% a = t(2, :) - tl;
% x = dot(a, dx) / (norm(dx)*norm(dx));
% y = dot(a, dy) / (norm(dy)*norm(dy));
% 
% plot3(l(:, 1), l(:, 2), l(:, 3));
% 
% dx = corners(3, :) - corners(2, :);
% dy = corners(1, :) - corners(2, :);
% n = cross(dx, dy);
% n = n / norm(n);
% 
% nrm = [corners(1, 1:3); corners(1, 1:3) + n];
% plot3(nrm(:, 1), nrm(:, 2), nrm(:, 3), '-k');
% plot3(corners(1, 1), corners(1, 2), corners(1, 3), 'or');
% plot3(corners(2, 1), corners(2, 2), corners(2, 3), 'og');
% plot3(corners(3, 1), corners(3, 2), corners(3, 3), 'ob');
