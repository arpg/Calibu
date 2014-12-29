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

im = project(w, h, pose, K, corners(1, :), corners(2, :), corners(3, :), corners(4, :), tag, true);
imshow(im);

%perturb the pose
test_pose = pose;
step = 0.01;
for i=1:6
    if (rand < 0.5)
        mult = -1;
    else
        mult = 1;
    end;
    test_pose(i) = pose(i) + mult*rand*step;
end
figure;
im2 = project(w, h, test_pose, K, corners(1, :), corners(2, :), corners(3, :), corners(4, :), tag, false);
imd = abs(im2 - im);
imshow(imd);


figure;
im3 = abs(im2 - im);
imshow(im3);

update( im, w, h, K, corners, tag, test_pose )