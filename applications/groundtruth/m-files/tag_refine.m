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
tag(2:7, 2:7) = tg(:,:)
img = im2double(imread('/Users/faradazerage/code/CoreDev/Calibu/build/applications/groundtruth/image.jpg'));
synth = im2double(imread('/Users/faradazerage/code/CoreDev/Calibu/build/applications/groundtruth/synthetic.jpg'));

[w,h] = size(img);

%imshow(img, []);
%hold on;
%plot(cs(:, 1), cs(:, 2));
%figure;
%imshow(synth, []);
%figure;
point_in_tag(corners, cs, pose);