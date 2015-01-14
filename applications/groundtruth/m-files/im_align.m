dir = '/Users/faradazerage/code/CoreDev/Calibu/build/applications/groundtruth/';
% dir = '/Users/faradazerage/code/Align/data/';
synth = single(imread(strcat(dir, 'synthetic.jpg')));
captured = single(imread(strcat(dir, 'captured.jpg')));
% [f, d] = vl_sift(captured, 'verbose');
% image(captured);
% pts = [f(1, :); f(2, :)];
% circle(f(1, :), f(2, :), 2);

[f1,d1] = vl_sift(captured) ;
[f2,d2] = vl_sift(synth) ;

size(d1)
size(d2)

[matches, scores] = vl_ubcmatch(d1,d2) ;

image(synth)
circle(f2(1, :), f2(2, :), 2);