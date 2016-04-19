function [outputFrame] = overlayImage(snapshot,checkerboard,replacement_image)
% snapshot = imread('Image18.png');
% checkerboard = imread('checkerboard_cropped.png');
% replacement_image = flip(imread('test_image.jpg'), 1);

scale = size(checkerboard, 1)/size(replacement_image, 1);
replacement_image = imresize(replacement_image, scale);

snapshot_pts = detectCheckerboardPoints(snapshot);
checkerboard_pts = detectCheckerboardPoints(checkerboard);

transform = estimateGeometricTransform(checkerboard_pts, snapshot_pts, 'projective');

outputView = imref2d(size(snapshot));
warped = imwarp(replacement_image, transform, 'OutputView', outputView);

alphaBlender = vision.AlphaBlender('Operation', 'Binary mask', 'MaskSource', 'Input port');

mask = warped(:,:,1) | ...
       warped(:,:,2) | ...
       warped(:,:,3) > 0;

outputFrame = step(alphaBlender, snapshot, warped, mask);
% figure(1)
% imshow(outputFrame);