function [outputFrame] = overlayImage(snapshot,reference_features,reference_pts,replacement_image,reference_image)

snapshot_gray = rgb2gray(snapshot);
detected_pts = detectSURFFeatures(snapshot_gray);
[snapshot_features, snapshot_pts] = extractFeatures(snapshot_gray, detected_pts);

% imshow(snapshot_gray);
% hold on
% plot(snapshot_pts,'showOrientation',true);

index_pairs = matchFeatures(reference_features, snapshot_features);

if length(index_pairs) >= 4
    matched_snapshot_pts = snapshot_pts(index_pairs(:,2));
    matched_reference_pts = reference_pts(index_pairs(:,1));
    
%     figure; showMatchedFeatures(snapshot_gray,reference_image,matched_snapshot_pts,matched_reference_pts, 'montage');

    transform = estimateGeometricTransform(matched_reference_pts, matched_snapshot_pts, 'projective');

    outputView = imref2d(size(snapshot));
    warped = imwarp(replacement_image, transform, 'OutputView', outputView);

    alphaBlender = vision.AlphaBlender('Operation', 'Binary mask', 'MaskSource', 'Input port');

    mask = warped(:,:,1) | ...
           warped(:,:,2) | ...
           warped(:,:,3) > 0;

    outputFrame = step(alphaBlender, snapshot, warped, mask);
else
    outputFrame = snapshot;
end

end