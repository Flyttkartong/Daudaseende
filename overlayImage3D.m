function [outputFrame] = overlayImage(snapshot,reference_features,reference_pts,replacement_image,reference_image,calibrationSession,Obj)

snapshot_gray = rgb2gray(snapshot);
detected_pts = detectSURFFeatures(snapshot_gray);
[snapshot_features, snapshot_pts] = extractFeatures(snapshot_gray, detected_pts);

% imshow(snapshot_gray);
% hold on
% plot(snapshot_pts,'showOrientation',true);

index_pairs = matchFeatures(reference_features, snapshot_features);

if length(index_pairs) >= 8
    matched_snapshot_pts = snapshot_pts(index_pairs(:,2));
    matched_reference_pts = reference_pts(index_pairs(:,1));
    
%     figure; showMatchedFeatures(snapshot_gray,reference_image,matched_snapshot_pts,matched_reference_pts, 'montage');

    transform = estimateGeometricTransform(matched_reference_pts, matched_snapshot_pts, 'projective');
    load('calibrationSession.mat')
    [F,epinliers]=estimateFundamentalMatrix(matched_snapshot_pts,matched_reference_pts);
    inliers1=matched_snapshot_pts(epinliers,:);
    inliers2=matched_reference_pts(epinliers,:);
    [R,t]= cameraPose(F,calibrationSession.CameraParameters,inliers1,inliers2);
    P=calibrationSession.CameraParameters.IntrinsicMatrix*[R t'];
    ProjectedPoints=pflat(P*[Obj.vertices'; ones(1,length(Obj.vertices))]);
    face1=Obj.objects(4).data.vertices(1,:);
    face2=Obj.objects(4).data.vertices(2,:);
    face3=Obj.objects(4).data.vertices(3,:);
    face4=Obj.objects(4).data.vertices(4,:);
    fill(ProjectedPoints(1,face1),ProjectedPoints(2,face1),'r',ProjectedPoints(1,face2),ProjectedPoints(2,face2),'b',ProjectedPoints(1,face3),ProjectedPoints(2,face3),'g',ProjectedPoints(1,face4),ProjectedPoints(2,face4),'y');
    
    

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