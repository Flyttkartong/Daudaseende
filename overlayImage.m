function [outputFrame, transform] = overlayImage(snapshot, replacementImage, snapshotPoints, referencePoints, previousTransform)

try
    transform = estimateGeometricTransform(referencePoints, snapshotPoints, 'projective');
    
    if rcond(transform.T) < 1e-06
       throw(MException()); 
    end
    
    if exist('previousTransform', 'var') == 1
        transform.T = previousTransform.T * transform.T;
    end

    outputView = imref2d(size(snapshot));
    warped = imwarp(replacementImage, transform, 'OutputView', outputView);

    alphaBlender = vision.AlphaBlender('Operation', 'Binary mask', 'MaskSource', 'Input port');

    mask = warped(:,:,1) | ...
           warped(:,:,2) | ...
           warped(:,:,3) > 0;

    outputFrame = step(alphaBlender, snapshot, warped, mask);
catch
    outputFrame = snapshot;
    transform = [];
end

% Display tracked points (for debugging)
locs = snapshotPoints;
circlePositions = [locs(:,1) locs(:,2) 3*ones(length(locs), 1)];
outputFrame = insertShape(outputFrame, 'Circle', circlePositions);

end