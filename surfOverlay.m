function [outputFrame, trackedPoints, transform] = surfOverlay(snapshot, replacementImage, referenceFeatures, referencePoints)

snapshotGray = rgb2gray(snapshot);
[snapshotFeatures, snapshotPoints] = getFeatures(snapshotGray);

indexPairs = matchFeatures(referenceFeatures, snapshotFeatures);

if length(indexPairs) >= 4
    matchedSnapshotPoints = snapshotPoints(indexPairs(:,2));
    matchedReferencePoints = referencePoints(indexPairs(:,1));

    [outputFrame, transform] = overlayImage(snapshot, replacementImage, matchedSnapshotPoints.Location, matchedReferencePoints.Location);
    trackedPoints = matchedSnapshotPoints.Location;
else
    outputFrame = snapshot;
    trackedPoints = [];
    transform = [];
end

end