function [outputFrame, trackedPoints, transform] = trackerOverlay(pointTracker, snapshot, replacementImage, oldTrackedPoints, transform)

[trackedPoints, isValid] = step(pointTracker, snapshot);

newValidLocations = trackedPoints(isValid,:);
oldValidLocations = oldTrackedPoints(isValid,:);

if nnz(isValid) >= 4
    [outputFrame, transform] = overlayImage(snapshot, replacementImage, newValidLocations, oldValidLocations, transform);
    trackedPoints = newValidLocations;
    pointTracker.setPoints(newValidLocations);
else
    outputFrame = snapshot;
    trackedPoints = [];
    transform = [];
end

end