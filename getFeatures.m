function [features, points] = getFeatures( image )

detected_pts = detectSURFFeatures(image);
[features, points] = extractFeatures(image, detected_pts);

end

