% Image setup
referenceImage = rgb2gray(imread('daudasystem_smaller.jpg'));
replacementImage = imread('test_image.jpg');
replacementImage = resizeFirst(replacementImage, referenceImage);

% Get SURF features and points
[referenceFeatures, referencePoints] = getFeatures(referenceImage);

% Get video object
vidobj = setupVideoObject();

% Start the image capture
start(vidobj)

% Create a handle used to update the figure and display it
imageHandle = setupImageHandle(vidobj);

% Create point tracker object
pointTracker = vision.PointTracker('MaxBidirectionalError', 2);
trackedPoints = [];
trackerInitialized = false;
frameCount = 0;

while ishandle(imageHandle)
    frameCount = frameCount + 1;
    
    % Fetch snapshot from camera
    snapshot = getsnapshot(vidobj);
    
    if ~isempty(trackedPoints) && frameCount < 20
        % Overlay replacement image using tracker
        [image, trackedPoints, transform] = trackerOverlay(pointTracker, snapshot, replacementImage, trackedPoints, transform);
    else
        frameCount = 0;
        % Overlay replacement image using SURF features
        [image, trackedPoints, transform] = surfOverlay(snapshot, replacementImage, referenceFeatures, referencePoints);
        
        % Set new points to track if SURF matching was sucessful
        if ~isempty(trackedPoints)
            if trackerInitialized
                setPoints(pointTracker, trackedPoints);
            else
                initialize(pointTracker, trackedPoints, snapshot);
                trackerInitialized = true;
            end
        end
    end
    
    % Do fancy schmancy stuff with image here!
%     image = overlayImage(snapshot, referenceFeatures, ...
%                          referencePoints, replacementImage, referenceImage);
    
    % Display final image
    try
        set(imageHandle, 'CData', image);
        drawnow;
    catch
        % Ignore "deleted handle" error when window is closed
    end
end

% Stop the image capture and clean up
stop(vidobj)
delete(vidobj)