
load('cameraParams.mat');
reference_image = imread('daudasystem_smaller.jpg');
reference_image = imrotate(reference_image,-90);
reference_image = rgb2gray(reference_image);
replacement_image = imread('test_image.jpg');
scale = size(reference_image, 1)/size(replacement_image, 1);
replacement_image = imresize(replacement_image, scale);
%Obj=read_wobj('tetra.obj')
load('tetraMat.mat');
translationScaleMatrix=[1/100 0 0 0; 0 1/100 0 0; 0 0 1/100 1;0 0 0 1];
translatedPoints=(translationScaleMatrix*[Obj.vertices ones(4,1)]')';
Obj.vertices=translatedPoints(:,1:3);



detected_pts = detectSURFFeatures(reference_image);
[reference_features, reference_pts] = extractFeatures(reference_image, detected_pts);

% Camera setup: may be camera specific, in that case put the following in a
% function or something
vidobj = videoinput('winvideo', 1, 'RGB24_1920x1080');
% vidobj.ROIPosition = [149 297 1618 783];

% Disable frame logging for performance
triggerconfig(vidobj, 'manual');

% Start the image capture
start(vidobj)

% Create a handle used to update the figure
snapshot = getsnapshot(vidobj);
image_handle = imshow(snapshot);

%colormap gray

% Disable EraseMode for performance
set(image_handle, 'EraseMode', 'none');




while ishandle(image_handle)
    % Fetch snapshot from camera
    snapshot = getsnapshot(vidobj);
    
    % Do fancy schmancy stuff with image here!
    image = overlayImage3D(snapshot, reference_features, ...
                         reference_pts, replacement_image, reference_image,cameraParams,Obj);
    
    % Display final image
    try
        set(image_handle, 'CData', image);        
        drawnow;
    catch
        % Ignore "deleted handle" error when window is closed
    end
end


% Stop the image capture and clean up
stop(vidobj)
delete(vidobj)