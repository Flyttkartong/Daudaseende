% Setup: may be camera specific, in that case put the following in a
% function or something
vidobj = videoinput('winvideo', 1, 'RGB24_1920x1080');
vidobj.ROIPosition = [149 297 1618 783];

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
    imagePoints=detectCheckerboardPoints(snapshot);
    
    % Display final image
    try
        
        set(image_handle, 'CData', snapshot);
        
        % Display plot
        if size(imagePoints) > 0
            hold on
            plot_handle = plot(imagePoints(1,1),imagePoints(1,2),'g*','markersize', 100,imagePoints(2:end,1),imagePoints(2:end,2),'r*','markersize', 100);
            hold off
        end
        
        drawnow;
        delete(plot_handle);
    catch
        % Ignore "deleted handle" error when window is closed
    end
end

% Stop the image capture and clean up
stop(vidobj)
delete(vidobj)
close all