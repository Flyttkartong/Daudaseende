function image_handle = setupImageHandle( vidobj )

snapshot = getsnapshot(vidobj);
image_handle = imshow(snapshot);

% Disable EraseMode for performance (unnecessary?)
set(image_handle, 'EraseMode', 'none');

end

