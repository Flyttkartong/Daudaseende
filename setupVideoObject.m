function vidobj = setupVideoObject()

vidobj = videoinput('winvideo', 1, 'RGB24_1920x1080');
vidobj.ROIPosition = [149 297 1618 783];

% Disable frame logging for performance
triggerconfig(vidobj, 'manual');

end