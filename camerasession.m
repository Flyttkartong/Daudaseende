%% get camera
camName=webcamlist;
cam=webcam(camName{1});

%preview camera
preview(cam);

%set resolution
cam.resolution='1920x1080'

prompt='Begin capture session? (y/n)';
s=input(prompt,'s');
if s=='y'
    for i=1:30
        image=snapshot(cam);
        grayim=rgb2gray(image);
        [centers,radii]=imfindcircles(grayim,[60 80]);
        imshow(image);
        hold on;
        viscircles(centers,radii);
        drawnow
    end
end

%exit camera session
prompt='Exit? (y/n)';
s=input(prompt,'s');
if s=='y'
   clear('cam');
end