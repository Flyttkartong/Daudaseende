%% get camera
camName=webcamlist;
cam=webcam(camName{1});

%h=image(zeros(vidres(2),vidRes(1),nBands));
%preview camera
%h=image([1,2,3,4]);
%preview(cam);%,h);

%set resolution
cam.resolution='1920x1080'

%prompt='Begin capture session? (y/n)';
%s=input(prompt,'s');
%if s=='y'
    %for i=1:30
    i=1;
    while 1
        
        %hold on;
        %plot(i,2*i,'0')
        %plot(i,2*i);
        image=snapshot(cam);
        grayim=rgb2gray(image);
        imagePoints=detectCheckerboardPoints(image);
        imshow(image);
        size(imagePoints)
        if size(imagePoints) > 0
            hold on
            plot(imagePoints(:,1),imagePoints(:,2),'r*','markersize', 100);
        end
        drawnow
        %grayim=rgb2gray(image);
        %[centers,radii]=imfindcircles(grayim,[60 80]);
        %imshow(image);
        %hold on;
        %viscircles(centers,radii);
        i=i+1;
        %drawnow
    end
%end

%exit camera session
prompt='Exit? (y/n)';
s=input(prompt,'s');
if s=='y'
   clear('cam');
end