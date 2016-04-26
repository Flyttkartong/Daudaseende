%load data
im1=imread('projekt_datorseende/calibrering/image13.png');
im2=imread('projekt_datorseende/calibrering/image15.png');


%get calibration
%load('projekt_datorseende/calibrering/calibrationSession');
calib

%get chessboard corners
imagePoints1=detectCheckerboardPoints(im1);
imagePoints2=detectCheckerboardPoints(im2);

imagePoints1=[imagePoints1 ones(length(imagePoints1),1)]';
imagePoints2=[imagePoints2 ones(length(imagePoints2),1)]';;

cali{1}=cameraParams.IntrinsicMatrix\imagePoints1;
cali{2}=cameraParams.IntrinsicMatrix\imagePoints2;

figure;
imshow(im1);
hold on;
plot(imagePoints1(1,11:end),imagePoints1(2,11:end),'or','markersize',30);
plot(imagePoints1(1,1:5),imagePoints1(2,1:5),'*g','markersize',30);
plot(imagePoints1(1,6:10),imagePoints1(2,6:10),'*b','markersize',30);
plot(imagePoints1(1,40:54),imagePoints1(2,40:54),'oy','markersize',30);
hold off;

figure;
imshow(im2);
hold on;
plot(imagePoints2(1,11:end),imagePoints2(2,11:end),'or','markersize',30);
plot(imagePoints2(1,1:5),imagePoints2(2,1:5),'*g','markersize',30);
plot(imagePoints2(1,6:10),imagePoints2(2,6:10),'*b','markersize',30);
plot(imagePoints2(1,40:54),imagePoints2(2,40:54),'oy','markersize',30);
hold off;

figure;
%imshow(cameraParams.IntrinsicMatrix\im1);
hold on;
plot(cali{1}(1,11:end),cali{1}(2,11:end),'or','markersize',30);
plot(cali{1}(1,1:5),cali{1}(2,1:5),'*g','markersize',30);
plot(cali{1}(1,6:10),cali{1}(2,6:10),'*b','markersize',30);
plot(cali{1}(1,40:54),cali{1}(2,40:54),'oy','markersize',30);
hold off;

figure;
%imshow(cameraParams.IntrinsicMatrix\im2);
hold on;
plot(cali{2}(1,11:end),cali{2}(2,11:end),'or','markersize',30);
plot(cali{2}(1,1:5),cali{2}(2,1:5),'*g','markersize',30);
plot(cali{2}(1,6:10),cali{2}(2,6:10),'*b','markersize',30);
plot(cali{2}(1,40:54),cali{2}(2,40:54),'oy','markersize',30);
hold off;


%create Mmatrix for essential matrix computation
M=createM(cali);

%Compute essential matrix
[U,S,V]=svd(M);
v=V(:,end);
Eapprox=reshape(v,[3 3]);
[U,S,V]=svd(Eapprox);
if det(U*V')>0
    E =U*diag([1 1 0])*V';
else
    V = -V;
    E = U*diag([1 1 0])*V';
end
E=E./E(end)

cali{2}'*E*cali{1}

F1=(cameraParams.IntrinsicMatrix')\E/cameraParams.IntrinsicMatrix;
l=F1*imagePoints1;
l = l./sqrt( repmat(l(1 ,:).^2 + l(2 ,:).^2 ,[3 1]));
r=1:20;%randi(length(imagePoints1), 1,20);
figure;
imshow(im2);
colormap gray;
hold on;
plot(imagePoints2(1,r),imagePoints2(2,r),'g+','Markersize',30)
rital(l(:,r),'k');
hold off
legend('image points','epipolar lines')

figure;

hist(abs(sum(l.*imagePoints2)),100);
title('Distribution of distance between image points and their corresponding epipolar line')
xlabel('distance between the epipolar line and the corresponding image point')
ylabel('number of image points')

meanD2=mean(abs(sum(l.*imagePoints2)))