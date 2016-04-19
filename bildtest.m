%load data
im1=imread('projekt_datorseende/calibrering/image21.png');
im2=imread('projekt_datorseende/calibrering/image24.png');


%get calibration
load('projekt_datorseende/calibrering/calibrationCanon600D');

%get chessboard corners
imagePoints1=detectCheckerboardPoints(im1);
imagePoints2=detectCheckerboardPoints(im2);

imagePoints1=[imagePoints1 ones(length(imagePoints1),1)]';
imagePoints2=[imagePoints2 ones(length(imagePoints2),1)]';;

cali{1}=cameraParams.IntrinsicMatrix\imagePoints1;
cali{2}=cameraParams.IntrinsicMatrix\imagePoints2;

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


F1=(cameraParams.IntrinsicMatrix')\E/cameraParams.IntrinsicMatrix;
l=F1*cali{1};
l = l./sqrt( repmat(l(1 ,:).^2 + l(2 ,:).^2 ,[3 1]));

r=randi(length(cali{1}), 20,1);
hist(abs(sum(l.*cali{2})),100);
title('Distribution of distance between image points and their corresponding epipolar line')
xlabel('distance between the epipolar line and the corresponding image point')
ylabel('number of image points')

meanD2=mean(abs(sum(l.*cali{2})))