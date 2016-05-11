close all
clear
clc

load('cameraParams.mat')

vidobj = VideoReader('MVI_9240.MOV');
i1 = readFrame(vidobj);
vidobj.CurrentTime = vidobj.Duration*0.8;
i2 = readFrame(vidobj);

% i1=imread('kruka1.jpg');
% i2=imread('kruka2.jpg');
%i1=undistortImage(i1,cameraParams);
%i2=undistortImage(i2,cameraParams);
imagesc(i1)
figure;
imagesc(i2)
i1_gray = rgb2gray(i1);
detected_pts_i1 = detectSURFFeatures(i1_gray,'NumOctaves',8);
i2_gray = rgb2gray(i2);
detected_pts_i2 = detectSURFFeatures(i2_gray,'NumOctaves',8);
[snapshot_features_i1, i1_pts] = extractFeatures(i1_gray, detected_pts_i1);
[snapshot_features_i2, i2_pts] = extractFeatures(i2_gray, detected_pts_i2);
index_pairs = matchFeatures(snapshot_features_i1, snapshot_features_i2);
matched_i2_pts = i2_pts(index_pairs(:,2));
matched_i1_pts = i1_pts(index_pairs(:,1));
figure;
showMatchedFeatures(i1,i2,matched_i1_pts,matched_i2_pts,'Montage')
K=cameraParams.IntrinsicMatrix';
input1all=[matched_i1_pts.Location' ;ones(1,matched_i1_pts.Count)];
input2all=[matched_i2_pts.Location' ; ones(1,matched_i2_pts.Count)];
input1all=K\input1all;
input2all=K\input2all;

im1=i1;
im2=i2;

iterations=10000;
bestE=[];
%bestEInd=0.02;
inlierDiscriminator=0.0008;
bestIndices=[];
comparator=zeros(1,size(input2all,2));
inliermask=zeros(1,size(input2all,2));
nbrOfInliers=zeros(1,iterations);
inliercounter=[];
inliers={};
Ecell={};
for j=1:iterations
    indices=randi(length(input1all),5,1);
    input1=input1all(:,indices);
    input2=input2all(:,indices);
    
    Evec=calibrated_fivepoint(input1(:,1:5), input2(:,1:5));
    if size(Evec,1)~=1
        for a=1:size(Evec,2)
            E=reshape(Evec(:,a),3,3);
            Ecell{j,a}=E;
            %Test epipolar contraint
            for i=1:size(input2all,2)
                comparator(i)=input2all(:,i)'*E*input1all(:,i);
                comparator(i)=abs(comparator(i));
                inliermask(i)= comparator(i)<inlierDiscriminator;
%                 inliercounter=length(find(inlierIndex));
            end
            inliers{j,a}=find(inliermask);
            inliercounter(j,a)=sum(inliermask);
            %comparator=max(input2all'*E*input1all);
%             if inliercounter>maxinliers;
%                 bestE=E;
%                 bestEInd=max(comparator);
%                 bestIndices=find(inlierIndex);
%             end
        end
    end
end
[bestRow,bestCol]=find(inliercounter==max(max(inliercounter)));
bestE=Ecell{bestRow,bestCol};
bestIndices=inliers{bestRow,bestCol};



if isempty(bestE)==1
    %locs = matched_snapshot_pts_coord;
    %circlePositions = [locs(:,1) locs(:,2) 3*ones(length(locs), 1)];
    %outputFrame = insertShape(snapshot, 'Circle', circlePositions);
    disp('Could not compute Essential matrix')
    return
else
    figure
    % Not completely sure if this is the correct way to index and display the points
    showMatchedFeatures(i1,i2,matched_i1_pts(bestIndices),matched_i2_pts(bestIndices),'Montage')
    size(matched_i1_pts(bestIndices))
end

[U,S,V]=svd(bestE);
W=[0 -1 0; 1 0 0; 0 0 1];
Pcell{1}=[U*W*V' U(:,3)];
Pcell{2}=[U*W*V' -U(:,3)];
Pcell{3}=[U*W'*V' U(:,3)];
Pcell{4}=[U*W'*V' -U(:,3)];
P1=[eye(3) [0 0 0]'];

P_2=Pcell;

xtilde={input1all(:,bestIndices),input2all(:,bestIndices)};


%% Setup DLT

P1n=K*P1;
isInFront = {};
X={};
for i=1:4
    X{i}=Triang(xtilde,P1,P_2{i});
    X{i}=pflat(X{i});
    
    %plot reconstructed 3D-points
    figure;
    plot3(X{i}(1,:),X{i}(2,:),X{i}(3,:),'.b','Markersize',2);
    hold on
    plotcams({P1; P_2{i}})
    axis equal
    hold off
    P2n=K*P_2{i};
    pProj2{i}=pflat(P2n*X{i});
    
    
    
    
    
    
    camera_center = null(P_2{i});
    principal_axis = P_2{i}(end,1:3);
    frontindex = [];
    for k = 1:length(X{i})
        Xprime = X{i}(:,k) - camera_center;
        if (((Xprime(1:3)' * principal_axis') > 0) && ((Xprime(1:3)' * [0 0 1]') > 0))
            frontindex = [frontindex i];
        end
        isInFront{i} = frontindex;
    end
end



