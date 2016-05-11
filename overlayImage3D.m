function [outputFrame] = overlayImage(snapshot,reference_features,reference_pts,replacement_image,reference_image,cameraParams,Obj)



%[snapshotRect,newOrigin]=undistortImage(snapshot,cameraParams);
snapshot_gray = rgb2gray(snapshot);
detected_pts = detectSURFFeatures(snapshot_gray);
%nbrOfPoints=detected_pts.Count;
%detected_pts_coord=detected_pts.Location;
Kref=[290/2 0 290/2;0 201/2 201/2; 0 0 1];
Pcell=cell(4);
P=[];



[snapshot_features, snapshot_pts] = extractFeatures(snapshot_gray, detected_pts);


% imshow(snapshot_gray);
% hold on
% plot(snapshot_pts,'showOrientation',true);

index_pairs = matchFeatures(reference_features, snapshot_features);
matched_snapshot_pts = snapshot_pts(index_pairs(:,2));
matched_reference_pts = reference_pts(index_pairs(:,1));
nbrOfPoints=matched_snapshot_pts.Count;
matched_snapshot_pts_coord=matched_snapshot_pts.Location;
matched_reference_pts_coord=matched_reference_pts.Location;
median_pt=median(matched_snapshot_pts_coord);
pts_std=std(matched_snapshot_pts_coord);
i=1;
% figure;
% imagesc(snapshot);
% hold on;
% plot(median_pt(1),median_pt(2),'yo','markersize',20)
% plot(matched_snapshot_pts_coord(:,1),matched_snapshot_pts_coord(:,2),'r*','markersize',20)
% hold off;


%First filter of outliers, can cause matching issues.
% while i<=nbrOfPoints
%     if norm(matched_snapshot_pts_coord(i,:)-median_pt) > 3*norm(pts_std)
%         matched_snapshot_pts_coord(i,:)=[];
%         nbrOfPoints=nbrOfPoints-1;
%     else
%         i=i+1;
%     end
% end
% i=1;

% figure;
% imagesc(snapshot);
% hold on;
% plot(median_pt(1),median_pt(2),'yo','markersize',20)
% plot(matched_snapshot_pts_coord(:,1),matched_snapshot_pts_coord(:,2),'r*','markersize',20)
% hold off

if length(index_pairs) >= 5
    
    
    %     figure; showMatchedFeatures(snapshot_gray,reference_image,matched_snapshot_pts,matched_reference_pts, 'montage');
    %     matched_snapshot_pts=cameraParams.IntrinsicMatrix'\matched_snapshot_pts;
    transform = estimateGeometricTransform(matched_reference_pts, matched_snapshot_pts, 'projective');
    %RT=(cameraParams.IntrinsicMatrix')\(transform.T');
    %[U,Alpha,V]=svd(H);
    K=cameraParams.IntrinsicMatrix';
    input1all=Kref\[matched_reference_pts.Location ones(matched_reference_pts.Count,1)]';
    input2all=K\[matched_snapshot_pts_coord  ones(size(matched_snapshot_pts_coord,1),1)]';
    
    % RANSAC approach
    bestE=[];
    bestEInd=10;
    iterations=100;
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
                currentIndex = j + a - 1;
                E=reshape(Evec(:,a),3,3);
                Ecell{currentIndex}=E;
                %Test epipolar contraint
                for i=1:size(input2all,2)
                    comparator(i)=input2all(:,i)'*E*input1all(:,i);
                    comparator(i)=abs(comparator(i));
                    inliermask(i)= comparator(i)<inlierDiscriminator;
                    %                 inliercounter=length(find(inlierIndex));
                end
                inliers{currentIndex}=find(inliermask);
                inliercounter(currentIndex)=sum(inliermask);
                %comparator=max(input2all'*E*input1all);
                %             if inliercounter>maxinliers;
                %                 bestE=E;
                %                 bestEInd=max(comparator);
                %                 bestIndices=find(inlierIndex);
                %             end
            end
        end
    end
    if nnz(inliercounter) == 0
        locs = matched_snapshot_pts_coord;
        circlePositions = [locs(:,1) locs(:,2) 3*ones(length(locs), 1)];
        outputFrame = insertShape(snapshot, 'Circle', circlePositions);
        disp('Could not compute Essential matrix')
        return
    end
    
    bestRow=find(inliercounter==max(inliercounter));
    bestE=Ecell{bestRow};
    bestIndices=inliers{bestRow};
    if isempty(bestE)==1
        locs = matched_snapshot_pts_coord;
        circlePositions = [locs(:,1) locs(:,2) 3*ones(length(locs), 1)];
        outputFrame = insertShape(snapshot, 'Circle', circlePositions);
        disp('Could not compute Essential matrix')
        return
    end
    
    [U,S,V]=svd(bestE);
    W=[0 -1 0; 1 0 0; 0 0 1];
    Pcell{1}=K*[U*W*V' U(:,3)];
    Pcell{2}=K*[U*W*V' -U(:,3)];
    Pcell{3}=K*[U*W'*V' U(:,3)];
    Pcell{4}=K*[U*W'*V' -U(:,3)];
    for i=1:4
        ProjectedPoints=Pcell{i}*[Obj.vertices'; ones(1,length(Obj.vertices))];
        if any(ProjectedPoints(end,:)<0)==0
            P=Pcell{i};
        end
    end
    
    if isempty(P)==1
        locs = matched_snapshot_pts_coord;
        circlePositions = [locs(:,1) locs(:,2) 3*ones(length(locs), 1)];
        outputFrame = insertShape(snapshot, 'Circle', circlePositions);
        
        disp('Could not compute Camera matrix')
        return
    end
    
    ProjectedPoints=pflat(P*[Obj.vertices'; ones(1,length(Obj.vertices))]);
    %ProjectedPoints=pflat([Obj.vertices ones(length(Obj.vertices),1)]*P);
    face1=Obj.objects(4).data.vertices(1,:);
    face2=Obj.objects(4).data.vertices(2,:);
    face3=Obj.objects(4).data.vertices(3,:);
    face4=Obj.objects(4).data.vertices(4,:);
    
    xtilde={input1all,input2all};
    
    %figure;
    %fill3(ProjectedPoints(1,face1),ProjectedPoints(2,face1),ProjectedPoints(3,face1),'r',ProjectedPoints(1,face2),ProjectedPoints(2,face2),ProjectedPoints(3,face2),'b',ProjectedPoints(1,face3),ProjectedPoints(2,face3),ProjectedPoints(3,face3),'g',ProjectedPoints(1,face4),ProjectedPoints(2,face4),ProjectedPoints(3,face4),'y');
    %fill(ProjectedPoints(1,face1),ProjectedPoints(2,face1),'r',ProjectedPoints(1,face2),ProjectedPoints(2,face2),'b',ProjectedPoints(1,face3),ProjectedPoints(2,face3),'g',ProjectedPoints(1,face4),ProjectedPoints(2,face4),'y');
    
    patches={[ProjectedPoints(1,face1(1)) ProjectedPoints(2,face1(1)) ProjectedPoints(1,face1(2)),ProjectedPoints(2,face1(2)) ProjectedPoints(1,face1(3)) ProjectedPoints(2,face1(3))], ...
        [ProjectedPoints(1,face2(1)) ProjectedPoints(2,face2(1)) ProjectedPoints(1,face2(2)),ProjectedPoints(2,face2(2)) ProjectedPoints(1,face2(3)) ProjectedPoints(2,face2(3))], ...
        [ProjectedPoints(1,face3(1)) ProjectedPoints(2,face3(1)) ProjectedPoints(1,face3(2)),ProjectedPoints(2,face3(2)) ProjectedPoints(1,face3(3)) ProjectedPoints(2,face3(3))], ...
        [ProjectedPoints(1,face4(1)) ProjectedPoints(2,face4(1)) ProjectedPoints(1,face4(2)),ProjectedPoints(2,face4(2)) ProjectedPoints(1,face4(3)) ProjectedPoints(2,face4(3))]};
    
    
    %outputView = imref2d(size(snapshot));
    %warped = imwarp(replacement_image, transform, 'OutputView', outputView);
    
    %alphaBlender = vision.AlphaBlender('Operation', 'Binary mask', 'MaskSource', 'Input port');
    
    %mask = warped(:,:,1) | ...
    %warped(:,:,2) | ...
    % warped(:,:,3) > 0;
    %{'red', 'green', 'blue','yellow' }
    %patches = projectMesh(P,Obj);
    %outputFrame = insertShape(snapshot,'FilledPolygon',patches);
    outputFrame = insertShape(snapshot,'FilledPolygon',patches,'Color',{'red', 'green', 'blue','yellow' }); %step(alphaBlender, snapshot, warped, mask);
    locs = matched_snapshot_pts_coord;
    circlePositions = [locs(:,1) locs(:,2) 3*ones(length(locs), 1)];
    outputFrame = insertShape(outputFrame, 'Circle', circlePositions);
else
    outputFrame = snapshot;
end

end