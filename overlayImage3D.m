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
    [bestRow,bestCol]=find(inliercounter==max(max(inliercounter)),1);
    bestE=Ecell{bestRow,bestCol};
    bestIndices=inliers{bestRow,bestCol};
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
    
    %H=transform.T';
    %     camMotion=struct(['R','n','t'],{zeros(3),zeros(3,1),zeros(3,1)});
    
    
    %     [K,R,t]=cv.decomposeProjectionMatrix(H);
    %     [U,A,V]=svd(H)
    %     d1=A(1,1);
    %     d2=A(2,2);
    %     d3=A(3,3);
    %
    %     s=det(U)*det(V);
    %     d1prim=d1/s;
    %     d2prim=d2/s;
    %     d3prim=d3/s;
    %
    %     x2=0;
    %
    %
    %
    % %     Sr=H*H'-eye(3);
    %
    %     sr=H*H'-eye(3);
    %      M=zeros(3);
    %     for i=1:3
    %         for j=1:3
    %             M(i,j)=-det(sr([1:i-1,i+1:end],[1:j-1,j+1:end]));
    %         end
    %     end
    %
    %     taprime=[sr(1,2)+sqrt(M(3,3)); sr(2,2); sr(2,3)-sign(sr(1,3))];
    %     tbprime=[sr(1,2)-sqrt(M(3,3)); sr(2,2); sr(2,3)+sign(sr(1,3))];
    %
    %     nu=2*sqrt(1+trace(sr)-trace(M));
    %     te=2+trace(sr)-nu;
    %
    %     ta=te*taprime/norm(taprime);
    %     tb=te*taprime/norm(tbprime);
    %     rho=sqrt(te^2+2*nu);
    %
    %     naprime=1/2sign(s(1,1))*rho/te*tb;
    %
    %     RaT=H'*(eye(3)-2/nu*naprime*ta');
    
    %     h1=H(1,1:3);
    %     h2=H(2,1:3);
    %     h3=H(3,1:3);
    
    
    
    
    %na11prime=[s(1,1);s(1,2)+sqrt(M(3,3));s(1,3)+sign(M(2,3))*sqrt(M(2,2))];
    %nb11prime=[s(1,1);s(1,2)-sqrt(M(3,3));s(1,3)-sign(M(2,3))*sqrt(M(2,2))];
    
    %na33prime=[s(1,3)+sign(M(1,2))*sqrt(M(2,2));s(2,3)+sqrt(M(1,1));s(3,3)];
    %nb33prime=[s(1,3)-sign(M(1,2))*sqrt(M(2,2));s(2,3)-sqrt(M(1,1));s(3,3)];
    
    %na11=na11prime/(norm(na11prime));
    %nb11=nb11prime/(norm(nb11prime));
    
    %na33=na33prime/(norm(na33prime));
    %nb33=nb33prime/(norm(nb33prime));
    % %
    %     nu=2*sqrt(1+trace(s)-M(1,1)-M(2,2)-M(3,3));
    %     normtesq=2+trace(s)-nu;
    %     rho=sqrt(2+trace(s)+nu);
    %
    
    %ta11ast=norm(na11prime)/(2*s(1,1))*nb11prime-normtesq/(2*norm(na11prime))*na11prime;
    %tb11ast=norm(nb11prime)/(2*s(1,1))*na11prime-normtesq/(2*norm(nb11prime))*nb11prime;
    
    %ta33ast=norm(na33prime)/(2*s(1,1))*nb33prime-normtesq/(2*norm(na33prime))*na33prime;
    %tb33ast=norm(nb33prime)/(2*s(1,1))*na33prime-normtesq/(2*norm(nb33prime))*nb33prime;
    
    %     Ra=H*(eye(3)-2/nu*ta11ast*na11');
    %     Rb=H*(eye(3)-2/nu*tb11ast*nb11');
    
    %Ra=H*(eye(3)-2/nu*ta33ast*na33');
    %Rb=H*(eye(3)-2/nu*tb33ast*nb33');
    
    %     ta=Ra*ta11ast;
    %     tb=Rb*tb11ast;
    
    %ta=Ra*ta33ast;
    %     %tb=Rb*tb33ast;
    % %
    %     ta11prime=[s(1,1);s(1,2)+sqrt(M(3,3));s(1,3)+sign(M(2,3))*sqrt(M(2,2))];
    %     tb11prime=[s(1,1);s(1,2)-sqrt(M(3,3));s(1,3)-sign(M(2,3))*sqrt(M(2,2))];
    % %
    %     ta33prime=[s(1,3)+sign(M(1,2))*sqrt(M(2,2));s(2,3)+sqrt(M(1,1));s(3,3)];
    %     tb33prime=[s(1,3)-sign(M(1,2))*sqrt(M(2,2));s(2,3)-sqrt(M(1,1));s(3,3)];
    % %
    %     ta11=sqrt(normtesq)*ta11prime/norm(ta11prime);
    %     tb11=sqrt(normtesq)*tb11prime/norm(tb11prime);
    %
    %     ta33=sqrt(normtesq)*ta33prime/norm(ta33prime);
    %     tb33=sqrt(normtesq)*tb33prime/norm(tb33prime);
    % %
    % %
    %     na11prime=(1/2)*sign(s(1,1)*rho/sqrt(normtesq)*tb11-ta11);
    %     nb11prime=(1/2)*sign(s(1,1)*rho/sqrt(normtesq)*ta11-tb11);
    % %
    %     na33prime=(1/2)*sign(s(1,1)*rho/sqrt(normtesq)*tb33-ta33);
    %     nb33prime=(1/2)*sign(s(1,1)*rho/sqrt(normtesq)*ta33-tb33);
    % %
    %     Ra=(eye(3)-2/nu*ta33*na33prime');
    %     Rb=(eye(3)-2/nu*tb11*nb11prime');
    %
    %     P=cameraParams.IntrinsicMatrix'*[RaT' ta];
    %     P=real(P);
    %P=cameraParams.IntrinsicMatrix'*H*[eye(3) ones(3,1)];
    %     P=cameraParams.IntrinsicMatrix'*[RaT' ta];
    %P=P./P(end);
    %F=estimateFundamentalMatrix(matched_snapshot_pts,matched_reference_pts);
    %inliers1=matched_snapshot_pts;%matched_snapshot_pts(epinliers,:);
    %inliers2=matched_reference_pts;%matched_reference_pts(epinliers,:);
    %[R,t]= cameraPose(F,cameraParams,inliers1,inliers2);
    
    %P=cameraMatrix(cameraParams,R,t');%cameraParams.IntrinsicMatrix'*[R t'];
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