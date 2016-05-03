function [faceVector]= projectMesh(P,Object)
projectedPoints=pflat(P*[Obj.vertices'; ones(1,length(Obj.vertices))]);
principalAxis=P(end,1:3);
nbrOfFaces=size(Obj.objects(4).data.vertices,1);
patches=cell(1,nbrOfFaces);
for i=1:nbrOfFaces
    face=Obj.objects(4).data.vertices(i,:);
    faceNormal=sum(Obj.objects(4).data.normals(i,:))/3;
    if principalAxis*faceNormal < 0 %backface culling;
        patches{i}=[projectedPoints(1,face(1)) projectedPoints(2,face(1)) projectedPoints(1,face(2)),projectedPoints(2,face(2)) projectedPoints(1,face(3)) projectedPoints(2,face(3))];
    end
end
faceVector = patches(~cellfun(@isempty, patches));
end