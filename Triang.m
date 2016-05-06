function [ X ] = Triang( x, P1, P2)
X=[];
for i=1:length(x{1})
    M=[P1 -x{1}(:,i) [0 0 0]' ; P2 [0 0 0]' -x{2}(:,i)];
    [U,S,V]=svd(M);
    v=V(:,end);
    X=[X v(1:4,:)];
end
end

