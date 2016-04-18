function [M] = createM(xtilde)
M = zeros(length(xtilde{1}),9);
    for i = 1:length(xtilde{1}) 
        xx=xtilde{2}(:,i)*xtilde{1}(:,i)';
        M(i,:)=xx(:)';       
    end
end