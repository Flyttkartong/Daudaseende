function [ noHomo ] = pflat( Homo )
    for i=1:size(Homo,2)
        noHomo(:,i)=Homo(:,i)./Homo(end,i); 
    end
end

