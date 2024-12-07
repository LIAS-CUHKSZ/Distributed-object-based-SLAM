function [ad] = adjointx(xi)
%ADJOINTX 6*1->6*6

w = xi(1:3,1);
t=xi(4:6,1);
ad=zeros(6,6);
ad(1:3,1:3)=skew(w);
ad(4:6,1:3)=skew(t);
ad(4:6,4:6)=skew(w);



end

