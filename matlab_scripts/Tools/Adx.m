function [AdT] = Adx(T)


C=T(1:3,1:3);
r=T(1:3,4);

AdT=zeros(6,6);
AdT(1:3,1:3)=C;
AdT(4:6,1:3)=skew(r)*C;
AdT(4:6,4:6)=C;



end

