function [P_aug] = Covariance_aug_one(P)
%UNTITLED 复制前六维到新的协方差的最后六维度
%   


Dim=length(P);

J= zeros(6,9);
J(1:6,1:6)=eye(6);

Ja=zeros(Dim+6, Dim);
Ja(1:Dim,1:Dim)=eye(Dim);

Ja(Dim+1:Dim+6, 1:9)=J;

P_aug=Ja*P*Ja';

end