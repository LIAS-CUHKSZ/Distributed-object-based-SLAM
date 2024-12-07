function [C,c] = averageCI(est_xi,P_est_by_j,cov_A,a)
% 待融合的个数
NUM_EST=size(est_xi,2)+1;

weight = 1/NUM_EST;

invP=zeros(size(cov_A));
H=[eye(6), zeros(6,9)];
xi=zeros(15,1);
for i=1:NUM_EST-1
    invP_now=inv(P_est_by_j(:,:,i));
    invP=invP+ weight *H'* invP_now *H ;
    xi=xi+H'*invP_now*weight*est_xi(:,i);
    
end

invP=invP+weight*inv(cov_A);

C=inv(invP);
c=C*xi;

end