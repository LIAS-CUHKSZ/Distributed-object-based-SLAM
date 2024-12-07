function [C,c] = weightedCI(est_xi,P_est_by_j,cov_A,a)
% 待融合的个数
NUM_EST=size(est_xi,2)+1;

weight=[];
tracesum = 1/trace(cov_A);
for i=1:NUM_EST-1
    tracesum=tracesum + 1/(trace(P_est_by_j(:,:,i)))   +     1/trace(cov_A(7:9,7:9));
end



invP=zeros(size(cov_A));
H=[eye(6), zeros(6,9)];
xi=zeros(15,1);
for i=1:NUM_EST-1
    weight(i)=(1/(trace(P_est_by_j(:,:,i)))   +     1/trace(cov_A(7:9,7:9)))/tracesum;
    invP_now=inv(P_est_by_j(:,:,i));
    invP=invP+  weight(i) *H'* invP_now *H ;
    xi=xi+H'*invP_now* weight(i)*est_xi(:,i);
    
end

weight(NUM_EST)=(1/trace(cov_A))/tracesum;

invP=invP+weight(i+1)*inv(cov_A);


C=inv(invP);
c=C*xi;

end