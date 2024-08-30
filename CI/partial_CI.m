function [X_CI,P_CI] = partial_CI(X_all,P_all,xi_prior,P_prior)

NUM_Neighbor=size(X_all,2); % 有多少个观测得来的估计
w = zeros(NUM_Neighbor+1,1); %加1是为了把先验的估计也一起融合进来
trP_inv=zeros(NUM_Neighbor+1,1);
E_sum=0;

%快速计算系数: w=(1/tr(P_i)) /  sum (1/tr(P_m))

for i=1:NUM_Neighbor
    cov_i = P_all(:,:,i);
    trcov_i = trace(cov_i);
    trP_inv(i,1)=1/trcov_i;
    E_sum=E_sum+trP_inv(i,1);  
end
trace_of_prior=trace(P_prior);
trP_inv(i+1,1)=1/trace_of_prior;
E_sum=E_sum+trP_inv(i+1,1);

J = zeros(6, 15);
J(1:6,1:6)=eye(6,6);

weightlist=zeros(NUM_Neighbor+1,1);

for i=1:NUM_Neighbor
    weightlist(i)=trP_inv(i,1)/E_sum;
end
weightlist(i+1)=trP_inv(i+1,1)/E_sum



inverse_P=J*weightlist(i+1)*inv(P_prior)*J'; % 6*6维

for i=1:NUM_Neighbor
    inverse_P=weightlist(i)*inv(P_all(:,:,i))+inverse_P;
end

P_CI=inv(inverse_P);

X_tmp = xi_prior(1:6,1);
for i=1:NUM_Neighbor
    inv_cov_i = inv(P_all(:,:,i)); % 6*6
    X_tmp=X_tmp+weightlist(i)*inv_cov_i*X_all(:,i);
end

X_CI=P_CI*X_tmp;


end