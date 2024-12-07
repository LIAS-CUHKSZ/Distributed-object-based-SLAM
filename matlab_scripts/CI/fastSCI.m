function [X_CI,P_CI, w] = fastSCI(X_all, P_all, P_indep)

NUM_EST=size(X_all,2); % how many columns of the X_all == the number of estimates to be fused
w = zeros(NUM_EST,1);
trP_inv=zeros(NUM_EST,1);
E_sum=0;

%快速计算系数: w=(1/tr(P_i)) /  sum (1/tr(P_m))

for i=1:NUM_EST
    cov_i = P_all(:,:,i);
    trcov_i = trace(cov_i);
    trP_inv(i,1)=1/trcov_i;
    E_sum=E_sum+trP_inv(i,1);  
end


J = zeros(6, 15);
J(1:6,1:6)=eye(6,6);

inv_P_CI = zeros(size(P_all(:,:,1)));
invPindep=zeros(size(P_indep(:,:,1)));
X_tmp = zeros(size(X_all(:,1)));
for i=1:NUM_EST
    w(i,1)=trP_inv(i,1)/E_sum;
    inv_P_CI=inv_P_CI + w(i,1)* inv(P_all(:,:,i)); 
    invPindep=invPindep+w(i,1)*inv( P_indep(:,:,i)); % 因为是独立部分，所以不需要加系数
    X_tmp=X_tmp+w(i,1)* inv(P_all(:,:,i))*X_all(:,i);
    
end


P_CI.Pdep=inv(inv_P_CI);
P_CI.Pindep= inv(invPindep);
P_CI.P=P_CI.Pdep+P_CI.Pindep;
X_CI=P_CI.Pdep*X_tmp;


end