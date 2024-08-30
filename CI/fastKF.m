function [X_CI,P_CI, w] = fastKF(X_all, P_all)

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
X_tmp = zeros(size(X_all(:,1)));
for i=1:NUM_EST
    w(i,1)=1/NUM_EST;
    inv_cov_i = inv(P_all(:,:,i)); % 6*6
    inv_P_CI=inv_P_CI + w(i,1)*inv_cov_i; 
    X_tmp=X_tmp+w(i,1)*inv_cov_i*X_all(:,i);
    
end


P_CI=inv(inv_P_CI);
X_CI=P_CI*X_tmp;





end