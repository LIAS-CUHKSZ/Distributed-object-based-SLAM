function [xi_now,C,X_kk]=SCI_Liegroup(T_nei,P_nei,H,X_i, A)
%% 迭代去做
A_i=A.Pindep;
A_d  =A.Pdep;

B_d=P_nei;%将B视作只有具有相关性部分
omega=0.999998;

Sigma1=A_i+1/omega*A_d;
Sigma2=1/(1-omega)*B_d;


itermax=3;
X_kk = X_i;
dim_all=length(A_i);

for m=1:1
    iSigma1=inv(Sigma1);
    iSigma2=inv(Sigma2);
    
    
    xi1=se3_log(X_kk*invT(X_i));
    xi2=se3_log(X_kk*invT( T_nei));
    iJ1=inv_J_left(xi1);
    iJ2=inv_J_left(xi2);

    iJ1_all=eye(dim_all);
    iJ1_all(1:6,1:6)=iJ1;

    xi1_all =zeros(dim_all,1);
    xi1_all(1:6,1)=xi1;
  
    
    Sigma_kk = inv( iJ1_all' * iSigma1*iJ1_all+ H*iJ2'*iSigma2*iJ2*H');
    
    xi_now=-Sigma_kk*( iJ1_all' * iSigma1*xi1_all+H*iJ2'*iSigma2*xi2  );
    X_kk=se3_exp(xi_now)*X_kk;

end

Sigma_kki=Sigma_kk*(iJ1_all'*iSigma1* A_i*iSigma1*iJ1_all)*Sigma_kk;
Sigma_kkd=Sigma_kk-Sigma_kki;
C.Pindep=Sigma_kki;
C.P=Sigma_kk;
C.Pdep=Sigma_kkd;

end