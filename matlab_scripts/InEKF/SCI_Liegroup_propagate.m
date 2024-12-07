
function [X_est, P_est] = SCI_Liegroup_propagate(X_last, P_last, w_m, a_m, dt, SIGMA_SET)
%% 基于process noise independent 假设，预测时，dep部分不发生改变

X_est=X_last;
P_est=P_last;



Pall=P_last.P;
P_indep=P_last.Pindep;
P_dep=P_last.Pdep;





sigma_a=SIGMA_SET.sigma_a;
sigma_g=SIGMA_SET.sigma_g;
sigma_ba=SIGMA_SET.sigma_ba;
sigma_bg=SIGMA_SET.sigma_bg;

epsQ_1=sigma_g^2;%n_g
epsQ_2=sigma_a^2;%    n_a
epsQ_3=sigma_bg^2;%n_wg
epsQ_4=sigma_ba^2;%n_wa

velocityInCurrentStep=X_last.vimu;
orientationInCurrentStep=X_last.Rimu;
g=[0;0;-9.81];% gravity

X_est.vimu=velocityInCurrentStep+((orientationInCurrentStep*(a_m-X_last.ba)'+g)*dt)';
X_est.Rimu = orientationInCurrentStep*so3_exp((w_m-X_last.bg)*dt);
X_est.pimu = X_last.pimu+(X_est.vimu+velocityInCurrentStep)*dt/2;



P_IMU_last=P_indep(1:9,1:9);
Ad_X=zeros(9,9);
Ad_X(1:3,1:3)=X_est.Rimu;
Ad_X(4:6,1:3)=skew(X_est.pimu')*X_est.Rimu;
Ad_X(7:9,1:3)=skew(X_est.vimu')*X_est.Rimu;
Ad_X(4:6,4:6)=X_est.Rimu;
Ad_X(7:9,7:9)=X_est.Rimu;
Ad_invX=inv(Ad_X);
P_I1=Ad_invX*P_IMU_last*Ad_invX';

A=zeros(9,9);
A(1:3,1:3)=computeA(P_I1(1:3,1:3));
A(4:6,1:3)=computeA(P_I1(4:6,1:3)+P_I1(4:6,1:3)');
A(7:9,1:3)=computeA(P_I1(7:9,1:3)+P_I1(7:7,1:3)');
A(4:6,4:6)=computeA(P_I1(1:3,1:3));
A(7:9,7:9)=computeA(P_I1(1:3,1:3));

sigma_bg=SIGMA_SET.sigma_bg;
sigma_ba=SIGMA_SET.sigma_ba;
sigmaN1=sigma_bg*sigma_bg*eye(3);
sigmaN3=sigma_ba*sigma_ba*eye(3);
sigmaN=zeros(9,9);
sigmaN(1:3,1:3)=sigmaN1;
sigmaN(7:9,7:9)=sigmaN3;

B=zeros(9,9);
B(1:3,1:3)=computeAB(P_I1(1:3,1:3),sigmaN1);
B(4:6,1:3)=(computeAB(P_I1(4:6,1:3),sigmaN1))';
B(7:9,1:3)=(computeAB(P_I1(7:9,1:3),sigmaN1))';
B(1:3,4:6)=computeAB(P_I1(4:6,1:3),sigmaN1);
B(4:6,4:6)=computeAB(P_I1(4:6,4:6),sigmaN1);
B(1:3,7:9)=computeAB(P_I1(7:9,1:3),sigmaN1);
B(7:9,7:9)=computeAB(P_I1(7:9,1:3),sigmaN1)+computeAB(P_I1(1:3,1:3),sigmaN3);

sigma4th_tail=(A*sigmaN+sigmaN*A')/6+B/4; 

Q=blkdiag(epsQ_1*eye(3), epsQ_2*eye(3), epsQ_3*eye(3), epsQ_4*eye(3)); % the cov matrix in state transition

Fn=zeros(9,9);
Gn=zeros(9,12);
Fn(4:6,7:9)=eye(3);
Fn(7:9,1:3)=skew(g);
Fn=0.167*Fn*Fn*Fn*dt*dt*dt+0.5*Fn*Fn*dt*dt+Fn*dt+eye(9); %exp矩阵展开捏

Gn(1:3,1:3)=X_last.Rimu;
Gn(4:6,1:3)=skew(X_last.pimu)*X_last.Rimu;
Gn(7:9,1:3)=skew(X_last.vimu)*X_last.Rimu;
Gn(7:9,4:6)=X_last.Rimu;


P_IMU_temp = Fn *P_indep(1:9,1:9)*Fn' + Gn*Q*Gn'*dt;
P_IMU_temp= P_IMU_temp+Ad_X*sigma4th_tail*Ad_X'*dt; % if 4-th approximation
P_IMU_est=(P_IMU_temp+P_IMU_temp')/2;


Dim = length(Pall);
Fn_aug = eye(Dim,Dim);
Fn_aug(1:9,1:9)=Fn;
Gn_aug = zeros(Dim,12);
Gn_aug(1:9,1:12)=Gn;
P_est_indep = Fn_aug * P_indep *Fn_aug' + Gn_aug * Q * Gn_aug'*dt;
P_est_indep(1:9,1:9)=P_IMU_est;

P_est.Pindep = P_est_indep;
P_est.P=P_est.Pindep+P_est.Pdep;

end


