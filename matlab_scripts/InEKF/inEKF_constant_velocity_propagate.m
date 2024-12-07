function [X_pred, P_pred]=  inEKF_constant_velocity_propagate(X_last, P_last, constant_v, w_u, dti)
%% 基于匀速假设，用来预测
% X_last_i,P_last_i: The estimate from the last step
% constant_v: the value of constant velocity
% w_u: the noise of the control input 
% dti: interval time

X_pred=X_last;
P_pred=P_last;
%% Propagate SE(3) state
Delta_T=eye(4);
Delta_T(1:3,4) = constant_v*dti;
T_last=eye(4);
T_last(1:3,1:3) =X_last.Rimu; 
T_last(1:3,4)  = X_last.pimu';

T_pred = T_last * Delta_T;

X_est.Rimu  =  T_pred(1:3, 1:3);
X_est.pimu  = T_pred(1:3,4)';

%% Propagate the covariance
Q = w_u*w_u * eye(6);
P_imu = P_last(1:6,1:6);
M = Adx( T_last );

P_imu_pred= P_imu + M * Q*M';

P_pred(1:6,1:6) = P_imu_pred;


end

