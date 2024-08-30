function [X_next, costf_value,dX,K,H_time] = IMU_LiDAR_plane_inupdate(X_last,P_last, lidar_m, Delta, N_lidLm, normal_vector, points_in_plane, sigma_if,X_est)
% IMU_LIDAR_INUPDATE: update the state estimation by invariant EKF
% Input: X_last - if no iteration: estimation after propagation; if iteration: the state of last iteration
%        P_last - covariance after propagation
%        lidar_m - lidar measurement
%        Delta  - delta(l) in algorithm 1
%        a_m, w_m - imu measurement
%        N_lidLM - number of lidar landmarks
%        SIGMA_SET - noise parameters
%        X_est - state after propagation
%        IF_IEKF_update_bgba - to determine if update ba and bg
% Output:X_next - state for next iteration or updated state
%        costf_value - cost function value
%        dX - correction this iteration
%        K - Kalman gain this iteration
%        H_time - Jacobian this iteration
    
deltat=0;
delta_X=Delta;
% prepropagate
%     [X_time,P_time]=inEKF_propagate(X_last, P_last, w_m,  a_m, deltat,SIGMA_SET, IF_IEKF_update_bgba);  
% if no pre propagate
P_time=P_last;
X_time = X_last;
X_next=X_time;

%% 
%2&3. residual & Jacobian 
sigma_lid=sigma_if;
H_time=zeros(N_lidLm,15);
residual=zeros(N_lidLm,1);
for i = 1:N_lidLm
    
    p_f_est =( X_time.Rl * lidar_m(i,:)' + X_time.pl'  )' ; %1*3
    resi_now=normal_vector(i,:) * ( p_f_est - points_in_plane(i,:))';
    residual(i,1)=resi_now;
    
    H_theta=-skew(X_time.Rl *lidar_m(i,:)' ) -skew( X_time.Rl  )- skew(X_time.pimu' );
    H_p=eye(3);
    H_tmp=[H_theta, H_p, zeros(3,9)];
    H_one=normal_vector(i,:) * H_tmp;
    H_time(i,:) = H_one;
end

H_time=-H_time;
R=sigma_lid*sigma_lid* eye(N_lidLm);

%% 
% 4. Compute Kalman gain 
J_inv=eye(length(P_time));
% J_inv=jaco_left_sum(delta_X);%compensating the covariance %%%%%%%%Jinv
J=inv(J_inv);
P_time_com = J_inv*P_time*J_inv';
S=H_time*J_inv*P_time*J_inv'*H_time' + R;   %covariance of residual
K=P_time*J_inv'*H_time'*inv(S);  %Kalman gain
%% 
% 5. Compute the correction $\Delta X$
dX=K*(residual+H_time*delta_X);%correction

%% 
% 6. Update X 
dR=dX(1:3);
dP=dX(4:6);
dV=dX(7:9);
exp_dX1=[so3_exp(dR),jaco_left(dR)*dP,jaco_left(dR)*dV;zeros(2,3),eye(2)];%5*5

update_temp_IMU=exp_dX1*[X_est.Rimu, X_est.pimu', X_est.vimu';         zeros(2,3),eye(2,2)];

X_next.Rimu=update_temp_IMU(1:3,1:3);
X_next.pimu=update_temp_IMU(1:3,4)';
X_next.vimu=update_temp_IMU(1:3,5)';
X_next.bg=X_time.bg+dX(10:12)';
X_next.ba=X_time.ba+dX(13:15)';
X_next.Rl=X_next.Rimu*X_next.Rli;
X_next.pl=(X_next.Rimu*X_next.pli')'+X_next.pimu;


%%  
% 7. Compute the cost function value
invP=inv(P_time_com);
costf_value=dX'*invP*dX +residual'*inv(R)*residual; %cost function value  


end