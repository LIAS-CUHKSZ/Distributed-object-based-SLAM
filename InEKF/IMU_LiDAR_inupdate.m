function [X_next, costf_value,dX,K,H_time] = IMU_LiDAR_inupdate(X_last,P_last, lidar_m, Delta, N_lidLm, p_lidlm_set, sigma_if, X_est)
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
    

    % 1.Compensate for temporal offset        
    deltat=0;
   % prepropagate
%     [X_time,P_time]=inEKF_propagate(X_last, P_last, w_m,  a_m, deltat,SIGMA_SET, IF_IEKF_update_bgba);  
   % if no pre propagate
    P_time=P_last;
    X_time = X_last;
    X_next=X_time;
    %% 
    % 2. Jacobian of H
    
    delta_X=Delta;
    residual=zeros(3*N_lidLm,1);
    feature_obs=zeros(N_lidLm,3);
    H_time=zeros(3*N_lidLm,15);%the batch measurement Jacobian matrix H=[H1',H2',...Hncam', 0...N_lidarlm...0]
    R=eye(3*N_lidLm);
  
    sigma_lid=sigma_if;
    for i=1:N_lidLm
        pfinL_time=(X_time.Rimu)'*(p_lidlm_set(i,:)'-X_time.pimu');  
        feature_obs(i,:)=pfinL_time';
        H_theta= (X_time.Rimu)'*skew(p_lidlm_set(i,:)');
        H_p=-X_time.Rimu';
        
        %H_phi=X_time.Rl'*skew(p_lidlm_set(i,:)'); %rotation
        %H_pl=-X_time.Rl';  %position
        %Hl_time=[H_phi, H_pl,zeros(3,9)] ;  %3×15
        Hl_time=[H_theta, H_p,zeros(3,9)] ;  %3×15
        H_time(3*(i-1)+1:3*i,:)=Hl_time;  %3N×15
        R(3*(i-1)+1:3*i, 3*(i-1)+1:3*i)=sigma_lid*sigma_lid*eye(3);
    end
    %%
    % 3. Compute the residual
    residual_set=lidar_m-feature_obs;
    for i=1:N_lidLm
        residual((i-1)*3+1:3*i,1)=residual_set(i,:);
    end
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

    
    
    %%  
    % 7. Compute the cost function value
    invP=inv(P_time_com);
    costf_value=dX'*invP*dX +residual'*inv(R)*residual; %cost function value  %%%%这里注意下






end