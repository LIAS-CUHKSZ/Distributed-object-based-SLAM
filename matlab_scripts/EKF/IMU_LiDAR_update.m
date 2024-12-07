function [X_next, costf_value, dX, K, S]  = IMU_LiDAR_update(X_est, P_est, lidar_m, dX_last, a_m, w_m, N_lidLm, p_lidlm_set, SIGMA_SET,X_propogated, IF_CALIBRATION)
%%
% 1.Compensate for temporal offset
if(IF_CALIBRATION)
    [X_,P_]=EKF_propagate(X_est, P_est, w_m, a_m,0,SIGMA_SET,IF_CALIBRATION);
    %%
    %2.  Jacobian of H
    residual=zeros(3*N_lidLm,1);
    H_=zeros(3*N_lidLm,29);%the batch measurement Jacobian matrix H=[H1',H2',...Hncam', 0...N_lidarlm...0]
    R=eye(3*N_lidLm);
    sigma_lid=SIGMA_SET.sigma_lid;
    sigma_t=SIGMA_SET.sigma_tl;
    feature_cal=zeros(N_lidLm,3);
    for i=1:N_lidLm
        p_f=p_lidlm_set(i,:);
        pfinL_=(X_.Rimu*X_.Rli)'*(p_lidlm_set(i,:)'-X_.pimu')-X_.Rli'*X_.pli';
        feature_cal(i,:)=pfinL_;
        H_theta=X_.Rli' * skew(X_.Rimu'*(p_f-X_.pimu)');%3*3
        H_p=-X_.Rli' * X_.Rimu';%3*3
        H_phi=skew(X_.Rli' * X_.Rimu' * (p_f-X_.pimu)'-X_.Rli'*X_.pli');%3*3
        H_pl=-X_.Rli';
        H_t=zeros(3,1);%H_theta*X_.Rimu*w_hat'+H_p*X_.vimu';
        Hl_=[H_theta, H_p, zeros(3,15), H_phi, H_pl, zeros(3,1),H_t];    %3×29
        %Hl_=[H_theta, H_p, zeros(3,21), zeros(3,1),H_t]; 
        H_(3*(i-1)+1:3*i,:)=Hl_;%3N×29  
        R(3*(i-1)+1:3*i, 3*(i-1)+1:3*i)=sigma_lid*sigma_lid*eye(3);
    end
    %%
    % 3. Compute the residual
    residual_set=lidar_m-feature_cal;
    for i=1:N_lidLm
        residual((i-1)*3+1:3*i,1)=residual_set(i,:);
    end
    %% 
    % 4. Compute Kalman gain 
    
    H_tc_all=H_(:,29);
    S=H_*P_*H_' + R + sigma_t*sigma_t*(H_tc_all*H_tc_all');
    K=P_*H_'*inv(S);
    %% 
    % 5. 计算迭代修正量 $\Delta X$, 更新$X$
    dX=K*(residual+H_*dX_last);
    deltaR=dX(1:3);
    deltaR1=dX(16:18);
    deltaR2=dX(22:24);
    X_next.Rimu=X_propogated.Rimu*so3_exp(deltaR);
    X_next.pimu=X_propogated.pimu+dX(4:6)';
    X_next.vimu=X_propogated.vimu+dX(7:9)';
    X_next.bg=X_propogated.bg+dX(10:12)';
    X_next.ba=X_propogated.ba+dX(13:15)';
    X_next.Rci=X_propogated.Rci*so3_exp(deltaR1);
    X_next.pci=X_propogated.pci+dX(19:21)';
    X_next.Rli=X_propogated.Rli*so3_exp(deltaR2);
    X_next.pli=X_propogated.pli+dX(25:27)';
    X_next.tc=X_propogated.tc+dX(28);
    X_next.tl=X_propogated.tl+dX(29);
    %% 
    % 6.compute the cost function value
    costf_value=dX'*inv(P_)*dX+residual'*inv(R)*residual;  %cost function value
else
    [X_,P_]=EKF_propagate(X_est, P_est, w_m, a_m,0,SIGMA_SET,IF_CALIBRATION);
    %%
    P_off=P_(1:15,1:15);
    %2.  Jacobian of H
    residual=zeros(3*N_lidLm,1);
    H_=zeros(3*N_lidLm,15);%the batch measurement Jacobian matrix H=[H1',H2',...Hncam', 0...N_lidarlm...0]
    R=eye(3*N_lidLm);
    sigma_lid=SIGMA_SET.sigma_lid;
    sigma_t=SIGMA_SET.sigma_tl;
    feature_cal=zeros(N_lidLm,3);
    for i=1:N_lidLm
        p_f=p_lidlm_set(i,:);
        pfinL_=(X_.Rimu*X_.Rli)'*(p_lidlm_set(i,:)'-X_.pimu')-X_.Rli'*X_.pli';
        feature_cal(i,:)=pfinL_;
        H_theta=X_.Rli' *skew( X_.Rimu'*(p_f-X_.pimu)');%3*3
        H_p=-X_.Rli' * X_.Rimu';%3*3
      %  H_theta=skew(X_.Rimu);%X_.Rli' *skew( X_.Rimu'*(p_f-X_.pimu)');%3*3
      %  H_p=-X_.Rimu';%3*3
        Hl_=[H_theta, H_p, zeros(3,9)]; 
        H_(3*(i-1)+1:3*i,:)=Hl_;%3N×15  
        R(3*(i-1)+1:3*i, 3*(i-1)+1:3*i)=sigma_lid*sigma_lid*eye(3);
    end
    %%
    % 3. Compute the residual
    residual_set=lidar_m-feature_cal;
    for i=1:N_lidLm
        residual((i-1)*3+1:3*i,1)=residual_set(i,:);
    end
    %% 
    % 4. Compute Kalman gain 
    
    H_tc_all=H_(:,15);
    S=H_*P_off*H_' + R + sigma_t*sigma_t*(H_tc_all*H_tc_all');
    K=P_off*H_'*inv(S);
    %% 
    % 5. 计算迭代修正量 $\Delta X$, 更新$X$
    dX_last_off=dX_last(1:15,:);
    dX=K*(residual+H_*dX_last_off);
    deltaR=dX(1:3);
    X_next.Rimu=X_propogated.Rimu*so3_exp(deltaR);
    X_next.pimu=X_propogated.pimu+dX(4:6)';
    X_next.vimu=X_propogated.vimu+dX(7:9)';
    X_next.bg=X_propogated.bg+dX(10:12)';
    X_next.ba=X_propogated.ba+dX(13:15)';
    X_next.Rci=X_propogated.Rci;
    X_next.pci=X_propogated.pci;
    X_next.Rli=X_propogated.Rli;
    X_next.pli=X_propogated.pli;
    X_next.tc=X_propogated.tc;
    X_next.tl=X_propogated.tl;
    %% 
    % 6.compute the cost function value
    costf_value=dX'*inv(P_off)*dX+residual'*inv(R)*residual;  %cost function value
end
end