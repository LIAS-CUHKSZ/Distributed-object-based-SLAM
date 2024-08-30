function [X, costf_value,dX,K,S] = IMU_camera_update(X_est,P_est, camera_m, dX_last, a_m, w_m, tc_last,SIGMA_SET,X_propogated, fx, fy, cx, cy, IF_CALIBRTAION)

if(IF_CALIBRTAION)
    %%
    % 1.Compensate for temporal offset
    deltat=X_est.tc-tc_last;
    [X_,P_]=EKF_propagate(X_est, P_est, w_m,  a_m, deltat, SIGMA_SET,IF_CALIBRTAION);
    
    %%
    %2.  Jacobian of H
    N_valid_camLm=length(camera_m.uv);
    residual=zeros(2*N_valid_camLm,1);
    H_=zeros(2*N_valid_camLm,29);%the batch measurement Jacobian matrix H=[H1',H2',...Hncam', 0...N_lidarlm...0]
    R=eye(2*N_valid_camLm);
    sigma_t=SIGMA_SET.sigma_tc;
    sigma_cam=SIGMA_SET.sigma_cam;
    feature_cal=zeros(N_valid_camLm,2);
    p_f_set=camera_m.landmark_G;
    for i=1:N_valid_camLm
      p_f=p_f_set(i,:);  
      pfinC_=(X_.Rimu*X_.Rci)'*(p_f'-X_.pimu') - X_.Rci'*X_.pci';
      xfinC_=pfinC_(1);yfinC_=pfinC_(2);zfinC_=pfinC_(3);
      feature_cal(i,:)=[fx*xfinC_/zfinC_+cx;fy*yfinC_/zfinC_+cy];
      Ji=(1/zfinC_)*[fx,0,fx*(-xfinC_/zfinC_);0,fy,fy*(-yfinC_/zfinC_)]; 
      H_RI=Ji * X_.Rci' * skew(X_.Rimu'*(p_f-X_.pimu)');
      H_pI=- Ji*  X_.Rci' * X_.Rimu';
      H_RCI=Ji*skew(X_.Rci' * X_.Rimu' * (p_f-X_.pimu)'-X_.Rci'*X_.pci');
      H_pCI =-Ji*X_.Rci';
      HC_=[H_RI, H_pI, zeros(2,9),H_RCI,H_pCI,zeros(2,6),zeros(2,1), zeros(2,1)] ; 
      H_(2*(i-1)+1:2*i,:)=HC_;%3N×29  
      R(2*(i-1)+1:2*i,2*(i-1)+1:2*i)=sigma_cam*sigma_cam*eye(2);
    end
    %%
    % 3. Compute the residual
    residual_set=camera_m.uv-feature_cal;
    for i=1:N_valid_camLm
        residual((i-1)*2+1:2*i,1)=residual_set(i,:);
    end
    %% 
    % 4. Compute Kalman gain 
    H_tc_all=H_(:,29);
    S=H_*P_*H_' + R;
    K=P_*H_'*inv(S);  %
    
    %% 
    % 5. 计算迭代修正量 $\Delta X$, 更新$X$
    dX=K*(residual+H_*dX_last);
    deltaR=dX(1:3);
    deltaR1=dX(16:18);
    deltaR2=dX(22:24);
    
    X.Rimu=X_propogated.Rimu*so3_exp(deltaR);
    X.pimu=X_propogated.pimu+dX(4:6)';
    X.vimu=X_propogated.vimu+dX(7:9)';
    X.bg=X_propogated.bg+dX(10:12)';
    X.ba=X_propogated.ba+dX(13:15)';
    X.Rci=X_propogated.Rci*so3_exp(deltaR1);
    X.pci=X_propogated.pci+dX(19:21)';
    X.Rli=X_propogated.Rli*so3_exp(deltaR2);
    X.pli=X_propogated.pli+dX(25:27)';
    X.tc=X_propogated.tc+dX(28);
    X.tl=X_propogated.tl+dX(29);
    %% 
    % 6.compute the cost function value
    costf_value=dX'*inv(P_)*dX+residual'*inv(R)*residual;  %cost function value

else
    %%
    % 1.Compensate for temporal offset
    deltat=X_est.tc-tc_last;
    [X_,P_]=EKF_propagate(X_est, P_est, w_m,  a_m, deltat, SIGMA_SET,IF_CALIBRTAION);
    
    P_off=zeros(15,15);
    P_off=P_(1:15, 1:15);
    %%
    %2.  Jacobian of H
    N_valid_camLm=length(camera_m.uv);
    residual=zeros(2*N_valid_camLm,1);
    H_=zeros(2*N_valid_camLm,15);%the batch measurement Jacobian matrix H=[H1',H2',...Hncam', 0...N_lidarlm...0]
    R=eye(2*N_valid_camLm);
    sigma_t=SIGMA_SET.sigma_tc;
    sigma_cam=SIGMA_SET.sigma_cam;
    feature_cal=zeros(N_valid_camLm,2);
    p_f_set=camera_m.landmark_G;
    for i=1:N_valid_camLm
      p_f=p_f_set(i,:);  
      pfinC_=(X_.Rimu*X_.Rci)'*(p_f'-X_.pimu') - X_.Rci'*X_.pci';
      xfinC_=pfinC_(1);yfinC_=pfinC_(2);zfinC_=pfinC_(3);
      feature_cal(i,:)=[fx*xfinC_/zfinC_+cx;fy*yfinC_/zfinC_+cy];
      Ji=(1/zfinC_)*[fx,0,fx*(-xfinC_/zfinC_);0,fy,fy*(-yfinC_/zfinC_)]; 
      H_RI=Ji * X_.Rci' * skew(X_.Rimu'*(p_f-X_.pimu)');
      H_pI=- Ji*  X_.Rci' * X_.Rimu';
      HC_=[H_RI, H_pI, zeros(2,9)];  %2 * 15   
      H_(2*(i-1)+1:2*i,:)=HC_;%3N×15  
      R(2*(i-1)+1:2*i,2*(i-1)+1:2*i)=sigma_cam*sigma_cam*eye(2);
    end
    %%
    % 3. Compute the residual
    residual_set=camera_m.uv-feature_cal;
    for i=1:N_valid_camLm
        residual((i-1)*2+1:2*i,1)=residual_set(i,:);
    end
    %% 
    % 4. Compute Kalman gain 
    H_tc_all=H_(:,15);
    
    S=H_*P_off*H_' + R;
    K=P_off*H_'*inv(S);  %
    
    %% 
    % 5. 计算迭代修正量 $\Delta X$, 更新$X$
    dX_last_off=dX_last(1:15,:);
    dX=K*(residual+H_*dX_last_off);
    deltaR=dX(1:3);
    X.Rimu=X_propogated.Rimu*so3_exp(deltaR);
    X.pimu=X_propogated.pimu+dX(4:6)';
    X.vimu=X_propogated.vimu+dX(7:9)';
    X.bg=X_propogated.bg+dX(10:12)';
    X.ba=X_propogated.ba+dX(13:15)';
    X.Rci=X_propogated.Rci;
    X.pci=X_propogated.pci;
    X.Rli=X_propogated.Rli;
    X.pli=X_propogated.pli;
    X.tc=X_propogated.tc;
    X.tl=X_propogated.tl;
    %% 
    % 6.compute the cost function value
    costf_value=dX'*inv(P_off)*dX+residual'*inv(R)*residual;  %cost function value

end
end