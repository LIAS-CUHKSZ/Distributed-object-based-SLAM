function [X_next, costf_value,dX, K, H_aug, X_obj_up] = IMU_camera_inupdate(X_last,P_last, camera_m, dX_last, a_m, w_m,SIGMA_SET, X_est, CAM_INTRISIC, IF_IEKF_update_bgba, pci,Rci, X_obj)
%% IMU_CAMERA_INUPDATE Compute invariant EKF update for Visaul Inertial Odometry
   X_obj_up=X_obj;
if(IF_IEKF_update_bgba) % if update bg and ba

    %% 1.Compensate for temporal offset 
    %if propagate
%     deltat=X_last.tc-tc_last;
%     [X_time,P_time]=inEKF_propagate(X_last, P_last, w_m,  a_m, deltat,SIGMA_SET, IF_IEKF_update_bgba);
    %if no propagate
    X_time=X_last;
    P_time=P_last;

    X_next=X_time;
    %% 2. Jacobian of H
    fx=CAM_INTRISIC.fx;
    fy=CAM_INTRISIC.fy;
    cx=CAM_INTRISIC.cx;
    cy=CAM_INTRISIC.cy;
    N_valid_camLm=length(camera_m.uv);
    delta_X=dX_last;
    residual=zeros(2*N_valid_camLm,1);
    feature_cal=zeros(N_valid_camLm,2);
    H_time=zeros(2*N_valid_camLm,15);%the batch measurement Jacobian matrix H=[H1',H2',...Hncam', 0...N_camlm...0]
    R=eye(2*N_valid_camLm);
    sigma_cam=SIGMA_SET.sigma_cam;
    landmarks=camera_m.landmark_G;
    %% 3. Compute the residual 
    for i=1:N_valid_camLm
        Rc=X_time.Rimu*Rci;
        pc=(X_time.Rimu*pci'+X_time.pimu')';
        p_f=landmarks(i,:);
        %landmark's position in camera frame now.
        pfinC_time=(Rc)'*(p_f'-pc');
        xfinC_=pfinC_time(1);yfinC_=pfinC_time(2);zfinC_=pfinC_time(3);
        feature_cal(i,:)=[fx*xfinC_/zfinC_+cx;fy*yfinC_/zfinC_+cy];
        %Jacobian of the projection model
        Jcam=(1/zfinC_)*[fx,0,fx*(-xfinC_/zfinC_);0,fy, fy*(-yfinC_/zfinC_)];

     
        H_R=Rc'*skew(p_f'); 
        H_p=-Rc';
        Hc_time=Jcam * [H_R, H_p,zeros(3,9)] ;  %2×15
        %Stack togethor
        H_time(2*(i-1)+1:2*i,:)=Hc_time;%2N×15  
        R(2*(i-1)+1:2*i,2*(i-1)+1:2*i)=sigma_cam*sigma_cam*eye(2);
    end
    residual_set=camera_m.uv-feature_cal;
    for i=1:N_valid_camLm
        residual((i-1)*2+1:2*i,1)=residual_set(i,:);
    end
    %% 4. Compute the Kalman gain
    % 
    % J_inv=eye(length(P_time));
    J_inv=jaco_left_sum(delta_X);%compensating the covariance %%%%%%%%Jinv
    P_time_com = J_inv*P_time*J_inv';
    S=H_time*J_inv*P_time*J_inv'*H_time' + R;   %covariance of residual
    K=P_time*J_inv'*H_time'*inv(S);  %Kalman gain
    %% 5. Compute the correction $\Delta X$
    % 
    dX=K*(residual+H_time*delta_X);%correction
    %% 6. Update X
    dR=dX(1:3);
    dP=dX(4:6);
    dV=dX(7:9);
    exp_dX1=[so3_exp(dR),jaco_left(dR)*dP,jaco_left(dR)*dV;zeros(2,3),eye(2)];%5*5
    update_temp_IMU=exp_dX1*[X_est.Rimu, X_est.pimu', X_est.vimu';  zeros(2,3),eye(2,2)];
    
    X_next.Rimu=update_temp_IMU(1:3,1:3);
    X_next.pimu=update_temp_IMU(1:3,4)';
    X_next.vimu=update_temp_IMU(1:3,5)';
    X_next.bg=X_time.bg+dX(10:12)';
    X_next.ba=X_time.ba+dX(13:15)';
    
    
    %% 7. Compute the cost function value
    invP=inv(P_time_com);
    costf_value=dX'*invP*dX +residual'*inv(R)*residual; %cost function value  %%%%这里注意下


else%  if don't update ba and bg
    X_time=X_last;
    P_time=P_last;

    X_next=X_time;
    %% 2. Jacobian of H
    fx=CAM_INTRISIC.fx;
    fy=CAM_INTRISIC.fy;
    cx=CAM_INTRISIC.cx;
    cy=CAM_INTRISIC.cy;
    N_valid_camLm=size(camera_m.uv,1);
    delta_X=dX_last;
    residual=zeros(2*N_valid_camLm,1);
    feature_cal=zeros(N_valid_camLm,2);
    H_time=zeros(2*N_valid_camLm,9);%the batch measurement Jacobian matrix H=[H1',H2',...Hncam', 0...N_camlm...0]
    R=eye(2*N_valid_camLm);
    sigma_cam=SIGMA_SET.sigma_cam;
    landmarks=camera_m.landmark_G;
    %% 3. Compute the residual 
    for i=1:N_valid_camLm
        Rc=X_time.Rimu*Rci;
        pc=(X_time.Rimu*pci'+X_time.pimu')';
        p_f=landmarks(i,:);
        %landmark's position in camera frame now.
        pfinC_time=(Rc)'*(p_f'-pc');
        xfinC_=pfinC_time(1);yfinC_=pfinC_time(2);zfinC_=pfinC_time(3);
        feature_cal(i,:)=[fx*xfinC_/zfinC_+cx;fy*yfinC_/zfinC_+cy];
        %Jacobian of the projection model
        Jcam=(1/zfinC_)*[fx,0,fx*(-xfinC_/zfinC_);0,fy, fy*(-yfinC_/zfinC_)];
        H_R=Rc'*skew(p_f'); 
        H_p=-Rc';
        Hc_time=Jcam * [H_R, H_p,zeros(3,3)] ;  %2×15
        %Stack togethor
        H_time(2*(i-1)+1:2*i,:)=Hc_time;%2N×15  
        R(2*(i-1)+1:2*i,2*(i-1)+1:2*i)=sigma_cam*sigma_cam*eye(2);
    end
    residual_set=camera_m.uv-feature_cal;
    for i=1:N_valid_camLm
        residual((i-1)*2+1:2*i,1)=residual_set(i,:);
    end
    %% 4. Compute the Kalman gain
   
    if(length(P_last)>9)
        H_aug = [H_time, zeros(2*N_valid_camLm,length(P_last)-9)];
    else
        H_aug=H_time;
    end

    J_inv=eye(length(P_time));
    P_time_com = J_inv*P_time*J_inv';
    S=H_aug*J_inv*P_time*J_inv'*H_aug' + R;   %covariance of residual
    


    K=P_time*J_inv'*H_aug'*inv(S);  %Kalman gain
    %% 5. Compute the correction $\Delta X$
    % 
    % dX=K*(residual+H_time*delta_X);%correction
    dX=K*residual;

    %% 6. Update X
    dR=dX(1:3);
    dP=dX(4:6);
    dV=dX(7:9);
    
    exp_dX1=[so3_exp(dR),jaco_left(dR)*dP,jaco_left(dR)*dV;zeros(2,3),eye(2)];%5*5
    update_temp_IMU=exp_dX1*[X_est.Rimu, X_est.pimu', X_est.vimu';         zeros(2,3),eye(2,2)];
    
    X_next.Rimu=update_temp_IMU(1:3,1:3);
    X_next.pimu=update_temp_IMU(1:3,4)';
    X_next.vimu=update_temp_IMU(1:3,5)';
    
    num_est=length(X_obj);

 if(num_est)
    for mm=1:num_est

    dT = dX(9+(mm-1)* 6+1: 9+(mm-1)* 6+6  );

    exp_dXo  =  se3_exp(dT);
    X_obj_up(mm).T = exp_dXo*X_obj(mm).T ;

    end
 end
    %% 7. Compute the cost function value
    invP=inv(P_time_com);
    costf_value=dX'*invP*dX +residual'*inv(R)*residual; %cost function value  

end

end