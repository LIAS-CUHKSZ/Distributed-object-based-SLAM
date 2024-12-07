function [err] = Show_result_Module(itermax, FLAG, REAL_PARA, BIAS_SET, real_path, IMU_data, X_ekf, X_iiekf, X_iekf, X_eiekf, t_ekf,t_iiekf, t_iekf, t_eiekf)
% SHOW_RESULT_MODULE: Show results
%   Input: time list and predicted results.
ifcam=FLAG.IF_camera;
iflid=FLAG.IF_LiDAR;
ifEKF=FLAG.IF_EKF;
ifiEKF=FLAG.IF_inEKF;
ifeiEKF=FLAG.IF_eiekf;
ifiiEKF=FLAG.IF_iiekf;
R_lidar_in_IMU_real = REAL_PARA.R_lidar_in_IMU_real;
p_lidar_in_IMU_real = REAL_PARA.p_lidar_in_IMU_real;
R_camera_in_IMU_real = REAL_PARA.R_camera_in_IMU_real;
p_camera_in_IMU_real = REAL_PARA.p_camera_in_IMU_real;
bia_R_lidar_in_IMU=BIAS_SET.bia_R_lidar_in_IMU;
bia_p_lidar_in_IMU=BIAS_SET.bia_p_lidar_in_IMU;
bia_p_camera_in_IMU=BIAS_SET.bia_p_camera_in_IMU;
bia_R_camera_in_IMU=BIAS_SET.bia_R_camera_in_IMU;
%%%Initial%%%%
%长度获取
sz_RealPath=length(real_path);  %真实路径点数
real_p_imu=zeros(sz_RealPath,3); %生成数据中IMU的真实位置
T_lidar_in_IMU_real=zeros(4,4);
T_lidar_in_IMU_real(4,4)=1;
T_lidar_in_IMU_real(1:3,1:3)=R_lidar_in_IMU_real;
T_lidar_in_IMU_real(1:3,4)=p_lidar_in_IMU_real';
T_camera_in_IMU_real=zeros(4,4);
T_camera_in_IMU_real(4,4)=1;
T_camera_in_IMU_real(1:3,1:3)=R_camera_in_IMU_real;
T_camera_in_IMU_real(1:3,4)=p_camera_in_IMU_real';
%%%%各个传感器数据用structure.T的形式存放，不计较速度%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%LiDAR的真实路径%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
TlidarinG=[];
if(iflid)
    real_p_lidar=zeros(sz_RealPath,3);
    for i=1:sz_RealPath
        real_p_imu(i,:)=real_path(i).T(1:3,4)';
        Tlidar=real_path(i).T*T_lidar_in_IMU_real;
        real_p_lidar(i,:)=Tlidar(1:3,4)';
        TlidarinG(i).T=Tlidar;   
    end
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%Camera真实路径%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
TcamerainG=[];
if(ifcam)
    real_p_cam=zeros(sz_RealPath,3);
    for i=1:sz_RealPath
        real_p_imu(i,:)=real_path(i).T(1:3,4)';
        Tcamera=real_path(i).T*T_camera_in_IMU_real;
        real_p_cam(i,:)=Tcamera(1:3,4)';
        TcamerainG(i).T=Tcamera;   
    end
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%获取各个估计器的估计结果%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
[T_IMU_ekf, T_lidar_ekf, T_camera_ekf,  ekf_p_imu, ekf_p_lidar, ekf_p_camera]=get_estimator_T(X_ekf,1,iflid, ifcam);
[T_IMU_iekf, T_lidar_iekf,  T_camera_iekf,  iekf_p_imu, iekf_p_lidar, iekf_p_camera]=get_estimator_T(X_iekf,0,iflid, ifcam);
[T_IMU_eiekf, T_lidar_eiekf,T_camera_eiekf,  eiekf_p_imu, eiekf_p_lidar, eiekf_p_camera]=get_estimator_T(X_eiekf,0,iflid, ifcam);
[T_IMU_iiekf, T_lidar_iiekf,T_camera_iiekf,  iiekf_p_imu, iiekf_p_lidar, iiekf_p_camera]=get_estimator_T(X_iiekf,0,iflid, ifcam);
[Err_IMU_R_ekf, Err_IMU_p_ekf]=cal_norm_error(T_IMU_ekf, real_path);
[Err_IMU_R_iekf, Err_IMU_p_iekf]=cal_norm_error(T_IMU_iekf, real_path);
[Err_IMU_R_eiekf, Err_IMU_p_eiekf]=cal_norm_error(T_IMU_eiekf, real_path);
[Err_IMU_R_iiekf, Err_IMU_p_iiekf]=cal_norm_error(T_IMU_iiekf, real_path);
if(iflid)
    [Err_Lid_R_ekf, Err_Lid_p_ekf]=cal_norm_error(T_lidar_ekf, TlidarinG);
    [Err_Lid_R_iekf, Err_Lid_p_iekf]=cal_norm_error(T_lidar_iekf, TlidarinG);
    [Err_Lid_R_eiekf, Err_Lid_p_eiekf]=cal_norm_error(T_lidar_eiekf, TlidarinG);
    [Err_Lid_R_iiekf, Err_Lid_p_iiekf]=cal_norm_error(T_lidar_iiekf, TlidarinG);
end
if(ifcam)
    [Err_cam_R_ekf, Err_cam_p_ekf]=cal_norm_error(T_camera_ekf, TcamerainG);
    [Err_cam_R_iekf, Err_cam_p_iekf]=cal_norm_error(T_camera_iekf, TcamerainG);
    [Err_cam_R_eiekf, Err_cam_p_eiekf]=cal_norm_error(T_camera_eiekf, TcamerainG);
    [Err_cam_R_iiekf, Err_cam_p_iiekf]=cal_norm_error(T_camera_iiekf, TcamerainG);
end

figure();
plot(1:length(real_p_imu(:,1)),real_p_imu(:,1));hold on
plot(1:length(real_p_imu(:,1)),iekf_p_imu(:,1));hold on

figure();
plot(1:length(real_p_imu(:,2)),real_p_imu(:,2));hold on
plot(1:length(real_p_imu(:,2)),iekf_p_imu(:,2));hold on

figure();
plot(1:length(real_p_imu(:,3)),real_p_imu(:,3));hold on
plot(1:length(real_p_imu(:,3)),iekf_p_imu(:,3));hold on



%%%%%%%%%%%%%%%%plot%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%1. trajectory
figure();
plot3(real_p_imu(:,1), real_p_imu(:,2), real_p_imu(:,3),'black','LineStyle','--','LineWidth',3,'DisplayName','Groundtruth'); hold on
if(ifEKF)
plot3(ekf_p_imu(:,1),ekf_p_imu(:,2),ekf_p_imu(:,3),'Color',"#2878B5",'LineWidth',1.5,'DisplayName','IEKF');hold on;    
end
if(ifiEKF)
plot3(iekf_p_imu(:,1),iekf_p_imu(:,2),iekf_p_imu(:,3),'c','LineWidth',1.5,'DisplayName','InEKF');hold on;  end
if(ifiiEKF)
plot3(eiekf_p_imu(:,1),eiekf_p_imu(:,2),eiekf_p_imu(:,3),'Color',"#F8AC8C",'LineWidth',1.5,'DisplayName','EIKF-C');hold on;  end
if(ifeiEKF)
plot3(iiekf_p_imu(:,1),iiekf_p_imu(:,2),iiekf_p_imu(:,3),'Color',"#C82423",'LineWidth',1.5,'DisplayName','EIKF-I');hold on;end
legend();
legend('Position',[0.71528,0.75714,0.19821,0.15913])
axis equal
%2. IMU R p error [RMSE]
figure();
subplot(2,1,1);
new_min_y=0.001;
new_max_y=1;
if(ifEKF)
semilogy(t_ekf, Err_IMU_R_ekf,'Color',"#2878B5",'LineWidth',1.5,'DisplayName','IEKF'); hold on
end
if(ifiEKF)
semilogy(t_iekf, Err_IMU_R_iekf,'c','LineWidth',1.5,'DisplayName','InEKF');hold on
end
if(ifeiEKF)
semilogy(t_eiekf, Err_IMU_R_eiekf, 'Color',"#F8AC8C",'LineWidth',1.5,'DisplayName','EIKF-C');hold on
end
if(ifiiEKF)
semilogy(t_iiekf, Err_IMU_R_iiekf,'Color',"#C82423",'LineWidth',1.5,'DisplayName','EIKF-I');hold on;  
end
%ylim([new_min_y new_max_y]);
grid on
legend();
ylabel('Orientation(rad)'); % Label the y-axis in English
title('Evolution of RMSE')
subplot(2,1,2);
if(ifEKF)
semilogy(t_ekf, Err_IMU_p_ekf,'Color',"#2878B5",'LineWidth',1.5,'DisplayName','IEKF'); hold on
end
if(ifiEKF)
semilogy(t_iekf, Err_IMU_p_iekf,'c','LineWidth',1.5,'DisplayName','InEKF');hold on
end
if(ifeiEKF)
semilogy(t_eiekf, Err_IMU_p_eiekf, 'Color',"#F8AC8C",'LineWidth',1.5,'DisplayName','EIKF-C');hold on
end
if(ifiiEKF)
semilogy(t_iiekf, Err_IMU_p_iiekf,'Color',"#C82423",'LineWidth',1.5,'DisplayName','EIKF-I');hold on;  
end
grid on
legend();
xlabel('Time(s)'); % Label the x-axis
ylabel('Position(m)'); % Label the y-axis in English
save("error_evol","Err_IMU_R_ekf","Err_IMU_R_iekf","Err_IMU_R_eiekf","Err_IMU_R_iiekf","Err_IMU_p_ekf","Err_IMU_p_iekf","Err_IMU_p_eiekf","Err_IMU_p_iiekf","t_iekf");
disp('--------------/mean RMSE/-------------')
disp('Orientation(rad):')
fprintf('Err_IMU_R_ekf | ');  disp(num2str(mean(Err_IMU_R_ekf(1:end))));
fprintf('Err_IMU_R_iekf | '); disp(num2str(mean(Err_IMU_R_iekf(1:end))));
fprintf('Err_IMU_R_eiekf | '); disp(num2str(mean(Err_IMU_R_eiekf(1:end))));
fprintf('Err_IMU_R_iiekf | '); disp(num2str(mean(Err_IMU_R_iiekf(1:end))));
disp('Position(m):')
fprintf('Err_IMU_p_ekf | ');disp(num2str(mean(Err_IMU_p_ekf(1:end))));
fprintf('Err_IMU_p_iekf | ');disp(num2str(mean(Err_IMU_p_iekf(1:end))));
fprintf('Err_IMU_p_eiekf | ');disp(num2str(mean(Err_IMU_p_eiekf(1:end))));
fprintf('Err_IMU_p_iiekf | ');disp(num2str(mean(Err_IMU_p_iiekf(1:end))));



%%3. NEES
err =[mean(Err_IMU_R_ekf(8:end)),mean(Err_IMU_p_ekf(8:end)),mean(Err_IMU_R_iekf(8:end)),mean(Err_IMU_p_iekf(8:end)),mean(Err_IMU_R_eiekf(8:end)),mean(Err_IMU_p_eiekf(8:end)),mean(Err_IMU_R_iiekf(8:end)),mean(Err_IMU_p_iiekf(8:end))] ;

end



