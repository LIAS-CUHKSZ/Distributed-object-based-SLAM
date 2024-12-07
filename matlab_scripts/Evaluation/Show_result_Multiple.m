function [] = Show_result_Multiple(FLAG, REAL_PARA, BIAS_SET, real_path, X_ekf_all, X_iekf_all,X_iiekf_all, X_eiekf_all, t_ekf_all)
%% Show Result Multiple
ifcam=FLAG.IF_camera;
iflid=FLAG.IF_LiDAR;
ifEKF=FLAG.IF_EKF;
ifiEKF=FLAG.IF_inEKF;
ifiiEKF=FLAG.IF_iiekf;
ifeiEKF=FLAG.IF_eiekf;

R_lidar_in_IMU_real = REAL_PARA.R_lidar_in_IMU_real;
p_lidar_in_IMU_real = REAL_PARA.p_lidar_in_IMU_real;
R_camera_in_IMU_real = REAL_PARA.R_camera_in_IMU_real;
p_camera_in_IMU_real = REAL_PARA.p_camera_in_IMU_real;

bia_R_lidar_in_IMU=BIAS_SET.bia_R_lidar_in_IMU;
bia_p_lidar_in_IMU=BIAS_SET.bia_p_lidar_in_IMU;
bia_p_camera_in_IMU=BIAS_SET.bia_p_camera_in_IMU;
bia_R_camera_in_IMU=BIAS_SET.bia_R_camera_in_IMU;

%% Ground Truth 
sz_RealPath=length(real_path);  % the length of real path
real_p_imu=zeros(sz_RealPath,3); % IMU real position
real_R_imu=zeros(sz_RealPath,3); % IMU real orientation
T_lidar_in_IMU_real=zeros(4,4);
T_lidar_in_IMU_real(4,4)=1;
T_lidar_in_IMU_real(1:3,1:3)=R_lidar_in_IMU_real;
T_lidar_in_IMU_real(1:3,4)=p_lidar_in_IMU_real';

T_camera_in_IMU_real=zeros(4,4);
T_camera_in_IMU_real(4,4)=1;
T_camera_in_IMU_real(1:3,1:3)=R_camera_in_IMU_real;
T_camera_in_IMU_real(1:3,4)=p_camera_in_IMU_real';
%% The data are stored as: structure.T
% Lidar real path
TlidarinG=[];
if(iflid)
    real_p_lidar=zeros(sz_RealPath,3);
    for i=1:sz_RealPath
        real_p_imu(i,:)=real_path(i).T(1:3,4)';
        [real_R_imu(i,3), real_R_imu(i,2), real_R_imu(i,1)]=dcm2angle(real_path(i).T(1:3,1:3));
        Tlidar=real_path(i).T*T_lidar_in_IMU_real;
        real_p_lidar(i,:)=Tlidar(1:3,4)';
        TlidarinG(i).T=Tlidar;   
    end
end
% Camera real path
TcamerainG=[];
if(ifcam)
    real_p_cam=zeros(sz_RealPath,3);
    for i=1:sz_RealPath
        [real_R_imu(i,3), real_R_imu(i,2), real_R_imu(i,1)]=dcm2angle(real_path(i).T(1:3,1:3));
        real_p_imu(i,:)=real_path(i).T(1:3,4)';
        Tcamera=real_path(i).T*T_camera_in_IMU_real;
        real_p_cam(i,:)=Tcamera(1:3,4)';
        TcamerainG(i).T=Tcamera;   
    end
end


%% Multiple Results
line_num=length(X_ekf_all); %一共有多少种设定所得到的状态
sz=length(X_ekf_all(1).X);
% initial 
px_IMU_list_ekf=zeros(sz,line_num);
py_IMU_list_ekf=zeros(sz,line_num);
pz_IMU_list_ekf=zeros(sz,line_num);
Rx_IMU_list_ekf=zeros(sz,line_num);
Ry_IMU_list_ekf=zeros(sz,line_num);
Rz_IMU_list_ekf=zeros(sz,line_num);
Ep_ekf=zeros(sz,line_num); % save position distance error
ER_ekf=zeros(sz,line_num); % save rotation error

px_IMU_list_iekf=zeros(sz,line_num);
py_IMU_list_iekf=zeros(sz,line_num);
pz_IMU_list_iekf=zeros(sz,line_num);
Rx_IMU_list_iekf=zeros(sz,line_num);
Ry_IMU_list_iekf=zeros(sz,line_num);
Rz_IMU_list_iekf=zeros(sz,line_num);
Ep_iekf=zeros(sz,line_num); % save position distance error
ER_iekf=zeros(sz,line_num); % save rotation error

px_IMU_list_iiekf=zeros(sz,line_num);
py_IMU_list_iiekf=zeros(sz,line_num);
pz_IMU_list_iiekf=zeros(sz,line_num);
Rx_IMU_list_iiekf=zeros(sz,line_num);
Ry_IMU_list_iiekf=zeros(sz,line_num);
Rz_IMU_list_iiekf=zeros(sz,line_num);
Ep_iiekf=zeros(sz,line_num); % save position distance error
ER_iiekf=zeros(sz,line_num); % save rotation error

px_IMU_list_eiekf=zeros(sz,line_num);
py_IMU_list_eiekf=zeros(sz,line_num);
pz_IMU_list_eiekf=zeros(sz,line_num);
Rx_IMU_list_eiekf=zeros(sz,line_num);
Ry_IMU_list_eiekf=zeros(sz,line_num);
Rz_IMU_list_eiekf=zeros(sz,line_num);
Ep_eiekf=zeros(sz,line_num); % save position distance error
ER_eiekf=zeros(sz,line_num); % save rotation error




for i = 1:line_num
X_ekf=X_ekf_all(i).X;
X_iekf=X_iekf_all(i).X;
X_iiekf=X_iiekf_all(i).X;
X_eiekf=X_eiekf_all(i).X;
t_ekf=t_ekf_all;

[T_IMU_ekf, T_lidar_ekf, T_camera_ekf,  ekf_p_imu, ekf_p_lidar, ekf_p_camera, ekf_R_imu, ekf_R_lidar, ekf_R_camera]=get_estimator_T(X_ekf,1,iflid, ifcam);
[T_IMU_iekf, T_lidar_iekf,  T_camera_iekf,  iekf_p_imu, iekf_p_lidar, iekf_p_camera,iekf_R_imu, iekf_R_lidar,iekf_R_camera]=get_estimator_T(X_iekf,0,iflid, ifcam);
[T_IMU_iiekf, T_lidar_iiekf,  T_camera_iiekf,  iiekf_p_imu, iiekf_p_lidar, iiekf_p_camera,iiekf_R_imu, iiekf_R_lidar,iiekf_R_camera]=get_estimator_T(X_iiekf,0,iflid, ifcam);
[T_IMU_eiekf, T_lidar_eiekf,T_camera_eiekf,  eiekf_p_imu, eiekf_p_lidar, eiekf_p_camera,eiekf_R_imu, eiekf_R_lidar, eiekf_R_camera]=get_estimator_T(X_eiekf,0,iflid, ifcam);

[ER_ekf(:,i), Ep_ekf(:,i)] = cal_invariant_error(T_IMU_ekf, real_path);
[ER_iekf(:,i), Ep_iekf(:,i)] = cal_invariant_error(T_IMU_iekf, real_path);
[ER_iiekf(:,i), Ep_iiekf(:,i)] = cal_invariant_error(T_IMU_iiekf, real_path);
[ER_eiekf(:,i), Ep_eiekf(:,i)] = cal_invariant_error(T_IMU_eiekf, real_path);

px_IMU_list_ekf(:,i)=ekf_p_imu(:,1);
py_IMU_list_ekf(:,i)=ekf_p_imu(:,2);
pz_IMU_list_ekf(:,i)=ekf_p_imu(:,3);

px_IMU_list_iekf(:,i)=iekf_p_imu(:,1);
py_IMU_list_iekf(:,i)=iekf_p_imu(:,2);
pz_IMU_list_iekf(:,i)=iekf_p_imu(:,3);

px_IMU_list_eiekf(:,i)=eiekf_p_imu(:,1);
py_IMU_list_eiekf(:,i)=eiekf_p_imu(:,2);
pz_IMU_list_eiekf(:,i)=eiekf_p_imu(:,3);

px_IMU_list_iiekf(:,i)=iiekf_p_imu(:,1);
py_IMU_list_iiekf(:,i)=iiekf_p_imu(:,2);
pz_IMU_list_iiekf(:,i)=iiekf_p_imu(:,3);

Rx_IMU_list_ekf(:,i)=ekf_R_imu(:,1);
Ry_IMU_list_ekf(:,i)=ekf_R_imu(:,2);
Rz_IMU_list_ekf(:,i)=ekf_R_imu(:,3);

Rx_IMU_list_iekf(:,i)=iekf_R_imu(:,1);
Ry_IMU_list_iekf(:,i)=iekf_R_imu(:,2);
Rz_IMU_list_iekf(:,i)=iekf_R_imu(:,3);

Rx_IMU_list_eiekf(:,i)=eiekf_R_imu(:,1);
Ry_IMU_list_eiekf(:,i)=eiekf_R_imu(:,2);
Rz_IMU_list_eiekf(:,i)=eiekf_R_imu(:,3);

Rx_IMU_list_eiekf(:,i)=iiekf_R_imu(:,1);
Ry_IMU_list_eiekf(:,i)=iiekf_R_imu(:,2);
Rz_IMU_list_eiekf(:,i)=iiekf_R_imu(:,3);
 norm(ER_ekf))

end

axis auto
%% figure 1. error comparison
figure();
sgtitle("Root Mean Squared Error",'Fontname', 'Times New Roman')
subplot(2,4,1);
for i = 1:line_num
    plot(t_ekf,Ep_ekf(:,i)); grid on; hold on;
end

ylabel('EKF: Error Position (m)', 'Fontname', 'Times New Roman');

subplot(2,4,5);
for i = 1:line_num
    plot(t_ekf,ER_ekf(:,i)); grid on; hold on;
end
ylabel('EKF: Error Orientation (deg)','Fontname', 'Times New Roman');

subplot(2,4,2);
for i = 1:line_num
    plot(t_ekf,Ep_iekf(:,i)); grid on; hold on;
end
ylabel('InEKF: Error position (m)','Fontname', 'Times New Roman');

subplot(2,4,6);
for i = 1:line_num
    plot(t_ekf,ER_iekf(:,i)); grid on; hold on;
end
ylabel('InEKF: Error Orientation (deg)','Fontname', 'Times New Roman');

subplot(2,4,3);
for i = 1:line_num
    plot(t_ekf,Ep_iiekf(:,i)); grid on; hold on;
end
title('EIKF-I: position RMSE');

subplot(2,4,7);
for i = 1:line_num
    plot(t_ekf,ER_iiekf(:,i)); grid on; hold on;
end
title('EIKF-I: rotation RMSE');

subplot(2,4,4);
for i = 1:line_num
    plot(t_ekf,Ep_eiekf(:,i)); grid on; hold on;
end
ylabel('EIKF-C: Error position (m)','Fontname', 'Times New Roman');

subplot(2,4,8);
for i = 1:line_num
    plot(t_ekf,ER_eiekf(:,i)); grid on; hold on;
end
ylabel('EIKF-C: Error Orientation (deg)','Fontname', 'Times New Roman');












