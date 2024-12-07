Exp=[0 1];%landmark or noise
Cam_LiD=[0 1];%cam or LiDAR
if(Exp(1) && Cam_LiD(1))
    load("evaluation/Camera/diff_landmarks_camera.mat")
elseif (Exp(2) && Cam_LiD(1))
    load("evaluation/Camera/diff_Noises_camera.mat")
elseif(Exp(1) && Cam_LiD(2))
    load("evaluation/LiDAR/diff_Landmarks_LiDAR.mat")
elseif(Exp(2) && Cam_LiD(2))
     load("evaluation/LiDAR/diff_Noises_LiDAR.mat")
else
    load("evaluation/LiDAR/diff_init_LiDAR.mat")
end

subplot(2,1,1);
semilogy(x, err_ekf_R, 'Color',"#2878B5", 'LineWidth', 2,'Marker', "square"); % plot EKF
hold on; % 保持图形以绘制后续数据

semilogy(x, err_iekf_R, 'c','LineWidth', 2, 'Marker', "square"); % plot InEKF
semilogy(x, err_eikf_R, 'Color',"#F8AC8C",'LineWidth', 2, 'Marker', "square"); %plot EIKF
semilogy(x, err_iiekf_R, 'Color',"#C82423",'LineWidth', 2, 'Marker', "square"); % plot IIEKF
ylabel('Orientation(rad)'); % Label the y-axis in English
legend(' IEKF', 'InEKF', 'EIKF-C', 'EIKF-I'); 
grid on;
title('Logarithmic Plot of RMSE');

subplot(2,1,2);
semilogy(x, err_ekf_p, 'Color',"#2878B5", 'LineWidth', 2,'Marker', "square"); % plot EKF
hold on; % 保持图形以绘制后续数据

semilogy(x, err_iekf_p, 'c','LineWidth', 2, 'Marker', "square"); % plot InEKF
semilogy(x, err_eikf_p, 'Color',"#F8AC8C",'LineWidth', 2, 'Marker', "square"); %plot EIKF
semilogy(x, err_iiekf_p, 'Color',"#C82423",'LineWidth', 2, 'Marker', "square"); % plot IIEKF


if(Exp(1))
    xlabel('Number of Landmarks'); % Label the x-axis
elseif (Exp(2))
    xlabel('S.T.D of Noises'); % Label the x-axis
else
    xlabel('Scale of Initial Value');
end

%num of lands
%scale of noise
%xlabel('Scale of Noise');

%scale of initial value
ylabel('Position(m)'); % Label the y-axis in English
legend(' IEKF', 'InEKF', 'EIKF-C', 'EIKF-I');
hold off; % 停止保持图形

grid on
