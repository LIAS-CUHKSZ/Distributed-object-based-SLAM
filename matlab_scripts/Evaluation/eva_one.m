%% Simulation for Camera-LiDAR-IMU Odometry Based on the Iterated Invariant Kalman Filter
% 
% 
% _To be used as comparision with our algorithm._ 
%% 
% 
% 
% _When run this code at the first time, you need to run *gen_data* first to 
% get the simulated dataset._

clear; clc
data_folder='Get_DATA';
addpath(genpath(data_folder));

tool_folder='tools';
addpath(genpath(tool_folder));

load("data16.mat");
%Senseor    Lidar, Camera [You can only choose one sensor at one time]

Sensors =   [1,   0];
% Filter    EKF|InEKF|EIKF|iterated InEKF
Filter = [1,   1,   1,   0];
IF_EKF_Online_Calib=0;
IF_IEKF_Update_bgba=1;

t_iiekf_set_up=[];
X_iekf_set=[];
X_ekf_set=[];
t_iekf_set=[];
t_iiekf_set=[];
t_ekf_set=[];
X_eiekf_set=[];
X_iiekf_set=[];
t_eiekf_set=[];
P_ekf_set=[];
P_iekf_set=[];
P_eiekf_set=[];
P_iiekf_set=[];
%% Initialization for Both Filters

itermax =2;  % The max iteration number for Gauss Newton.

N_cam=2;% Initial Count of camera measurement
N_lid=2;% Initial Count of LiDAR measurement
N_imu=1;% Intial Step of IMU propagation
%Set the initial bias of states:
%IMU状态初始误差
bia_R_imu_angle=[pi/30,pi/15,-pi/6];
bia_R_imu = angle2dcm(bia_R_imu_angle(1), bia_R_imu_angle(2), bia_R_imu_angle(3));
bia_p_imu =[5,-3,2];
bia_v_imu = zeros(1,3);
bia_b_a   = [0,0,0];
bia_b_g   = [0.00,0.00,0.00];

%外参初始误差
bia_p_camera_in_IMU = zeros(1,3);
bia_angle_R_ci=[0,0,0];
bia_R_camera_in_IMU = angle2dcm(bia_angle_R_ci(1), bia_angle_R_ci(2), bia_angle_R_ci(3));

bia_p_lidar_in_IMU = [0,0,0];
bia_angle_R_li=[0,0,0];
bia_R_lidar_in_IMU =  angle2dcm(bia_angle_R_li(1),bia_angle_R_li(2),bia_angle_R_li(3));
bia_t_c = 0;
bia_t_l = 0;

%Set the initial states:
X_init.Rimu = bia_R_imu*T_real0(1:3, 1:3);
X_init.pimu = T_real0(1:3, 4)'+bia_p_imu;

T_imu0 =zeros(4,4); T_imu0(4,4)=1; T_imu0(1:3,1:3)=X_init.Rimu; T_imu0(1:3,4)=X_init.pimu';

X_init.vimu = IMU_data(N_imu).v_inglobal_real+bia_v_imu;
X_init.ba = b_a_real + bia_b_a;
X_init.bg = b_g_real + bia_b_g;

%Set the initial extrinsic paramters:
i_R_c=R_camera_in_IMU_real; % You can get this by pre calibration or the draft of design, etc
i_p_c=p_camera_in_IMU_real;
i_R_l= R_lidar_in_IMU_real;
i_p_l= p_lidar_in_IMU_real;

X_init.Rli = bia_R_lidar_in_IMU * R_lidar_in_IMU_real;
X_init.pli = p_lidar_in_IMU_real+ bia_p_lidar_in_IMU;
X_init.pci = p_camera_in_IMU_real+ bia_p_camera_in_IMU;
X_init.Rci = bia_R_camera_in_IMU *R_camera_in_IMU_real;


i_T_c=zeros(4,4);
i_T_c(4,4)=1;    i_T_c(1:3, 1:3)=i_R_c;       i_T_c(1:3, 4)=i_p_c';  

i_T_l=zeros(4,4);
i_T_l(4,4)=1;    i_T_l(1:3, 1:3)=i_R_l;       i_T_l(1:3, 4)=i_p_l';  

T_camera0 = T_imu0 * i_T_c;
T_lidar0  = T_imu0 * i_T_l;  %T_imu0有过一次误差，外参也有过一次误差，没有必要再来一次。

X_init.pc = T_camera0(1:3,4)' ;
X_init.Rc = T_camera0(1:3, 1:3);
X_init.pl = T_lidar0(1:3,4)';
X_init.Rl = T_lidar0(1:3,1:3);
X_init.tc = t_c_real+bia_t_c;
X_init.tl = t_l_real+bia_t_l;

% Initialize the Covariance Matrix |P|

P_init=10^-4* eye(29);
if(IF_IEKF_Update_bgba)    
    P_init_iekf=10^-4*eye(15);
else
    P_init_iekf=10^-4*eye(9);
end
%% 
% Set Noises

SIGMA_SET.sigma_tc=sigma_tc;
SIGMA_SET.sigma_tl=sigma_tl;
SIGMA_SET.sigma_cam=sigma_cam;
SIGMA_SET.sigma_lid=sigma_lid;
SIGMA_SET.sigma_a=sigma_a;
SIGMA_SET.sigma_g=sigma_g;
SIGMA_SET.sigma_bg=sigma_bg;
SIGMA_SET.sigma_ba=sigma_ba;
%% 
% 
%% Esitimate Now
%% EKF

if(Filter(1))
    N_cam=2;% Initial Count of camera measurement
    N_lid=2;% Initial Count of LiDAR measurement
    N_imu=1;% Intial Step of IMU propagation    
    CAM_IF_UPDATE=0;% Value 1 means that new camera measurement is received.
    LID_IF_UPDATE=0;% Value 1 means that new LiDAR measurement is received.
    
  % To show the iniatial point.
    N_update=1;
    N_ekf_set_up(N_update)=1; 
    X_ekf_set(N_update).Rimu=X_init.Rimu;
    X_ekf_set(N_update).pimu=X_init.pimu;
    X_ekf_set(N_update).Rli=X_init.Rli;
    X_ekf_set(N_update).pli=X_init.pli;        
    X_ekf_set(N_update).Rci=X_init.Rci;
    X_ekf_set(N_update).pci=X_init.pci;
    X_ekf_set(N_update).vimu=X_init.vimu;
    X_ekf_set(N_update).bg=X_init.bg;
    X_ekf_set(N_update).ba=X_init.ba;
    t_ekf_set_up(N_update)=0;
    N_update=N_update+1;

    t_imu=IMU_data(N_imu).t;
    X_former=X_init;
    P_former=P_init;
    camera_measure_now=camera_data(N_cam).uv;
    lidar_measure_now=LiDAR_data(N_lid).p_m;

    tic  %计时开始
    while (t_imu<T_WORKING) 
   
        if(IF_EKF_Online_Calib)
            t_imu=IMU_data(N_imu).t;        
            t_cam=camera_data(N_cam).timestamp;%In case that the time letency is negative, start from the second measurement.
            t_lid=LiDAR_data(N_lid).timestamp;
     %   ====================== propagate======================================
            acc_m=IMU_data(N_imu).a;%IMU's measurement is used by propagation
            w_m=IMU_data(N_imu).w;
            [X_pred, P_pred]=  EKF_propagate(X_former, P_former, w_m, acc_m, dti, SIGMA_SET, 1);
     %   ================== Iterated EKF Update ==============================
            % Receiving New Measurements
            if ((abs(t_cam-t_imu)<0.0000001)&&Sensors(2))
                    CAM_IF_UPDATE=1;
                    camera_measure_now=camera_data(N_cam);%读取新的照片
                    N_cam=N_cam+1;
            end
            if ((abs(t_lid-t_imu)<0.0000001)&&Sensors(1))
                LID_IF_UPDATE=1;
                lidar_measure_now=LiDAR_data(N_lid).p_m;
                N_lid=N_lid+1; 
            end
            %To update
            [X_esti, P_esti]=EKF_iter_update(X_pred, P_pred, CAM_IF_UPDATE,LID_IF_UPDATE, acc_m,w_m,camera_data(N_cam),lidar_measure_now, N_LMLIDAR,    lm_lidar,    SIGMA_SET,1, fx, fy, cx, cy,    1);
            %Only record updated one
            if (CAM_IF_UPDATE || LID_IF_UPDATE)
                X_ekf_set(N_update).Rimu=X_esti.Rimu;
                X_ekf_set(N_update).pimu=X_esti.pimu;
                X_ekf_set(N_update).Rli=X_esti.Rli;
                X_ekf_set(N_update).pli=X_esti.pli;        
                X_ekf_set(N_update).Rci=X_esti.Rci;
                X_ekf_set(N_update).pci=X_esti.pci;
                X_ekf_set(N_update).vimu=X_esti.vimu;
                X_ekf_set(N_update).bg=X_esti.bg;
                X_ekf_set(N_update).ba=X_esti.ba;
                t_ekf_set_up(N_update)=t_imu;
                P_ekf_set(N_update).P=P_esti;
                N_ekf_set_up(N_update)=N_imu;
                N_update=N_update+1;
            end
            N_imu=N_imu+1;
            X_former=X_esti;
            P_former=(P_esti+P_esti')/2;
            LID_IF_UPDATE=0;
            CAM_IF_UPDATE=0;
        else
            P_former=P_init(1:15, 1:15);
            t_imu=IMU_data(N_imu).t;        
            t_cam=camera_data(N_cam).timestamp;%In case that the time letency is negative, start from the second measurement.
            t_lid=LiDAR_data(N_lid).timestamp;
     %   ====================== propagate======================================
            acc_m=IMU_data(N_imu).a;%IMU's measurement is used by propagation
            w_m=IMU_data(N_imu).w;
            [X_pred, P_pred]=  EKF_propagate(X_former, P_former, w_m, acc_m, dti, SIGMA_SET,0);
     %   ================== Iterated EKF Update ==============================
            % Receiving New Measurements
            if ((abs(t_cam-t_imu)<0.0000001)&&Sensors(2))
                    CAM_IF_UPDATE=1;
                    camera_measure_now=camera_data(N_cam);%读取新的照片
                    N_cam=N_cam+1;
            end
            if ((abs(t_lid-t_imu)<0.0000001)&&Sensors(1))
                LID_IF_UPDATE=1;
                lidar_measure_now=LiDAR_data(N_lid).p_m;
                N_lid=N_lid+1; 
            end
            %To update
            [X_esti, P_esti]=EKF_iter_update(X_pred, P_pred, CAM_IF_UPDATE,LID_IF_UPDATE, acc_m,w_m,camera_data(N_cam),lidar_measure_now, N_LMLIDAR,    lm_lidar,    SIGMA_SET,1, fx, fy, cx, cy,    0);
            %Only record updated one
            if (CAM_IF_UPDATE|LID_IF_UPDATE)
                X_ekf_set(N_update).Rimu=X_esti.Rimu;
                X_ekf_set(N_update).pimu=X_esti.pimu;
                X_ekf_set(N_update).Rli=X_esti.Rli;
                X_ekf_set(N_update).pli=X_esti.pli;        
                X_ekf_set(N_update).Rci=X_esti.Rci;
                X_ekf_set(N_update).pci=X_esti.pci;
                X_ekf_set(N_update).vimu=X_esti.vimu;
                X_ekf_set(N_update).bg=X_esti.bg;
                X_ekf_set(N_update).ba=X_esti.ba;
                t_ekf_set_up(N_update)=t_imu;
                P_ekf_set(N_update).P=P_esti;
                N_ekf_set_up(N_update)=N_imu;
                N_update=N_update+1;
            end
            N_imu=N_imu+1;
            X_former=X_esti;
            P_former=(P_esti+P_esti')/2;
            LID_IF_UPDATE=0;
            CAM_IF_UPDATE=0;


        end
    end
    disp('EKF Time consuming:');
    toc
    %============================Estimater Done!=================================
    %================================Debug======================================
    disp("Follow time:"); 
    t_ekf=t_imu
end


%% Iterated InEKF

if(Filter(4))
    N_cam=2;% Initial Count of camera measurement
    N_lid=2;% Initial Count of LiDAR measurement
    N_imu=1;% Intial Step of IMU propagation    
    CAM_IF_UPDATE=0;% Value 1 means that new camera measurement is received.
    LID_IF_UPDATE=0;% Value 1 means that new LiDAR measurement is received.
    
    % To show the iniatial point.
    N_update=1;
    N_iekf_set_up(N_update)=1; 
   
    X_iiekf_set(N_update).Rimu=X_init.Rimu;
    X_iiekf_set(N_update).pimu=X_init.pimu;
    X_iiekf_set(N_update).Rl=X_init.Rl;
    X_iiekf_set(N_update).pl=X_init.pl;
    X_iiekf_set(N_update).Rc=X_init.Rc;
    X_iiekf_set(N_update).pc=X_init.pc;
    X_iiekf_set(N_update).vimu=X_init.vimu;
    X_iiekf_set(N_update).bg=X_init.bg;
    X_iiekf_set(N_update).ba=X_init.ba;
    t_iiekf_set_up(N_update)=0;
    N_update=N_update+1;
    t_imu=IMU_data(N_imu).t;

    X_former=X_init;
    P_former=P_init_iekf;
    camera_measure_now=camera_data(N_cam).uv;
    lidar_measure_now=LiDAR_data(N_lid).p_m;
    EstX_i=[]; EstY_i=[]; EstZ_i=[]; %Record the estimated position of IMU.
    tic
    while (t_imu<T_WORKING) 
            t_imu=IMU_data(N_imu).t;
            t_iiekf_set(N_imu)=t_imu;
            t_cam=camera_data(N_cam).timestamp;%In case that the time letency is negative, start from the second measurement.
            t_lid=LiDAR_data(N_lid).timestamp;
    %   ====================== propagate======================================
            acc_m=IMU_data(N_imu).a;%IMU's measurement is used by propagation
            w_m=IMU_data(N_imu).w;
            [X_pred, P_pred]=  inEKF_propagate(X_former, P_former, w_m, acc_m, dti, SIGMA_SET, IF_IEKF_Update_bgba);
    %   ================== Iterated EKF Update ==============================
      %      Read measurements
            if ((abs(t_cam-t_imu)<0.0000001)&&Sensors(2))
                CAM_IF_UPDATE=1;
                camera_measure_now=camera_data(N_cam);%读取新的照片
                N_cam=N_cam+1;
            end
            if ((abs(t_lid-t_imu)<0.0000001)&&Sensors(1))
                LID_IF_UPDATE=1;
                lidar_measure_now=LiDAR_data(N_lid).p_m;
                N_lid=N_lid+1;  
            end

            [X_esti, P_esti]=inEKF_iter_update(X_pred, P_pred, CAM_IF_UPDATE,LID_IF_UPDATE, acc_m,w_m,camera_data(N_cam), lidar_measure_now,N_LMLIDAR,N_LMCAMERA,lm_camera,lm_lidar, SIGMA_SET, itermax,CAM_INTRISIC,IF_IEKF_Update_bgba);
            if (CAM_IF_UPDATE|LID_IF_UPDATE) 
            X_iiekf_set(N_update).Rimu=X_esti.Rimu;
            X_iiekf_set(N_update).pimu=X_esti.pimu;
            X_iiekf_set(N_update).Rl=X_esti.Rl;
            X_iiekf_set(N_update).pl=X_esti.pl;
            X_iiekf_set(N_update).Rc=X_esti.Rc;
            X_iiekf_set(N_update).pc=X_esti.pc;
            X_iiekf_set(N_update).vimu=X_esti.vimu;
            X_iiekf_set(N_update).bg=X_esti.bg;
            X_iiekf_set(N_update).ba=X_esti.ba;
            t_iiekf_set_up(N_update)=t_imu;

            P_iiekf_set(N_update).P=P_esti;
            N_iiekf_set_up(N_update)=N_imu;
            N_update=N_update+1;
            end
            N_imu=N_imu+1;
        %%%%%%%%%%%%%%%%%%%% Tracking detection %%%%%%%%%%%%%%%%%%%%%
       
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        X_former=X_esti;
        P_former=(P_esti+P_esti')/2;
        LID_IF_UPDATE=0;
        CAM_IF_UPDATE=0;
    end

    disp('IIEKF Time consuming:');
    toc
    %============================Estimater Done!=================================
    %================================Debug======================================
    disp("Follow time:"); 
    t_iiekf = t_imu
end
%% InEKF

if(Filter(2))
    N_cam=2;% Initial Count of camera measurement
    N_lid=2;% Initial Count of LiDAR measurement
    N_imu=1;% Intial Step of IMU propagation    
    CAM_IF_UPDATE=0;% Value 1 means that new camera measurement is received.
    LID_IF_UPDATE=0;% Value 1 means that new LiDAR measurement is received.
    
    % To show the iniatial point.
    N_update=1;
    N_iekf_set_up(N_update)=1; 
   
    X_iekf_set(N_update).Rimu=X_init.Rimu;
    X_iekf_set(N_update).pimu=X_init.pimu;
    X_iekf_set(N_update).Rl=X_init.Rl;
    X_iekf_set(N_update).pl=X_init.pl;
    X_iekf_set(N_update).Rc=X_init.Rc;
    X_iekf_set(N_update).pc=X_init.pc;
    X_iekf_set(N_update).vimu=X_init.vimu;
    X_iekf_set(N_update).bg=X_init.bg;
    X_iekf_set(N_update).ba=X_init.ba;
    t_iekf_set_up(N_update)=0;
    N_update=N_update+1;
    t_imu=IMU_data(N_imu).t;

    X_former=X_init;
    P_former=P_init_iekf;
    camera_measure_now=camera_data(N_cam).uv;
    lidar_measure_now=LiDAR_data(N_lid).p_m;
    EstX_i=[]; EstY_i=[]; EstZ_i=[]; %Record the estimated position of IMU.
    tic
    while (t_imu<T_WORKING) 
            t_imu=IMU_data(N_imu).t;
            t_iekf_set(N_imu)=t_imu;
            t_cam=camera_data(N_cam).timestamp;%In case that the time letency is negative, start from the second measurement.
            t_lid=LiDAR_data(N_lid).timestamp;
    %   ====================== propagate======================================
            acc_m=IMU_data(N_imu).a;%IMU's measurement is used by propagation
            w_m=IMU_data(N_imu).w;
            [X_pred, P_pred]=  inEKF_propagate(X_former, P_former, w_m, acc_m, dti, SIGMA_SET, IF_IEKF_Update_bgba);
    %   ================== Iterated EKF Update ==============================
      %      Read measurements
            if ((abs(t_cam-t_imu)<0.0000001)&&Sensors(2))
                CAM_IF_UPDATE=1;
                camera_measure_now=camera_data(N_cam);%读取新的照片
                N_cam=N_cam+1;
            end
            if ((abs(t_lid-t_imu)<0.0000001)&&Sensors(1))
                LID_IF_UPDATE=1;
                lidar_measure_now=LiDAR_data(N_lid).p_m;
                N_lid=N_lid+1;  
            end

            [X_esti, P_esti]=inEKF_iter_update(X_pred, P_pred, CAM_IF_UPDATE,LID_IF_UPDATE, acc_m,w_m,camera_data(N_cam), lidar_measure_now,N_LMLIDAR,N_LMCAMERA,lm_camera,lm_lidar, SIGMA_SET, 1,CAM_INTRISIC,IF_IEKF_Update_bgba);
            if (CAM_IF_UPDATE|LID_IF_UPDATE) 
            X_iekf_set(N_update).Rimu=X_esti.Rimu;
            X_iekf_set(N_update).pimu=X_esti.pimu;
            X_iekf_set(N_update).Rl=X_esti.Rl;
            X_iekf_set(N_update).pl=X_esti.pl;
            X_iekf_set(N_update).Rc=X_esti.Rc;
            X_iekf_set(N_update).pc=X_esti.pc;
            X_iekf_set(N_update).vimu=X_esti.vimu;
            X_iekf_set(N_update).bg=X_esti.bg;
            X_iekf_set(N_update).ba=X_esti.ba;
            t_iekf_set_up(N_update)=t_imu;

            P_iekf_set(N_update).P=P_esti;
            N_iekf_set_up(N_update)=N_imu;
             N_update=N_update+1;
            end
            N_imu=N_imu+1;
        %%%%%%%%%%%%%%%%%%%% Tracking detection %%%%%%%%%%%%%%%%%%%%%
       
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        X_former=X_esti;
        P_former=(P_esti+P_esti')/2;
        LID_IF_UPDATE=0;
        CAM_IF_UPDATE=0;
    end

    disp('IEKF Time consuming:');
    toc
    %============================Estimater Done!=================================
    %================================Debug======================================
    disp("Follow time:"); 
    t_iekf = t_imu
end
%% EIKF

t_eiekf_set_up=[];
if(Filter(3))
    N_cam=2;% Initial Count of camera measurement
    N_lid=2;% Initial Count of LiDAR measurement
    N_imu=1;% Intial Step of IMU propagation    
    CAM_IF_UPDATE=0;% Value 1 means that new camera measurement is received.
    LID_IF_UPDATE=0;% Value 1 means that new LiDAR measurement is received.

  % To show the iniatial point.
        N_update=1;
        N_eiekf_set_up(N_update)=1; 
        X_eiekf_set(N_update).Rimu=X_init.Rimu;
        X_eiekf_set(N_update).pimu=X_init.pimu;
        X_eiekf_set(N_update).Rl=X_init.Rl;
        X_eiekf_set(N_update).pl=X_init.pl;
        X_eiekf_set(N_update).Rc=X_init.Rc;
        X_eiekf_set(N_update).pc=X_init.pc;
        X_eiekf_set(N_update).vimu=X_init.vimu;
        X_eiekf_set(N_update).bg=X_init.bg;
        X_eiekf_set(N_update).ba=X_init.ba;
        t_eiekf_set_up(N_update)=0;
        N_update=N_update+1;
    t_imu=IMU_data(N_imu).t;
    X_former=X_init;
    P_former=P_init_iekf;
    camera_measure_now=camera_data(N_cam).uv;
    lidar_measure_now=LiDAR_data(N_lid).p_m;
    EstX_i=[]; EstY_i=[]; EstZ_i=[]; %Record the estimated position of IMU.
    tic
    while (t_imu<T_WORKING) 
            t_imu=IMU_data(N_imu).t;
            t_eiekf_set(N_imu)=t_imu;
            t_cam=camera_data(N_cam).timestamp;%In case that the time letency is negative, start from the second measurement.
            t_lid=LiDAR_data(N_lid).timestamp;
    %   ====================== propagate======================================
            acc_m=IMU_data(N_imu).a;%IMU's measurement is used by propagation
            w_m=IMU_data(N_imu).w;
            [X_pred, P_pred]=  inEKF_propagate(X_former, P_former, w_m, acc_m, dti, SIGMA_SET,IF_IEKF_Update_bgba);
    %   ================== Iterated EKF Update ==============================
      %      Read measurements
            if ((abs(t_cam-t_imu)<0.0000001)&&Sensors(2))
                CAM_IF_UPDATE=1;
                camera_measure_now=camera_data(N_cam);%读取新的照片
                N_cam=N_cam+1;
            end

            if ((abs(t_lid-t_imu)<0.0000001)&&Sensors(1))

                LID_IF_UPDATE=1;
                lidar_measure_now=LiDAR_data(N_lid).p_m;
                N_lid=N_lid+1;  
            end
    
            [X_esti, P_esti]=EIEKF_iter_update(X_pred, P_pred, CAM_IF_UPDATE,LID_IF_UPDATE, acc_m,w_m,camera_measure_now,lidar_measure_now,N_LMLIDAR,lm_lidar, SIGMA_SET, CAM_INTRISIC,IF_IEKF_Update_bgba);
            if (CAM_IF_UPDATE || LID_IF_UPDATE)
                X_eiekf_set(N_update).Rimu=X_esti.Rimu;
                X_eiekf_set(N_update).pimu=X_esti.pimu;
                X_eiekf_set(N_update).Rl=X_esti.Rl;
                X_eiekf_set(N_update).pl=X_esti.pl;
                X_eiekf_set(N_update).Rc=X_esti.Rc;
                X_eiekf_set(N_update).pc=X_esti.pc;
                X_eiekf_set(N_update).vimu=X_esti.vimu;
                X_eiekf_set(N_update).bg=X_esti.bg;
                X_eiekf_set(N_update).ba=X_esti.ba;
            
                P_eiekf_set(N_update).P=P_esti;
                t_eiekf_set_up(N_update)=t_imu;
                N_eiekf_set_up(N_update)=N_imu;
                N_update=N_update+1;
            end
            N_imu=N_imu+1;
        %%%%%%%%%%%%%%%%%%%% Tracking detection %%%%%%%%%%%%%%%%%%%%%
 
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        X_former=X_esti;
        P_former=(P_esti+P_esti')/2;
        LID_IF_UPDATE=0;
        CAM_IF_UPDATE=0;
        
    end
    disp('EIEKF Time consuming:');
    toc
    %============================Estimater Done!=================================
    %================================Debug======================================
    disp("Follow time:"); 
    t_eiekf=t_imu
end
%% Show Results
% Choose which to show:

FLAG.IF_EKF=Filter(1);
FLAG.IF_inEKF=Filter(2);
FLAG.IF_eiekf=Filter(3);
FLAG.IF_iiekf=Filter(4);
FLAG.IF_camera=Sensors(2);
FLAG.IF_LiDAR=Sensors(1);

REAL_PARA.R_lidar_in_IMU_real=R_lidar_in_IMU_real;
REAL_PARA.p_lidar_in_IMU_real=p_lidar_in_IMU_real;
REAL_PARA.R_camera_in_IMU_real=R_camera_in_IMU_real;
REAL_PARA.p_camera_in_IMU_real=p_camera_in_IMU_real;
REAL_PARA.b_g_real=b_g_real;
REAL_PARA.b_a_real=b_a_real;
BIAS_SET.bia_R_lidar_in_IMU=bia_angle_R_li;
BIAS_SET.bia_p_lidar_in_IMU=bia_p_lidar_in_IMU;
BIAS_SET.bia_p_camera_in_IMU=bia_p_camera_in_IMU;
BIAS_SET.bia_R_camera_in_IMU=bia_angle_R_ci;
real_path_select = filterArrayByIndex(real_path,N_iekf_set_up);

Show_result_Module(itermax, FLAG, REAL_PARA, BIAS_SET, real_path_select, IMU_data, X_ekf_set,X_iiekf_set, X_iekf_set, X_eiekf_set,t_ekf_set_up,t_iiekf_set_up, t_iekf_set_up, t_eiekf_set_up);