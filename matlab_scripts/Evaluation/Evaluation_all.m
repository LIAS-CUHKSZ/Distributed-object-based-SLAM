%% This script is used to run multiple times and draw results.

new_data=0;
if (new_data)
    clear;clc
    Interval.dti=0.01;
    Interval.dtl=0.02;
    Interval.dtc=0.02;

    sigma_tc=0.0000; 
    sigma_tl=0.0000;
    sigma_bg=0.05; 
    sigma_ba=0.001; 
    sigma_a=0.01;
    sigma_g=0.05;
    sigma_cam=0.2; 
    sigma_lid=0.4; 
    SIGMA.sigma_g=sigma_g;
    SIGMA.sigma_a=sigma_a;
    SIGMA.sigma_tc=sigma_tc;
    SIGMA.sigma_tl=sigma_tl;
    SIGMA.sigma_bg=sigma_bg;
    SIGMA.sigma_ba=sigma_ba;
    SIGMA.sigma_cam=sigma_cam;
    SIGMA.sigma_lid=sigma_lid;

    fx=460;
    fy=460;
    cx=255;
    cy=255;
    image_w=640;
    image_h=640;

    CAM_INTRISIC.FOV=pi/4;
    CAM_INTRISIC.fx=fx;
    CAM_INTRISIC.fy=fy;
    CAM_INTRISIC.cx=cx;
    CAM_INTRISIC.cy=cy;    
    CAM_INTRISIC.image_w=image_w;
    CAM_INTRISIC.image_h=image_h;
    CAM_INTRISIC.tc=Interval.dtc; %相机采样频率
    CAM_INTRISIC.sigma_cam=sigma_cam;
    CAM_INTRISIC.sigma_tc=sigma_tc;

    p_camera_in_IMU_real = [0.0, 0.0, 0.0];
    R_camera_in_IMU_real = angle2dcm(0,pi/2 , 0);
    p_lidar_in_IMU_real = [1.0, 1.0, 1.0];
    R_lidar_in_IMU_real = angle2dcm(0, pi/2, pi/3);

    b_g_real=[0.02,0.01,0.02];%bias 
    b_a_real=[0.03,0.05,0.03];
    g_real=[0;0;-9.81];


   Real_Para.pci=p_camera_in_IMU_real;
   Real_Para.Rci=R_camera_in_IMU_real;
   Real_Para.pli=p_lidar_in_IMU_real;
   Real_Para.Rli=R_lidar_in_IMU_real;
   Real_Para.bg=b_g_real;
   Real_Para.ba=b_a_real;
   Real_Para.g=g_real;

   Para_Genearte_Data(3,Interval, SIGMA, CAM_INTRISIC, Real_Para);
end

new_run=1;
%% New run
if(new_run)

biasRlist=pi/12*rand(3,10);
biasplist=10*rand(3,10);
run_cnt=1;

for biasR=biasRlist
    for biasp=biasplist
        data_filename='./data/data1.mat';
        load(data_filename);
        %Iterations
        Initial_settings.itermax=3;
        Initial_settings.ifcalib=0;
        
        %Senseor    Lidar, Camera
        Sensors =   [0,   1];
        % Filter    EKF|IEKF|EIEKF
        Filter_choice = [1,  1,   1 ];
        
        Initial_settings.whichfilter=Filter_choice;
        Initial_settings.whichsensor=Sensors;
       
        %% Initialize parameters
        %Set the initial bias of states:
        %IMU位置和姿态的初始误差
        Initial_settings.bia_R_imu_angle=biasR;
        Initial_settings.bia_R_imu = angle2dcm(Initial_settings.bia_R_imu_angle(1), Initial_settings.bia_R_imu_angle(2), Initial_settings.bia_R_imu_angle(3));
        Initial_settings.bia_p_imu = biasp;
        Initial_settings.bia_v_imu = zeros(1,3);
        Initial_settings.bia_b_a   = [0,0,0];
        Initial_settings.bia_b_g   = [0.00,0.00,0.00];
        
        %外参初始误差
        Initial_settings.bia_p_camera_in_IMU = zeros(1,3);
        Initial_settings.bia_angle_R_ci=[0,0,0];
        Initial_settings.bia_R_camera_in_IMU = angle2dcm(Initial_settings.bia_angle_R_ci(1), Initial_settings.bia_angle_R_ci(2), Initial_settings.bia_angle_R_ci(3));
        Initial_settings.bia_p_lidar_in_IMU = [0,0,0];
        Initial_settings.bia_angle_R_li=[0,0,0];
        Initial_settings.bia_R_lidar_in_IMU =  angle2dcm(Initial_settings.bia_angle_R_li(1),Initial_settings.bia_angle_R_li(2),Initial_settings.bia_angle_R_li(3));
        Initial_settings.bia_t_c = 0;
        Initial_settings.bia_t_l = 0;
        
        %Set the initial states:
        X_init.Rimu = Initial_settings.bia_R_imu*T_real0(1:3, 1:3);
        X_init.pimu = T_real0(1:3, 4)'+Initial_settings.bia_p_imu';
        
        T_imu0 =zeros(4,4); T_imu0(4,4)=1; T_imu0(1:3,1:3)=X_init.Rimu; T_imu0(1:3,4)=X_init.pimu';
        
        X_init.vimu = IMU_data(1).v_inglobal_real+Initial_settings.bia_v_imu;
        X_init.ba = b_a_real + Initial_settings.bia_b_a;
        X_init.bg = b_g_real + Initial_settings.bia_b_g;
        
        %Set the initial extrinsic paramters:
        i_R_c=R_camera_in_IMU_real; % You can get this by pre calibration or the draft of design, etc
        i_p_c=p_camera_in_IMU_real;
        i_R_l= R_lidar_in_IMU_real;
        i_p_l= p_lidar_in_IMU_real;
        
        X_init.Rli = Initial_settings.bia_R_lidar_in_IMU * R_lidar_in_IMU_real;
        X_init.pli = p_lidar_in_IMU_real+ Initial_settings.bia_p_lidar_in_IMU;
        X_init.pci = p_camera_in_IMU_real+ Initial_settings.bia_p_camera_in_IMU;
        X_init.Rci = Initial_settings.bia_R_camera_in_IMU *R_camera_in_IMU_real;
        
        
        i_T_c=zeros(4,4);
        i_T_c(4,4)=1;    i_T_c(1:3, 1:3)=i_R_c;       i_T_c(1:3, 4)=i_p_c';  
        
        i_T_l=zeros(4,4);
        i_T_l(4,4)=1;    i_T_l(1:3, 1:3)=i_R_l;       i_T_l(1:3, 4)=i_p_l';  
        
        T_camera0 = T_imu0 * i_T_c;
        T_lidar0  = T_imu0 * i_T_l;  
        
        X_init.pc = T_camera0(1:3,4)' ;
        X_init.Rc = T_camera0(1:3, 1:3);
        X_init.pl = T_lidar0(1:3,4)';
        X_init.Rl = T_lidar0(1:3,1:3);
        X_init.tc = t_c_real+Initial_settings.bia_t_c;
        X_init.tl = t_l_real+Initial_settings.bia_t_l;
        
        Initial_settings.X_init=X_init;
        P_init=10^-4* eye(29);
        P_init_iekf=10^-4*eye(15);
        Initial_settings.P_init=P_init;
        Initial_settings.P_init_iekf=P_init_iekf;
        
        % Set Noises
        SIGMA_SET.sigma_tc=sigma_tc;
        SIGMA_SET.sigma_tl=sigma_tl;
        SIGMA_SET.sigma_cam=sigma_cam;
        SIGMA_SET.sigma_lid=sigma_lid;
        SIGMA_SET.sigma_a=sigma_a;
        SIGMA_SET.sigma_g=sigma_g;
        SIGMA_SET.sigma_bg=sigma_bg;
        SIGMA_SET.sigma_ba=sigma_ba;
        Initial_settings.SIGMA_SET=SIGMA_SET;
        %% Store settings for drawing the results.
        FLAG.IF_EKF=Filter_choice(1);
        FLAG.IF_inEKF=Filter_choice(2);
        FLAG.IF_eiekf=Filter_choice(3);
        FLAG.IF_camera=Sensors(2);
        FLAG.IF_LiDAR=Sensors(1);
        REAL_PARA.R_lidar_in_IMU_real=R_lidar_in_IMU_real;
        REAL_PARA.p_lidar_in_IMU_real=p_lidar_in_IMU_real;
        REAL_PARA.R_camera_in_IMU_real=R_camera_in_IMU_real;
        REAL_PARA.p_camera_in_IMU_real=p_camera_in_IMU_real;
        REAL_PARA.b_g_real=b_g_real;
        REAL_PARA.b_a_real=b_a_real;
        BIAS_SET.bia_R_lidar_in_IMU=Initial_settings.bia_angle_R_li;
        BIAS_SET.bia_p_lidar_in_IMU=Initial_settings.bia_p_lidar_in_IMU;
        BIAS_SET.bia_p_camera_in_IMU=Initial_settings.bia_p_camera_in_IMU;
        BIAS_SET.bia_R_camera_in_IMU=Initial_settings.bia_angle_R_ci;
        [real_path_select,X_ekf_set, X_iekf_set, X_eiekf_set,t_ekf_set_up, t_iekf_set_up, t_eiekf_set_up]=Filter(data_filename,Initial_settings);
        X_ekf_set_all(run_cnt).X=X_ekf_set;
        X_iekf_set_all(run_cnt).X=X_iekf_set;
        X_eiekf_set_all(run_cnt).X=X_eiekf_set;
        run_cnt=run_cnt+1;
        t_update=t_ekf_set_up;
    end
    end
end

Show_result_Multiple(FLAG, REAL_PARA, BIAS_SET, real_path_select,  X_ekf_set_all, X_iekf_set_all, X_eiekf_set_all,t_update);
