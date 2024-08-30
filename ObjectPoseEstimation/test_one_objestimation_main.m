%% Simulation For Multi-agent Pose Estimation: Distributed Invariant Kalman Filter Based on Covariance Intersection 
% _In this scripts, we only take the 5 agents case as an example. But we have 
% tried our best to make scripts easy to extend to other scenes like more agents. 
% You can feel free to use this simulation, but please refer to our work._
% 
% When run this code at the first time, you can either choose to use our generated 
% datasets (/dataset) or generate the data by yourself.
% 
% 
% 
% *How to generate dataset?* 
%% 
% * Open file "Scenes". 
% * Run  "</Scenes/generate_scene.mlx generate_scene.mlx>". You can modify the 
% number of agents, trajectory, noise scale, graph structure and so on.
%% 
% *We will compare the following methods:*
%% 
% # Method 1- Dead Reckoning (Control Group):Independent IMU propagation with 
% part of the agent update while seeing the feature. 
% # Method 2-InEKF: Use Invariant Kalman Filter(InEKF) to fuse both environmental 
% measurements and relative measurements.
% # Method 3-InCIKF: the proposed algorithm.
% # Method 4-EKF: Use SO(3)-EKF to fuse both environmental measurements and 
% relative measurements.
%% 
% *Evaluation metrics:* 
% 
% The evaluation results will be saved automatically in file "TestResults/[dataset 
% name/]". The specific evaluation functions are in the file "Evaluation". 
%% 
% # Absolute trajectory error~(ATE) curve and average ATE rmse for each agent's 
% trajectoroies. (</Evaluation/ATE.m ATE.m>)
% # Average ATE rmse of all agents (</Evaluation/ATE_ALL.m ATE_ALL.m>)
%% 
% _Enjoy_.

if(1)
clc;clear;
addpath(genpath(pwd));
datasetname='data2';   
datapath=['dataset\',datasetname,'.mat'];
load(datapath);
%% Initial Settings
% Initialization


N_imu=1;
N_mea=2;
t_update_set=[];
N_update_set=[];
N_ROBOTS=length(Robot);



% Set the initial bias
% remark: we track pimu, Rimu, vimu, ba, bg.
% 
% 

% IMU state initial error for 5 agents (We assume that they have the same bias for simplicity coding)
bia_R_imu_angle=[0,0,0];
bia_R_imu = angle2dcm(bia_R_imu_angle(1), bia_R_imu_angle(2), bia_R_imu_angle(3));
bia_p_imu =[0,0,0];
bia_v_imu = [0,0,0];% zeros(1,3);
bia_b_a   = [0,0,0];
bia_b_g   = [0.00,0.00,0.00];

initializer=[]; %structure to save the initial (X,P)
for i=1:N_ROBOTS
    initializer(i).X_init.Rimu=bia_R_imu*Robot(i).T_real0(1:3, 1:3);
    initializer(i).X_init.pimu=Robot(i).T_real0(1:3, 4)'+bia_p_imu;
    initializer(i).X_init.vimu=Robot(i).IMU_data(N_imu).v_inglobal_real+bia_v_imu;
    initializer(i).X_init.ba=b_a_real + bia_b_a;
    initializer(i).X_init.bg=b_g_real + bia_b_g;
end

P_init=10^-2* eye(15);
% Set noises

SIGMA_SET.sigma_a=sigma_a;
SIGMA_SET.sigma_g=sigma_g;
SIGMA_SET.sigma_bg=sigma_bg;
SIGMA_SET.sigma_ba=sigma_ba;
SIGMA_SET.sigma_cam=sigma_cam;
%% Estimate Now

W_list = [];
TrPlist=[];
MAX_FUSE_O = 10; %对于agent i来说，其所见的一个特征最多融合次数为10次。


IF_VIO_UPDATE=1;
IF_CI=0;
IF_ESTIMATE_OBJ=1;

for i=1:N_ROBOTS          
    X_robots(i).objectlist=[];
    X_robots(i).Tobj=[];
    X_robots(i).objindex=[];
end


    %%-------- INITIALIZE -------------
    N_imu=1;
    N_mea=2;
    N_cam=2;% Initial Count of camera measurement
    CAM_IF_UPDATE=0;% Value 1 means that new camera measurement is received.
    N_update=1;
    t_update_set(N_update)=0;
    N_update_set(N_update)=1;
    for i=1:1  
        X_init=initializer(i).X_init;
        method3(i).X(N_update)=X_init;     
        X_robots(i).X_last(N_imu).X=X_init;
        P_robots(i).P_last(N_imu).P=P_init;   
    end
   
    t_imu=Robot(1).IMU_data(N_imu).t;

    NUM_CAM_UPDATE=zeros(5,1);
    %%-------- MAIN LOOP ---------------
    while(t_imu<T_WORKING)
        t_imu=Robot(1).IMU_data(N_imu).t; % Just use the 1st robot's timestamp
        %%1. 局部VIO
        for i=1:1
            X_last_i=X_robots(i).X_last(N_imu).X;
            P_last_i=P_robots(i).P_last(N_imu).P;
            IMU_data=Robot(i).IMU_data;
            camera_data=Robot(i).camera_data;
            acc_m=IMU_data(N_imu).a;
            w_m=IMU_data(N_imu).w;
            [X_pred, P_pred]=  inEKF_propagate(X_last_i, P_last_i, w_m, acc_m, dti, SIGMA_SET);
            %Note: once saving, should be saved as the next time dat
            X_robots(i).X_last(N_imu+1).X=X_pred;
            P_robots(i).P_last(N_imu+1).P=P_pred;
            if(IF_VIO_UPDATE)
                t_cam=camera_data(N_cam).timestamp;
                if ((abs(t_cam-t_imu)<0.0000001))
                    CAM_IF_UPDATE=1;
                    camera_measure_now=camera_data(N_cam);
                    number_of_features_seen = length(camera_measure_now.uv); 
                    itermax=1;
                    if(number_of_features_seen>6)
                        [X_esti, P_esti]=inEKF_iter_update(X_pred, P_pred, CAM_IF_UPDATE,acc_m,w_m,camera_measure_now, SIGMA_SET, itermax,CAM_INTRISIC,1, p_camera_in_IMU_real, R_camera_in_IMU_real);
                        X_robots(i).X_last(N_imu+1).X=X_esti;
                        P_robots(i).P_last(N_imu+1).P=P_esti;
                        NUM_CAM_UPDATE(i,1)=NUM_CAM_UPDATE(i,1)+1;
                    end
                end           
            end  
        end
        if(CAM_IF_UPDATE==1)
            N_cam=N_cam+1;
            CAM_IF_UPDATE=0;
        end


   
        %%2.fuse neighbors' estimation on robot i
        t_mea=m_data(N_mea).t;
        if(abs(t_imu-t_mea)<0.000000001)% meaning that receiving new measurements.
            N_update=N_update+1; % once receiving new measurements, conducting CI or KF update.
            %% 2.Using CI to fuse the dependent estimates
            method3(i).X(N_update)=X_robots(i).X_last(N_imu+1).X;
            if(IF_CI)
                for i=1:5 % choose robot i.
                    nn=1;
                    real_Neighbornumber=length(find(A(i,:)))-1; % number of neighbors (denote j) of i. (without i itself)
                    est_xi=zeros(6,real_Neighbornumber);
                    P_est_by_j=zeros(6,6,real_Neighbornumber);
                    % --start "namespace: robot i"--
                    
                    for j_index=1:real_Neighbornumber % To get the measurement of j of i.
                        
                       j=m_data(N_mea).robot(i).index(j_index); % which j
                       Robot_mea_now=m_data(N_mea).robot(j); % Take out j's relative observation
                       
                       index_list_j=Robot_mea_now.index;
                       i_index = find(index_list_j==i);
                       X_m_i_in_j=Robot_mea_now.m_inter(i_index).m;
           
                       X_j_prior=eye(4,4);
                       X_j_prior(1:3,1:3)=X_robots(j).X_last(N_imu+1).X(1).Rimu;
                       X_j_prior(1:3,4)=X_robots(j).X_last(N_imu+1).X(1).pimu';
                       X_i_prior=eye(4,4);
                       X_i_prior(1:3,1:3)=X_robots(i).X_last(N_imu+1).X(1).Rimu;
                       X_i_prior(1:3,4)=X_robots(i).X_last(N_imu+1).X(1).pimu';
                       X_i_est_by_j=X_j_prior * X_m_i_in_j;
    
                       inv_X_i_prior = invT(X_i_prior);
                       inv_X_i_est_by_j = invT(X_i_est_by_j); 
                       xi_now=se3_log(X_i_prior * inv_X_i_est_by_j);
                       est_xi(:,nn)=xi_now; 
                          
                       inv_X_i_in_j_mea=invT(X_m_i_in_j);
                       R_ji=sigma_ij(j)*sigma_ij(j)*eye(6);
                       
                        
                       AdxJ = Adx(X_j_prior);
                       Theta =AdxJ * sigma_ij(j) *ones(6,1);
                       H_j = inv_J_left(Theta);
                       % H_j = inv(jacobian_left(Theta));
                       H_Rij = AdxJ;
    
                       P_est_res_now=P_robots(j).P_last(N_imu).P(1:6,1:6)  + H_Rij  *R_ji * H_Rij';
           
                       P_est_by_j(:,:,nn)=(P_est_res_now+P_est_res_now')/2;
                       nn=nn+1;
                        
                    end
                    a=zeros(15,1);
                    cov_A = P_robots(i).P_last(N_imu+1).P;
                    %---first to fuse pseudo i's pose mesurements from neighbors based on fast CI--
                    [xi_CI,P_CI, w]=fastCI(est_xi,P_est_by_j);        
                    b = xi_CI;
                    B = P_CI; 
                    H = zeros(15,6);
                    H(1:6,1:6)=eye(6);
                    %---then fuse with prior estimation by solving the optimization problem-----
                    [c,C,omega]=conventionalCI(a,cov_A,b,B,H');    
                    P_robots(i).P_last(N_imu+1).P=C;    
                    X_CI = se3_exp(-c(1:6,1))*X_i_prior;
                    method3(i).X(N_update).Rimu=X_CI(1:3,1:3);
                    method3(i).X(N_update).pimu=X_CI(1:3,4)';
                    X_robots(i).X_last(N_imu+1).X(1).Rimu=X_CI(1:3,1:3); % if CI, rewrite.
                    X_robots(i).X_last(N_imu+1).X(1).pimu=X_CI(1:3,4)';
                end
            end


            %%4. Consensus on object estimation among neighbors
            
            for i=1:N_ROBOTS
                mea_now=m_data(N_mea).robot(i);
                % --start "namespace: robot i"--
                num_see_feature=m_data(N_mea).robot(i).feature_flag;
                T_mea_feature=m_data(N_mea).robot(i).m_feature; 
                index_i_see=m_data(N_mea).robot(i).indexlist;
                X_update=X_robots(i).X_last(N_imu+1).X;

                if(IF_ESTIMATE_OBJ)
                    if(num_see_feature)% only if seeing the feature, would the robot update

                        %% 对状态的更新增加
                        new_union = union(X_robots(i).objectlist,index_i_see);
                        new_object_list =  setdiff(new_union,X_robots(i).objectlist);
                        X_robots(i).objectlist =new_union;
                        new_union=[];%及时清零，因为下一次还会赋值。否则影响其他机器人
                        if(~isempty(new_object_list))   %如果有之前没有估计过的目标位姿，开启初始化
                            
                            for zz=1:length(new_object_list)
                                %新加入的特征对应的编号
                                new_obj_index=new_object_list(zz);
                            
                                %找到该从未被初始化的特征，在该时刻测量的indexlist是哪个位置
                                index_num = find(index_i_see==new_obj_index);
                                
                                T_i = eye(4);
                                T_i(1:3,1:3)=X_update.Rimu;
                                T_i(1:3,4) = X_update.pimu';
                                X_robots(i).Tobj(end+1).T = T_i * T_mea_feature(index_num).T;
                                X_robots(i).objindex(end+1) = new_obj_index; %请注意这里的顺序是和看到的先后有关的！
                            end
                        end



                        X_pred=X_robots(i).X_last(N_imu+1).X; % after CI
                        P_pred=P_robots(i).P_last(N_imu+1).P;
                        [X_update,P_update]=inEKF_X_iter_update(X_pred, P_pred, landmarks, T_mea_feature, sigma_if(i), num_see_feature,itermax);
                        P_update=(P_update+P_update')/2;
                        % if update, rewrite.
                        X_robots(i).X_last(N_imu+1).X=X_update;
                        X_robots(i).X_last(N_imu+1).X=X_update;
                        P_robots(i).P_last(N_imu+1).P=P_update;
                        method3(i).X(N_update)=X_update;
                        
                    end
                end
                method3(i).X(N_update)=X_update;
                % --end "namespace: robot i"--
                TrPlist(i).P(N_update)=trace(P_robots(i).P_last(N_imu+1).P(1:6,1:6));
            end
            
            
            N_update_set(N_update)=N_imu;
            t_update_set(N_update)=t_mea;
            N_mea=N_mea+1;

        end
        N_imu=N_imu+1;
        CAM_IF_UPDATE=0;
    end
actin=actin+1;
activatedmethod(actin).methodname='DInCIKF';
activatedmethod(actin).result=method3;
end

if(Method(3))
    figure()
    len = size(TrPlist(1).P,2);
    step = 1:len-1;
    
    plot(step, TrPlist(1).P(1,2:end), "DisplayName", 'Robot 1');hold on
    plot(step, TrPlist(2).P(1,2:end), "DisplayName", 'Robot 2');hold on
    plot(step, TrPlist(3).P(1,2:end), "DisplayName", 'Robot 3');hold on
    plot(step, TrPlist(4).P(1,2:end), "DisplayName", 'Robot 4');hold on
    plot(step, TrPlist(5).P(1,2:end), "DisplayName", 'Robot 5');hold on
        xlabel("step");
    ylabel('Trace of Covariance');
    legend
end
% 
%  The real path all will be used in evaluation.

real_path_all=[];
for i=1:N_ROBOTS
    real_path=Robot(i).real_path;
    real_path_select = filterArrayByIndex(real_path,N_update_set);
    real_path_all(i).rp=real_path_select;
end
% Evaluation and Plot




if(Method(1))
disp("method 1");
ATE(real_path_all, method1, landmarks, Eff_Range,1,datasetname);
end

if(Method(2))
disp("method 2");
ATE(real_path_all, method2, landmarks, Eff_Range,2,datasetname);
end

if(Method(3))
disp("method 3");
ATE(real_path_all, method3, landmarks, Eff_Range,3,datasetname);
end

if(Method(4))
    disp("method 4");
    ATE(real_path_all, method4, landmarks, Eff_Range,4,datasetname);
end
ATE_ALL(real_path_all,activatedmethod,actin,datasetname);

if(Method(5))
    disp("method 5");
    ATE(real_path_all, method5, landmarks, Eff_Range,5,datasetname);
end
ATE_ALL(real_path_all,activatedmethod,actin,datasetname);
% Save the estimated trajectory for video 
% 把保存下来的轨迹转化成(x,y,z)的list

positions=turn_structure2xyz(method3);

save('save_video_helper/data/data.mat', 'positions','real_path_all','landmarks','method3');