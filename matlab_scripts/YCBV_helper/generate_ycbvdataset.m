
clear;clc;

seq='0027';

datapath=['E:\datasets\YCB-V\data3\data\', seq, '\'];

savepath=['E:\datasets\YCB-V\data3\data\',seq,'split\'];

figure()
if exist(savepath)==0 
    mkdir(savepath); 
end
agent_num=3;
overlap=0.8;
sigma_if=0.01*[1,1,1,1,1];

sigma_ij=[0.0001,0.0001,0.0001,0.0001,0.0001];

data_split(datapath, savepath, agent_num, overlap,sigma_ij);

% mm=load("E:\datasets\YCB-V\data1\data\0001split\agent_1_original.mat"     );

%% 提高频率
dti=0.01;
dt_object=0.04; % dt_object必须是dti的整数倍！

sigma_bg=0.001; 
sigma_ba=0.001; 
sigma_a=0.001; %%0.01
sigma_g=0.001;
b_g_real=[0.02,0.01,0.02];%bias 
b_a_real=[0.03,0.05,0.03];
g_real=[0;0;-9.81];
len_of_each_robo=100000;
for ii=1:agent_num
    agentspath=load(fullfile(savepath, sprintf('agent_%d_original.mat', ii)));
    realplow(ii).path=agentspath.opose;
    [Robot(ii).real_path, Robot(ii).T_real0]=High_freq(realplow(ii).path, dti, dt_object);

    Robot(ii).IMU_data=IMU_gendata(Robot(ii).real_path,b_g_real,b_a_real,sigma_g, sigma_a, dti);

    len_of_this_robo=length(Robot(ii).real_path);
    len_of_each_robo=min( len_of_this_robo,  len_of_each_robo);

end

T_WORKING=dti*(len_of_each_robo-1);

% 
%% 相对测量 以及对于物体的测量
% 
A=ones(agent_num,agent_num); %连通图
% G=digraph(A);
% figure();
% plot(G,'LineWidth',2,'EdgeColor','k','NodeColor','r','Layout','auto','ArrowSize',10,'layout','auto');
% 
% 
% 
% 
% 

DEBUG=1;
n=1;
time=0;
m_data=[];
% h=waitbar(0,'time evolution');
while(time <=T_WORKING)
    m_data(n).t=time;
    
    
    for i=1:agent_num
    % 物体测量
       this_pose=realplow(i).path{1,n};
       T_r=eye(4);
       T_r(1:3,:)=this_pose;
       scatter3(T_r(1,4), T_r(2,4), T_r(3,4),10,'blue','filled','o'); hold on
       
       indexes=load(fullfile(savepath, sprintf('agent_%d_index.mat', i)));
       
    
       ind_now=indexes.indexes{1,n}.cls_indexes;
       m_data(n).robot(i).indexlist=ind_now;
       LEN=length(ind_now);
       m_data(n).robot(i).feature_flag=LEN;

       objmea=load(fullfile(savepath, sprintf('agent_%d_object.mat', i)));
       for nn=1:LEN
           Too=eye(4);
           Too(1:3,:)=objmea.objposess{1,n}.poses(:,:,nn);
           m_data(n).robot(i).m_feature(nn).T=(Too);  
           m_data(n).robot(i).m_feature(nn).index=ind_now(nn);  
           
           if(DEBUG)
               if(time==0 )
               T_ob=(T_r)*Too;
               p_pb=T_ob(1:3,4);
               scatter3(p_pb(1),p_pb(2),p_pb(3),77,"red","filled","d"); hold on
               end
         
           end
       end


      % Relative 测量

        neighbors=find(A(i,:));
        num_neighbor=length(neighbors);
        nnn=1;
        m_data(n).robot(i).index=zeros(num_neighbor-1,1);

        for j_index=1:num_neighbor
            j=neighbors(j_index);
            if(j~=i)
                m_data(n).robot(i).index(nnn)=j;

                relarela=load(fullfile(savepath, sprintf('agent_%d_in_%d_relative.mat', j, i)));
                this_rere=eye(4);
                this_rere(1:3,:)=relarela.rpose{1,n};

                m_data(n).robot(i).m_inter(nnn).m=this_rere;
                nnn=nnn+1;
            end
        end

    end
    % 
    % waitbar(n/len_of_each_robo,h);
    time=time+dt_object;
    n=n+1;

end
% delete(h);
%% 路标在世界系
% 暂且选取robot1 第一帧
landmarks=[];
T1_real0=Robot(1).real_path(end).T;
index1=m_data(1).robot(1).indexlist;
len=length(index1);
for pp=1:len
    
    in=index1(pp);
    landmarks(in).T=(T1_real0)*m_data(end).robot(1).m_feature(pp).T;
    scatter3( landmarks(in).T(1,4), landmarks(in).T(2,4),landmarks(in).T(3,4)); hold on
end



landmarks(1).indexinorder=index1;




%% 数据集质量测试

% for i=1:agent_num
%     % 每个agent 能否从对于物体的观测恢复出来自己，以及时间上是否对应
% 
% 
% 
% 
% 
% 
% end












%% Useless
% Parameters related with camera
Eff_Range=100;
sigma_cam=1;  % pixels
scale=1;
fx=460;
fy=460;
cx=255;
cy=255;
image_w=640;
image_h=640;
p_camera_in_IMU_real = [0.0, 0.0, 0.0]/scale;
R_camera_in_IMU_real = angle2dcm(0,0 , 0);

folder='dataset'; 
if exist(folder)==0 
    mkdir(folder); 
end
datasetname='ycbv';
savename=[folder,'/',datasetname,'.mat'];
save(savename);