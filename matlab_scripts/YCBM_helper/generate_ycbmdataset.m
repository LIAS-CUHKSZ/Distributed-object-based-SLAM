clear;clc
%% 数据读取
% dataset_root='E:\datasets\YCB-M\';
% camera_file(1).name='xtion_annotations\xtion\';
% camera_file(2).name= 'realsense_r200_annotations\realsense_r200\';
% % camera_file(3).name= 'ensenso_annotations\ensenso\';
% camera_file(3).name= 'astra_annotations\astra\';
% 


dataset_root='E:\datasets\YCB-M\';
camera_file(1).name='xtion_annotations\xtion\';
camera_file(2).name= 'realsense_r200_annotations\realsense_r200\';
camera_file(3).name='pico_flexx_annotations\pico_flexx\';
% camera_file(4).name='kinect2_annotations\kinect2\';
camera_file(4).name= 'ensenso_annotations\ensenso\';
% camera_file(6).name='basler_tof_annotations\basler_tof\';
camera_file(5).name= 'astra_annotations\astra\';
    



sequence_name='006_008_011_021_035_037_052';
seq=split(sequence_name,'_');
num_ooo=length(seq);
index_list_all=zeros(num_ooo,1);
for miao=1:num_ooo
    index_list_all(miao)=str2num( seq{miao,1});
end

agent_num=5;
A=ones(agent_num,agent_num);


%% 参数
dti=0.01;
dt_object=0.50; % dt_object必须是dti的整数倍！
sigma_bg=0.001; 
sigma_ba=0.001; 
sigma_a=0.001; %%0.01
sigma_g=0.001;
b_g_real=[0.02,0.01,0.02];%bias 
b_a_real=[0.03,0.05,0.03];
g_real=[0;0;-9.81];

sigma_ij=[0.0001,0.0001,0.0001,0.0001,0.0001];
sigma_if=0.1*[1,1,1,1,1];


%% 初始化存储结构
m_data=[];
Realplow=[];
landmarks=[];


%% 确定数据集长度
num_list=zeros(3,1);
for i=[1,2,3,4,5]
    path=[dataset_root,camera_file(i).name,sequence_name,'\trajectoryr\'];
    traj=dir([path,'*.json']); 
    num=length(traj);
    num_list(i)=num-10; %读取的最后两帧是参数

end
n_data=min(num_list);
T_WORKING=dt_object*(n_data-2);


%% landmarks真值绘制
path=[dataset_root,camera_file(1).name,sequence_name,'\trajectory\'];
traj=dir([path,'*.json']); 
num=length(traj);

i=1;
ob=loadjson([traj(i).folder,'\',traj(i).name]);
T_camera_now=eye(4);
translation=ob.camera_data.location_worldframe;
rotation=ob.camera_data.quaternion_xyzw_worldframe;
rot_mat=quat2rotm(rotation);
T_camera_now(1:3,1:3)=rot_mat;
T_camera_now(1:3,4)=translation';
T_camera=invT(T_camera_now);

objectnow=ob.objects;
num_o=length(objectnow);
pos_object_list=zeros(1,3);
% 
index_order=[];
for z=1:num_o
    pose_now=eye(4);
    pose_now(1:3,1:3)=quat2rotm(objectnow(z).quaternion_xyzw);
    pose_now(1:3,4)=objectnow(z).location;          
    T_object=(T_camera)*(pose_now);  %也就是说原始数据是是世界系在相机里面

    pos_object_list(z,:)=T_object(1:3,4)'; 
    name_obj=objectnow(z).class;
    index=str2double(name_obj(1:3));
    landmarks(z).index=index;
    landmarks(z).T=T_object;
    index_order(end+1)=index;
end

landmarks(1).indexinorder=index_order;
% draw the z-axis of the robot and object
nnn=length(index_order);
for z=1:nnn
    T=landmarks(z).T;
    x=T(1:3,1)*20;
    y=T(1:3,2)*20;
    z=T(1:3,3)*20;
    o=T(1:3,4);
    quiver3(o(1),o(2),o(3),x(1),x(2),x(3),0.1,'r','LineWidth',3);hold on
    quiver3(o(1),o(2),o(3),y(1),y(2),y(3),0.1,'g','LineWidth',3);hold on
    quiver3(o(1),o(2),o(3),z(1),z(2),z(3),0.1,'b','LineWidth',3);hold on
end


for i=1:agent_num
    Realplow(i).realpath=[];
end
%% 读取低频数据, 以及观测数据
time=0;
for n=1:n_data
    m_data(n).t=time;
   
    for i=1:agent_num
        % 读取对应agent在对应时刻的数据
        path=[dataset_root,camera_file(i).name,sequence_name,'\trajectoryr\'];
        ob=loadjson([path,num2str(n,'%06d'),'.json']);

        %记录低频位姿
        T_camera_now=eye(4);
        translation=ob.camera_data.location_worldframe;
        rotation=ob.camera_data.quaternion_xyzw_worldframe;
        rot_mat=quat2rotm(rotation);
        T_camera_now(1:3,1:3)=rot_mat;
        T_camera_now(1:3,4)=translation';
        T_r=invT(T_camera_now);
        Realplow(i).realpath(n).T=T_r; %%% 机器人在统一世界坐标系之下的位姿
        position(n,:)=T_r(1:3,4)'; %%%记录位置便于可视化 

        %记录对于物体的观测
        objectnow=ob.objects;
        num_o=length(objectnow);
     
        m_data(n).robot(i).feature_flag=0;
        m_data(n).robot(i).indexlist=[];

        for z=1:num_o
            pose_z=eye(4);
            pose_z(1:3,1:3)=quat2rotm(objectnow(z).quaternion_xyzw);
            pose_z(1:3,4)=objectnow(z).location;
            name_obj=objectnow(z).class;
            index=str2double(name_obj(1:3));

            m_data(n).robot(i).indexlist(end+1)=index;
            m_data(n).robot(i).m_feature(z).T=(pose_z);
            m_data(n).robot(i).m_feature(z).index=index;
            m_data(n).robot(i).feature_flag=m_data(n).robot(i).feature_flag+1;
                  
        end     

        

       % 相对测量
        neighbors=find(A(i,:)); % find() will return the non-zero index of the vector, which is the neighbor in the Adjaceny matrix.
        num_neighbor=length(neighbors);
        nnn=1;
        m_data(n).robot(i).index=zeros(num_neighbor-1,1);
       
        for j_index=1:num_neighbor
            j=neighbors(j_index);
            if(j~=i)
                % load j
                jpath=[dataset_root,camera_file(j).name,sequence_name,'\trajectoryr\'];
                filename=[jpath,num2str(n,'%06d'),'.json'];
                j_file=loadjson(filename);
                T_camera_nowj=eye(4);
                translation=j_file.camera_data.location_worldframe;
                rotation=j_file.camera_data.quaternion_xyzw_worldframe;
                rot_mat=quat2rotm(rotation);
                T_camera_nowj(1:3,1:3)=rot_mat;
                T_camera_nowj(1:3,4)=translation';
                Tj=invT(T_camera_nowj);

                m_data(n).robot(i).index(nnn)=j;
                n_ij=randn(6,1)*sigma_ij(i);
                m_data(n).robot(i).m_inter(nnn).m=invT(T_r)*se3_exp(n_ij)*Tj;
                nnn=nnn+1;
            end
        end
    end

    time=time+dt_object;
end


%% IMU数据，插值
for i=1:agent_num
    [Robot(i).real_path, Robot(i).T_real0]=High_freqT(Realplow(i).realpath, dti, dt_object);  % 自适应插值根据两个相差多少来差值
    Robot(i).IMU_data=IMU_gendata(Robot(i).real_path,b_g_real,b_a_real,sigma_g, sigma_a, dti);
end


%% 数据测试
for i=1:agent_num
    
    n_imu=1;
    n_mea=1;

    
    timu=Robot(i).real_path(n_imu).t;
    
    tmea=m_data(n_mea).t;

    while(timu<T_WORKING)
        pose_now=Robot(i).real_path(n_imu).T;
        if(abs(timu-tmea)<0.000001)
            
            mea=m_data(n_mea).robot(i);

            no=mea.feature_flag;
            poslis=zeros(no,3);
            for qq=1:no
                Tob=pose_now*mea.m_feature(qq).T;
                poslis(qq,:)=Tob(1:3,4)';
  
            end
            n_mea=n_mea+1;
            tmea=m_data(n_mea).t;
            scatter3(poslis(:,1), poslis(:,2), poslis(:,3),'*' ); hold on
        end
        n_imu=n_imu+1;
        timu=Robot(i).real_path(n_imu).t;

    end

end







%% 保存数据集
folder='dataset'; 
if exist(folder)==0 
    mkdir(folder); 
end


datasetname='ycbmr5';
savename=[folder,'/',datasetname,'.mat'];
save(savename);

