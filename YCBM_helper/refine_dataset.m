clear;clc
%% Path
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



A=ones(4,4);

indexlist=[002;003;004;005;006;007;008;009;010;011;019;021;024;025;035;036;037;040;051;051;061];

colorlist=[1 0 0;
    0 1 0;
    0 0 1;
    0 1 1;
    1 0 1;
    1 1 0;
    0 0 0;
    0 0.4470 0.7410;
    0.8500 0.3250 0.0980;
    0.9290 0.6940 0.1250;
    0.4940 0.1840 0.5560;
    0.4660 0.6740 0.1880;
    0.3010 0.7450 0.9330;
    0.6350 0.780 0.1840;
    0.7 0 1;
    0.4,0.4,0.9;
    0.9,0.5,1;
    0.6,0,0.6;
    1,0.3,0.7;
    0.4, 0.5,0.3;
    0,0.7,0.6;
    1,0.5,0.2];


%% 参数
dti=0.01;
dt_object=0.1; % dt_object必须是dti的整数倍！

b_g_real=[0.02,0.01,0.02];%bias 
b_a_real=[0.03,0.05,0.03];
g_real=[0;0;-9.81];

sigma_ij=[0.0001,0.0001,0.0001,0.0001,0.0001];
sigma_if=0.02*[1,1,1,1,1];



m_data=[];
object_cams=[];
index_seen=[];
R=0.1*eye(6);

for mmm=1:5
    object_cams(mmm).Tlist=[];

end

Realplow=[];
object=[];
P=[0,0,1;1,0,0;0,-1,0]; %%% Permutation matrix
num_list=[];


i=1;

%% 确定数据集长度
for i=[1,2,3,4,5]

    path=[dataset_root,camera_file(i).name,sequence_name,'\trajectory\'];
    traj=dir([path,'*.json']); 
    num=length(traj);
    num_list(end+1)=num-3; %读取的最后两帧是参数

end
n_data=min(num_list);
T_WORKING=dt_object*(n_data-1);



%% 确定物体的唯一世界系位姿
% 
for jjj=[1]   %[1,2,3,5,7]
    path=[dataset_root,camera_file(jjj).name,sequence_name,'\trajectory\'];
    traj=dir([path,'*.json']); 
    num=length(traj);

    for i=1
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


    end

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

end


% 
reslist=[];
%% 读取各个相机数据, 重写
for i=[1,2,3,4,5]
    
    NUM_effective=1;
    path=[dataset_root,camera_file(i).name,sequence_name,'\trajectory\'];
    traj=dir([path,'*.json']); 

    foldere=[dataset_root,camera_file(i).name,sequence_name,'\trajectoryr\'];
 
    if (exist(foldere))

        rmdir(foldere,'s');
        mkdir(foldere)
   
    else

        mkdir(foldere); 
    end


    num=length(traj);
    ne=1;
    pos_list=[];
    for n=1
        ob=loadjson([traj(n).folder,'\',traj(n).name]);
        

        objectnow=ob.objects;
        num_o=length(objectnow);
        
       
        index_list=[];
        for z=1:1
            pose_z=eye(4);
            pose_z(1:3,1:3)=quat2rotm(objectnow(z).quaternion_xyzw);
            pose_z(1:3,4)=objectnow(z).location;
            name_obj=objectnow(z).class;
            index=str2double(name_obj(1:3));
            
            where=find(index_order==index);
            objreal=landmarks(where).T;
            Tr=objreal * invT(pose_z);
            
            n_if=randn(6,1)*sigma_if(1);
            
            ob_in_cam=invT(Tr)*se3_exp(n_if)*objreal;

            ob.objects(z).location=ob_in_cam(1:3,4)';
            ob.objects(z).quaternion_xyzw=rotm2quat( ob_in_cam(1:3,1:3) );

            index_list(end+1)=index;         

        end

        Tlast=Tr;

    
        for z=2:num_o
            % 把其他物体也优化了吐了           
            name_obj=objectnow(z).class;
            index=str2double(name_obj(1:3));

            where=find(index_order==index);
            objreal=landmarks(where).T;
            n_if=randn(6,1)*sigma_if(1);
       
            % ob_in_cam=invT(Tr) *se3_exp(n_if)* objreal;
            ob_in_cam=invT(Tr) * objreal;
            ob.objects(z).location=ob_in_cam(1:3,4)';
            ob.objects(z).quaternion_xyzw=rotm2quat( ob_in_cam(1:3,1:3) );
            ob.objects(z).class=name_obj;

            pose_z=eye(4);
            pose_z(1:3,1:3)=quat2rotm(objectnow(z).quaternion_xyzw);
            pose_z(1:3,4)=objectnow(z).location;
            

        end


        Tri=invT(Tr);
        pos_list(1,:)=Tr(1:3,4)'; 
        ob.camera_data.location_worldframe=Tri(1:3,4)';
        ob.camera_data.quaternion_xyzw_worldframe=rotm2quat(Tri(1:3,1:3));
        objson=jsonencode(ob);

        fi=[foldere,traj(n).name];
        filele= fopen(fi, 'w+');
        fprintf(filele, '%s',objson);
        fclose(filele);

    end



 


    for n=2:num-3
        ob=loadjson([traj(n).folder,'\',traj(n).name]);
        

        objectnow=ob.objects;
        num_o=length(objectnow);
        
       
        index_list=[];
        for z=1:1
            pose_z=eye(4);
            pose_z(1:3,1:3)=quat2rotm(objectnow(z).quaternion_xyzw);
            pose_z(1:3,4)=objectnow(z).location;
            name_obj=objectnow(z).class;
            index=str2double(name_obj(1:3));
  
            where=find(index_order==index);
            objreal=landmarks(where).T;
            
            Tr=objreal * invT(pose_z);
             
            n_if=randn(6,1)*sigma_if(1);
            
            ob_in_cam=invT(Tr)*se3_exp(n_if)*objreal;

            ob.objects(z).location=ob_in_cam(1:3,4)';
            ob.objects(z).quaternion_xyzw=rotm2quat( ob_in_cam(1:3,1:3) );

            index_list(end+1)=index;         
    
           
        end
        res=norm(so3_log(Tr(1:3,1:3) * (Tlast(1:3,1:3)') ));
        dis=norm(Tr(1:3,4)-Tlast(1:3,4));
        reslist(ne, i)=dis;

     
        Tlast=Tr;
        ne=ne+1;

       
        for z=2:num_o
            % 把其他物体也优化了吐了           
            name_obj=objectnow(z).class;
            index=str2double(name_obj(1:3));
            where=find(index_order==index);
            objreal=landmarks(where).T;
            n_if=randn(6,1)*sigma_if(1);
       
            ob_in_cam=invT(Tr) *se3_exp(n_if)* objreal;
            ob.objects(z).location=ob_in_cam(1:3,4)';
            ob.objects(z).quaternion_xyzw=rotm2quat( ob_in_cam(1:3,1:3) );
           

        
        end
        
        if (res<=pi/12 && dis<=8.0)
            Tri=invT(Tr);
            pos_list(NUM_effective,:)=Tr(1:3,4)'; 
            ob.camera_data.location_worldframe=Tri(1:3,4)';
            ob.camera_data.quaternion_xyzw_worldframe=rotm2quat(Tri(1:3,1:3));
            objson=jsonencode(ob);
            
            % foldere=[traj(n).folder,'r\'];

            fi=[foldere,num2str(NUM_effective,'%06d'),'.json'];
            filele= fopen(fi, 'w+');
            fprintf(filele, '%s',objson);
            fclose(filele);
            NUM_effective=NUM_effective+1;
        end
    end
    NUM_effective
    plot3(pos_list(:,1), pos_list(:,2),pos_list(:,3)); hold on
end

reslist=reslist(1:n_data-1,:);
max(reslist)
mean(reslist)
% % datasetname='ycbm';
% savename=[folder,'/',datasetname,'.mat'];
% save(savename);
% 
% 
