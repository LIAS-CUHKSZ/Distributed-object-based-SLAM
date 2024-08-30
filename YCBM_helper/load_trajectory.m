clear;clc
%% Path
dataset_root='E:\datasets\YCB-M\';
camera_file(1).name='xtion_annotations\xtion\';
camera_file(2).name= 'realsense_r200_annotations\realsense_r200\';
camera_file(3).name='pico_flexx_annotations\pico_flexx\';
camera_file(4).name='kinect2_annotations\kinect2\';
camera_file(5).name= 'ensenso_annotations\ensenso\';
camera_file(6).name='basler_tof_annotations\basler_tof\';
camera_file(7).name= 'astra_annotations\astra\';
    
sequence_name='009_010_019_035_040';



index_list_all=[009,010,019,035,040];



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


index_seen=[];
num_list=[];




for i=[1,2,5,7]

    path=[dataset_root,camera_file(i).name,sequence_name,'\trajectoryr\'];
    traj=dir([path,'*.json']); 
    num=length(traj);
    num_list(end+1)=num-3; %读取的最后两帧是参数

end
n_data=min(num_list)-1;








figure();


 



for jjj=[1,2,5,7]   %[1,2,3,5,7]
    path=[dataset_root,camera_file(jjj).name,sequence_name,'\trajectoryr\'];
    traj=dir([path,'*.json']); 
    num=length(traj);
    num_list(end+1)=num-3; %读取的最后两帧是参数

    pose_z={}; 
    
   
    jjj
    for i=1:n_data
        
        ob=loadjson([traj(i).folder,'\',traj(i).name]);
        T_camera_now=eye(4);
        translation=ob.camera_data.location_worldframe;
        rotation=ob.camera_data.quaternion_xyzw_worldframe;
        rot_mat=quat2rotm(rotation);
        T_camera_now(1:3,1:3)=rot_mat;
        T_camera_now(1:3,4)=translation';
        T_camera=invT(T_camera_now);
        Robot(jjj).realpath(i).T=T_camera; %%% 机器人在统一世界坐标系之下的位姿
        position(i,:)=T_camera(1:3,4)'; %%%记录位置便于可视化
       
        objectnow=ob.objects;
        num_o=length(objectnow);
        pos_object_list=zeros(1,3);

        % 

        for z=1:num_o
            pose_z{i}=eye(4);
            pose_z{i}(1:3,1:3)=quat2rotm(objectnow(z).quaternion_xyzw);
            pose_z{i}(1:3,4)=objectnow(z).location;
            
            T_object=(T_camera)*(pose_z{i});  %也就是说原始数据是是世界系在相机里面
        
            pos_object_list(z,:)=T_object(1:3,4)'; 
            object(jjj).realpath(i).object(z).T=T_object; %%% 物体在统一世界坐标系之下的位姿

            name_obj=objectnow(z).class;
            index=str2double(name_obj(1:3));

            index_seen=union(index_seen,index);


            % colorvalue=colorlist(find(indexlist==index),:);
            % scatter3(pos_object_list(z,1),pos_object_list(z,2),pos_object_list(z,3),10); hold on %只画每个机器人第一帧的结果
            % 

        end

        axis equal
    end
     
    plot3(position(:,1), position(:,2),position(:,3),'Color',colorlist(jjj,:),'LineWidth',3  );hold on
    % draw the z-axis of the robot and object
   
    for i=1:5:n_data
     
        T=Robot(jjj).realpath(i).T;
        x=T(1:3,1)*20;
        y=T(1:3,2)*20;
        z=T(1:3,3)*20;
        o=T(1:3,4);
        quiver3(o(1),o(2),o(3),x(1),x(2),x(3),0.1,'r','LineWidth',3);hold on
        quiver3(o(1),o(2),o(3),y(1),y(2),y(3),0.1,'g','LineWidth',3);hold on
        quiver3(o(1),o(2),o(3),z(1),z(2),z(3),0.1,'b','LineWidth',3);hold on
        nnn=length(object(jjj).realpath(i).object);
        for z=1:nnn
            T=object(jjj).realpath(i).object(z).T;
            x=T(1:3,1)*20;
            y=T(1:3,2)*20;
            z=T(1:3,3)*20;
            o=T(1:3,4);
            quiver3(o(1),o(2),o(3),x(1),x(2),x(3),0.1,'r','LineWidth',3);hold on
            quiver3(o(1),o(2),o(3),y(1),y(2),y(3),0.1,'g','LineWidth',3);hold on
            quiver3(o(1),o(2),o(3),z(1),z(2),z(3),0.1,'b','LineWidth',3);hold on
        end
    end
 
end




