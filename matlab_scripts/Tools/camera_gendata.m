% camera_gendata: to generate camera data (u,v)
% Inputs: CAM_INTRISIC - instrisic of camera
%         lm_camera - camera landmark position list
%         t - duration (s) of the data    
%         p_camera_in_IMU_real,R_camera_in_IMU_real- extrinsic parameter.
%               (position and rotation of camera in the IMU frame) 
%         dt - sample interval of the camera
%         tc - latency of the arrival of camera data compared with the IMU.
%         Position0 - initial position
%         trigK - trajectory shape
%         num - # of robot
%Outputs: camera_data - structure: index(corresponding to the landmark);
%                                   landmark_G (corresponding landmark position in the global frame)
%                                   uv: after pinhole projection (from 3D to 2D)
% 


function [camera_data] = camera_gendata(CAM_INTRISIC,lm_camera,t,p_camera_in_IMU_real,R_camera_in_IMU_real,dt, Position0,trigK)
    feature_in_camera=[];
    time=0;
    time_sp=0;
    nn=1;
    ref_z=[0;0;1];
    
    FOV=CAM_INTRISIC.FOV;
    sigma_cam=CAM_INTRISIC.sigma_cam;
  
    fx=CAM_INTRISIC.fx;
    fy=CAM_INTRISIC.fy; 
    cx=CAM_INTRISIC.cx;
    cy=CAM_INTRISIC.cy; 
    image_w=CAM_INTRISIC.image_w;
    image_h=CAM_INTRISIC.image_h;

    lm_num_list=[];
    while(time<=t+0.5)
        [~,~,~,T_imu] = get_tria_point(time, Position0, trigK);
        %T_imu=gen_trajectory_point(time,scale);
        feature_in_camera(nn).t=time;
        feature_in_camera(nn).plist=zeros(length(lm_camera),3);
        camera_data(nn).timestamp=time_sp;
        j=1;

            T_cam_in_imu= eye(4);
            T_cam_in_imu(1:3,1:3)=R_camera_in_IMU_real;  T_cam_in_imu(1:3,4)=p_camera_in_IMU_real';
        for i=1:length(lm_camera)
     
            T_cam_in_G=T_imu * T_cam_in_imu;
            inv_T_cinG=eye(4);
            inv_T_cinG(1:3, 1:3)=T_cam_in_G(1:3,1:3)';
            inv_T_cinG(1:3,4)=  -T_cam_in_G(1:3,1:3)' *  T_cam_in_G(1:3,4);


            P1_f_in_g=zeros(4,1);
            P1_f_in_g(1:3,1)=lm_camera(i,:)';
            P1_f_in_g(4,1)=1;
 
            P1_f_in_c=inv_T_cinG*P1_f_in_g;

            this_f_in_cam=P1_f_in_c(1:3,1);

            %this_f_in_cam=(T_imu(1:3,1:3)*R_camera_in_IMU_real)'*(lm_camera(i,:)'-T_imu(1:3,4))-R_camera_in_IMU_real'*p_camera_in_IMU_real';
            x=this_f_in_cam(1);
            y=this_f_in_cam(2);
            z=this_f_in_cam(3);
           
            if (z >0)
       
                direction=ref_z;
                d_feature2cam= this_f_in_cam;
                theta=acos( dot( direction,  d_feature2cam)  /(norm(direction)  * norm(d_feature2cam)     )    );    

                u=fx*x/z+cx;
                v=fy*y/z+cy;

               % if(theta <= FOV && u<=image_w && u>0 && v<=image_h && v>0) %%rad
               if(theta <= FOV ) 
                    pic=[fx*x/z+cx,fy*y/z+cy];
                    n_cam=sigma_cam*randn(1,2);
                    h=pic+n_cam;
                    camera_data(nn).uv(j,:)=h;
                    camera_data(nn).landmark_G(j,:)=lm_camera(i,:);
                    camera_data(nn).index(j)=i;
                    j=j+1;
                    Q=theta/pi *180;
                    
                end     
            end
        end
        lm_num_list(nn)=j-1;
        
        time=time+dt;
        time_sp=time_sp+dt;
        nn=nn+1;

    end
        disp('landmark number mean:'); disp(num2str(mean(lm_num_list)));
    end
