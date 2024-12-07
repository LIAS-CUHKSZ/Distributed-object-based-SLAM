function [feature_in_lidar] = LiDAR_gendata(lm_lidar, sigma_lid,sigma_tl,t,p_lidar_in_IMU_real,R_lidar_in_IMU_real,dt, tl, p0,k)
%用来生成LiDAR的测量值  


    feature_in_lidar=[];
    time=tl;
    time_sp=0;
    nn=1;
    while(time<=t)
        [~,~,~,T_imu]=get_tria_point(time,p0,k);
        feature_in_lidar(nn).t=time;
        feature_in_lidar(nn).plist=zeros(length(lm_lidar),3);
        feature_in_lidar(nn).timestamp=time_sp;
        for i=1:length(lm_lidar)
            %T_f=[1,0,0,lm_lidar(i,1);0,1,0,lm_lidar(i,2);0,0,1,lm_lidar(i,3);0,0,0,1];
            %T_finlid=T_lidar'*T_f;
            p_finlid=(T_imu(1:3,1:3)*R_lidar_in_IMU_real)'*(lm_lidar(i,:)'-T_imu(1:3,4))-R_lidar_in_IMU_real'*p_lidar_in_IMU_real';
            feature_in_lidar(nn).plist(i,:)=p_finlid;
            n_lid=sigma_lid*randn(1,3);
            feature_in_lidar(nn).p_m(i,:)=feature_in_lidar(nn).plist(i,:)+n_lid;
        end
        n_il=sigma_tl*randn(1);
        time=time+dt+n_il;
        time_sp=time_sp+dt;
        nn=nn+1;
    end

end