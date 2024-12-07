function [] = ATE_ALL(real_path_all,activatedmethod,actin,datasetname, landmarks, comparelocal)
% function [] = ATE_ALL(real_path_all,activatedmethod,actin,datasetname)
Line=[];
%for each trajectory
if(~comparelocal)
for j=1:actin
    X=activatedmethod(j).result;
    method_name = activatedmethod(j).methodname;
    n=length(real_path_all); % n agents
    results=[];
    R_err_list=zeros(n,1);
    p_err_list=zeros(n,1);
    % for n agents
    for i=1:n
        real_path=real_path_all(i).rp;
        estX=X(i).X;
        sz_RealPath=length(real_path); 
        real_p_imu=zeros(sz_RealPath,3);
        for m=1:sz_RealPath
            real_p_imu(m,:)=real_path(m).T(1:3,4)';
        end
        [T_IMU_est, p_IMU_est]=get_estimator_T(estX);
        [Err_IMU_R, Err_IMU_p]=cal_norm_error(T_IMU_est, real_path);
        results(i).p_IMU_est=p_IMU_est;
        results(i).Err_IMU_R=Err_IMU_R;
        results(i).Err_IMU_p=Err_IMU_p;
    end

    % 把所有机器人的误差取norm
    rmsep=zeros(sz_RealPath,1);
    rmser=zeros(sz_RealPath,1);
    for s = 1:sz_RealPath   % s 代表路径哪一步"time step"
        all_robot_error_posi_now = zeros(n,1);
        all_robot_error_rot_now = zeros(n,1);
        for rob = 1:n
            
            all_robot_error_posi_now(rob)=results(rob).Err_IMU_p(s);
            all_robot_error_rot_now(rob)=results(rob).Err_IMU_R(s);
        end
        norm_all_robot_error_posi_now = norm(all_robot_error_posi_now);
        norm_all_robot_error_rot_now = norm(all_robot_error_rot_now);
        rmsep(s) = norm_all_robot_error_posi_now;
        rmser(s) = norm_all_robot_error_rot_now;

    end

    Line(j).position=rmsep;
    Line(j).rotation=rmser;
    Line(j).methodname=method_name;
end

% colorlist=[255,194,075;246,111,105;021,151,165]/256;
% colorlist=[250,127,111; 255,190,122; 142,207,201; 128,124,125;216,179,101    ]/256;



colorlist=[233,15,68; 
    255,200,57; 
    99,173,238;
    171,198,228;
    128,128,128]/256;


co=1;
% draw position results
figure()
title('Average RMSE for all robots');
step=length( real_p_imu(:,1));
subplot(2,1,1);
for j = 1:actin
    plot(1:step, Line(j).position, 'DisplayName',Line(j).methodname,'LineWidth',1.6,'Color',[colorlist(j,1),colorlist(j,2),colorlist(j,3)]);hold on
end
legend();

ylabel('Position (m)');
%------------
subplot(2,1,2);
for j = 1:actin
    plot(1:step, Line(j).rotation, 'DisplayName',Line(j).methodname,'LineWidth',1.6,'Color',[colorlist(j,1),colorlist(j,2),colorlist(j,3)]);hold on
end
ylabel('Rotation (rad)')
xlabel('Steps')
legend();

f1=gcf;
folder=['TestResults/',datasetname,'/']; 
name = [datasetname,'rmse.pdf'];

savepath1 = [folder,name];
if exist(folder)==0 
    mkdir(folder); 
end
exportgraphics(f1,savepath1)

else
    j=actin;
    X=activatedmethod(j).result;
    method_name = activatedmethod(j).methodname;
    n=length(real_path_all); % n agents
    results=[];
    R_err_list=zeros(n,1);
    p_err_list=zeros(n,1);
    % for n agents
    for i=1:n
        real_path=real_path_all(i).rp;
        estX=X(i).X;
        sz_RealPath=length(real_path); 
        real_p_imu=zeros(sz_RealPath,3);
        for m=1:sz_RealPath
            real_p_imu(m,:)=real_path(m).T(1:3,4)';
        end
        [T_IMU_est, p_IMU_est]=get_estimator_T(estX);
        [Err_IMU_R, Err_IMU_p]=cal_norm_error(T_IMU_est, real_path);
        results(i).p_IMU_est=p_IMU_est;
        results(i).Err_IMU_R=Err_IMU_R;
        results(i).Err_IMU_p=Err_IMU_p;
    end

    % 把所有机器人的误差取norm
    rmsep=zeros(sz_RealPath,1);
    rmser=zeros(sz_RealPath,1);
    for s = 1:sz_RealPath   % s 代表路径哪一步"time step"
        all_robot_error_posi_now = zeros(n,1);
        all_robot_error_rot_now = zeros(n,1);
        for rob = 1:n
            
            all_robot_error_posi_now(rob)=results(rob).Err_IMU_p(s);
            all_robot_error_rot_now(rob)=results(rob).Err_IMU_R(s);
        end
        norm_all_robot_error_posi_now = norm(all_robot_error_posi_now);
        norm_all_robot_error_rot_now = norm(all_robot_error_rot_now);
        rmsep(s) = norm_all_robot_error_posi_now;
        rmser(s) = norm_all_robot_error_rot_now;

    end

    Line(j).position=rmsep;
    Line(j).rotation=rmser;
    Line(j).methodname=method_name;

colorlist=[233,15,68; 
    255,200,57; 
    99,173,238;
    171,198,228;
    128,128,128]/256;
    for j=1:actin-1
        X=activatedmethod(j).result;
        method_name = activatedmethod(j).methodname;
        n=length(real_path_all); % n agents
        results=[];
        R_err_list=zeros(n,1);
        p_err_list=zeros(n,1);
        % for n agents
        for i=1:n
            real_path=real_path_all(i).rp;
            estX=X(i).X;
            sz_RealPath=length(real_path); 
            real_p_imu=zeros(sz_RealPath,3);
            for m=1:sz_RealPath
                real_p_imu(m,:)=real_path(m).T(1:3,4)';
            end
            [T_IMU_est, p_IMU_est]=get_estimator_T(estX);
            [Err_IMU_R, Err_IMU_p]=cal_norm_error(T_IMU_est, real_path);
            results(i).p_IMU_est=p_IMU_est;
            results(i).Err_IMU_R=Err_IMU_R;
            results(i).Err_IMU_p=Err_IMU_p;
        end
    
        % 把所有机器人的误差取norm
        rmsep=zeros(sz_RealPath,1);
        rmser=zeros(sz_RealPath,1);
        for s = 1:sz_RealPath   % s 代表路径哪一步"time step"
            all_robot_error_posi_now = zeros(n,1);
            all_robot_error_rot_now = zeros(n,1);
            for rob = 1:n
                
                all_robot_error_posi_now(rob)=results(rob).Err_IMU_p(s);
                all_robot_error_rot_now(rob)=results(rob).Err_IMU_R(s);
            end
            norm_all_robot_error_posi_now = norm(all_robot_error_posi_now);
            norm_all_robot_error_rot_now = norm(all_robot_error_rot_now);
            rmsep(s) = norm_all_robot_error_posi_now;
            rmser(s) = norm_all_robot_error_rot_now;
    
        end
    
        Line(j).position=rmsep;
        Line(j).rotation=rmser;
        Line(j).methodname=method_name;
    end
    

    co=1;
    % draw position results
    figure()
    title('Average RMSE for all robots');
    step=length( real_p_imu(:,1));

    subplot(3,1,1);
    j=actin;
    
    plot(1:step, Line(j).position, 'DisplayName',Line(j).methodname,'LineWidth',1.6,'Color',[colorlist(j,1),colorlist(j,2),colorlist(j,3)],'LineStyle','-.');hold on
    legend();ylabel('Position (m)');xlabel('Steps')
    subplot(3,1,2);
    for j = 1:actin-1
        plot(1:step, Line(j).position, 'DisplayName',Line(j).methodname,'LineWidth',1.6,'Color',[colorlist(j,1),colorlist(j,2),colorlist(j,3)]);hold on
    end
    
    legend();
    
    ylabel('Position (m)');
    %------------
    subplot(3,1,3);
    for j = 1:actin-1
        plot(1:step, Line(j).rotation, 'DisplayName',Line(j).methodname,'LineWidth',1.6,'Color',[colorlist(j,1),colorlist(j,2),colorlist(j,3)]);hold on
    end
    ylabel('Rotation (rad)')
    xlabel('Steps')
    legend();
    

%% Plot average estimation results on object pose

for j=1:actin
    %五个萝卜头
    This=activatedmethod(j).result;
    for i=1:n
        Tobj = This(i).Xobj;
        obj_inorder=This(i).obj_inorder;
        
        len=length(obj_inorder);
        for zzz=1:len

            realobj(zzz).T=landmarks(obj_inorder(zzz)  ).T;

        end
        if(~isempty(Tobj))
            [Err_obj_R, Err_obj_p]=cal_norm_error(Tobj, realobj);
            err_normR(i)=norm(Err_obj_R);
            err_normp(i)=norm(Err_obj_p);
        end
    end
    if(~isempty(Tobj))
        method_normR(j)=norm(err_normR);
        method_normp(j)=norm(err_normp);
        disp('~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~')
        disp(activatedmethod(j).methodname);
        Roterr=method_normR(j)
        Poserr=method_normp(j)
    end
end










    f1=gcf;
    folder=['TestResults/',datasetname,'/']; 
    name = [datasetname,'rmse.pdf'];
    
    savepath1 = [folder,name];
    if exist(folder)==0 
        mkdir(folder); 
    end
    exportgraphics(f1,savepath1)






end


end





