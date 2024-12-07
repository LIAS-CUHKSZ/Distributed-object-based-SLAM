function [] = ATE(real_path_all, X, landmarks, Eff_Range,num_method,datasetname)
%function [] = ATE(real_path_all, X)
%ATE Calculate ATE between real path and the estimated results.
%Input: 
% real_path_all: real path for each agent
% X: estimated result



n=length(real_path_all); %n agents
results=[];

%% figure 1. x,y,z trajectory.
figure();

R_err_list=zeros(n,1);
p_err_list=zeros(n,1);
for i=1:n
    real_path=real_path_all(i).rp;
    estX=X(i).X;
    sz_RealPath=length(real_path); 
    real_p_imu=zeros(sz_RealPath,3);
    for m=1:sz_RealPath
        real_p_imu(m,:)=real_path(m).T(1:3,4)';
    end

   time=X.t;
   
    [T_IMU_est, p_IMU_est]=get_estimator_T(estX);
    [Err_IMU_R, Err_IMU_p]=cal_norm_error(T_IMU_est, real_path);
    results(i).p_IMU_est=p_IMU_est;
    results(i).Err_IMU_R=Err_IMU_R;
    results(i).Err_IMU_p=Err_IMU_p;
    
    %x-y-z following trajectories
    step=length( real_p_imu(:,1));
%     stepx=length(p_IMU_est(:,1));
    disp('--------------/RMSE/-------------')
    disp('Robots ')
    i
    disp('Orientation(rad): ')
    fprintf('Err_IMU_R | ');  disp(num2str(mean(Err_IMU_R(1:end))));
    disp('Position(m):')
    fprintf('Err_IMU_p | ');disp(num2str(mean(Err_IMU_p(1:end))));
    

    subplot(3,n,i);
    h{1}=plot(1:step,  real_p_imu(:,1),'DisplayName','Groundtruth'); hold on
    h{2}=plot(1:length(p_IMU_est(:,1)),  p_IMU_est(:,1),'LineWidth',2,'DisplayName','Estimate','LineStyle','--'); hold on
    xlabel('steps')
    ylabel('x-position (m)')
    title(['Robot',num2str(i)]);

    subplot(3,n,i+n);
    plot(1:step,  real_p_imu(:,2)); hold on
    plot(1:step,  p_IMU_est(:,2),'LineStyle','--'); hold on
    xlabel('steps')
    ylabel('y-position (m)')

    g=subplot(3,n,i+2*n);
   h1= plot(1:step,  real_p_imu(:,3)); hold on
    h2=plot(1:step,  p_IMU_est(:,3),'LineStyle','--'); hold on   
    xlabel('steps')
    ylabel('z-position (m)')
   
    R_err_list(i,1) = mean(Err_IMU_R(1:end));
    p_err_list(i,1) = mean(Err_IMU_p(1:end)); 
end

legend();
disp('============/Average RMSE/===========')
disp('Average Rotation RMSE among Robots: ')
disp(num2str(mean(R_err_list)));
disp('Average Position RMSE among Robots: ')
disp(num2str(mean(p_err_list)));
disp('======================================')



legend1 = legend(g,'show');
set(h1,'DisplayName','ground truth');
set(h2,'DisplayName','estimate');
set(legend1,...
    'Position',[0.409716541578968 0.00585479508702602 0.24390243902439 0.0431547619047619],...
    'Orientation','horizontal',...
    'FontSize',10,...
    'FontName','Times New Roman');
f1=gcf;

% folder=['TestResults/',datasetname,'/']; 
% name = ['method', num2str(num_method),datasetname,'_err_all.pdf'];
% savepath1 = [folder,name];
% if exist(folder)==0 
%     mkdir(folder); 
% end
% exportgraphics(f1,savepath1)


% 
% %% figure 2. 3D performance and landmarks
% figure()
% 
% for i=1:n
%     real_path=real_path_all(i).rp;
%     estX=X(i).X;
%     sz_RealPath=length(real_path); 
%     real_p_imu=zeros(sz_RealPath,3);
%     for m=1:sz_RealPath
%         real_p_imu(m,:)=real_path(m).T(1:3,4)';
%     end
% 
%     plot3(real_p_imu(:,1), real_p_imu(:,2), real_p_imu(:,3),'LineWidth',2,'DisplayName','Ground Truth'); hold on 
% 
%     [T_IMU_est, p_IMU_est]=get_estimator_T(estX);
%     displayname = ['CI tracking: Robot',num2str(i)];
%     plot3(p_IMU_est(:,1), p_IMU_est(:,2), p_IMU_est(:,3),'DisplayName',displayname,'LineStyle','--','LineWidth',2); hold on 
% 
% end
% 
% num_lm = length(landmarks);
% p_lm_list = [];
% for j = 1:num_lm
% 
%     if(~isempty(landmarks(j).T))
%         p_lm_list(end+1,:) = landmarks(j).T(1:3,4)';
%         scatter3(p_lm_list(end,1), p_lm_list(end,2),p_lm_list(end,3),'.','green'); hold on
%     end
%     %%%%% also show pose
% 
% 
% %     t=linspace(0,pi,25);
% %     p=linspace(0,2*pi,25);
% %     [theta,phi]=meshgrid(t,p);
% %     if(mod(j,5)==0)  % Downsampling to show the effective range
% %     x=Eff_Range*sin(theta).*sin(phi)+p_lm_list(j,1);
% %     y=Eff_Range*sin(theta).*cos(phi)+p_lm_list(j,2);
% %     z=Eff_Range*cos(theta)+p_lm_list(j,3);
% %     surf(x,y,z,'linestyle','none','FaceAlpha',0.01); 
% %     axis equal    
% end
% axis equal
% view([-22 35])
% f2=gca;
% name2 = ['method', num2str(num_method),datasetname,'_3Dscene.pdf'];
% savepath2 = [folder,name2];
% exportgraphics(f2,savepath2);
% 




%% pure scene 

if(0)
    figure();
    for i=1:n
        real_path=real_path_all(i).rp;
        estX=X(i).X;
        sz_RealPath=length(real_path); 
        real_p_imu=zeros(sz_RealPath,3);
        for m=1:sz_RealPath
            real_p_imu(m,:)=real_path(m).T(1:3,4)';
        end
    
        plot3(real_p_imu(:,1), real_p_imu(:,2), real_p_imu(:,3),'LineWidth',2,'DisplayName','Ground Truth'); hold on 

    end

    num_lm = length(landmarks);
    p_lm_list = zeros(num_lm,3);
    for j = 1:num_lm
        p_lm_list(j,:) = landmarks(j).T(1:3,4)';
        scatter3(p_lm_list(j,1), p_lm_list(j,2),p_lm_list(j,3),'.','green'); hold on
%         t=linspace(0,pi,25);
%         p=linspace(0,2*pi,25);
%         [theta,phi]=meshgrid(t,p);
%         if(mod(j,3)==0)  % Downsampling to show the effective range
%         x=Eff_Range*sin(theta).*sin(phi)+p_lm_list(j,1);
%         y=Eff_Range*sin(theta).*cos(phi)+p_lm_list(j,2);
%         z=Eff_Range*cos(theta)+p_lm_list(j,3);
%         surf(x,y,z,'linestyle','none','FaceAlpha',0.02); 
%         axis equal    
%         hold on
%         end
    end
    axis equal
    grid on
    view([-22 35])
    f2=gca;
    name2 = ['pure','_3Dscene.pdf'];
    savepath2 = [folder,name2];
    exportgraphics(f2,savepath2);   


end






end

