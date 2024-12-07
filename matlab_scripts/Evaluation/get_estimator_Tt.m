
function [T_IMU, p_imu, R_imu]  = get_estimator_Tt(X,ttttt, datasetname, robotname)


sz=length(X);
p_imu=zeros(sz,3);

% z-y-x euler 
R_imu=zeros(sz,3);

T_IMU=[];  



if(~isempty(ttttt))
    mat=zeros( sz, 8);
    for i=1:sz
        
        tnow=ttttt(i);
    
        quat=dcm2quat(X(i).Rimu);
    
        mat(i,8)=tnow;
        mat(i,1:4)=quat;
        mat(i,5:7)=X(i).pimu;
        p_imu(i,:)=X(i).pimu;
        Timu=zeros(4,4);
        Timu(4,4)=1;
        Timu(1:3,1:3)=X(i).Rimu;
        Timu(1:3,4)=X(i).pimu';
        T_IMU(i).T=Timu;
        [R_imu(i,3),R_imu(i,2),R_imu(i,1)]=dcm2angle(X(i).Rimu);
    end
else
    mat=zeros( sz, 7);
    for i=1:sz
        
       
    
        quat=dcm2quat(X(i).T(1:3,1:3));
   
        mat(i,1:4)=quat;
        mat(i,5:7)=X(i).T(1:3,4)';
        p_imu(i,:)=X(i).T(1:3,4)';
       
        T_IMU(i).T=X(i).T;
        [R_imu(i,3),R_imu(i,2),R_imu(i,1)]=dcm2angle(X(i).T(1:3,1:3));
    end


end

% folder=['TestResults/',datasetname,'/']; 
% name = ['method', num2str(num_method),datasetname,'_err_all.pdf'];
% savepath1 = [folder,name];
% if exist(folder)==0 
%     mkdir(folder); 
% end
% exportgraphics(f1,savepath1)

%% 储存结果

folder=['realdataresults/',datasetname,'/']; 
if exist(folder)==0 
    mkdir(folder); 
end
filename=[folder, robotname,'.csv'];
csvwrite(filename,mat);

end
