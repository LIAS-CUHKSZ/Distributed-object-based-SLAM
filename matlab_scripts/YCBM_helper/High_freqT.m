function  [real_path, T_real0]=High_freqT(robot_low_freq, dti, dt_object)
%UNTITLED4 此处显示有关此函数的摘要
%   此处显示详细说明



N_interp=floor(dt_object/dti);  %每两帧之间插值多少个

steps=length(robot_low_freq);








T_all=(steps-1)*dt_object;
inter=[];

t=0:dt_object:T_all;

T_real0=robot_low_freq(1).T;

time=0;
real_path(1).T=T_real0;
real_path(1).t=time;

poslist=[];


















for i = 1:steps-1


    interlst=interbw2(robot_low_freq(i).T(1:3,:),robot_low_freq(i+1).T(1:3,:),t(i),t(i+1),N_interp);
   
    for qq=1:N_interp-1
        Ttmp=eye(4);
        Ttmp(1:3,:)=interlst{1,qq};
        real_path(end+1).T=Ttmp;
        poslist(end+1,:)=Ttmp(1:3,4)';
        time=time+dti;
        real_path(end).t=time;
    end
 
    Ttmp=robot_low_freq(i+1).T;
    real_path(end+1).T=Ttmp;
    poslist(end+1,:)=Ttmp(1:3,4)';
    time=time+dti;
    real_path(end).t=time;
end


plot3(poslist(:,1),poslist(:,2),poslist(:,3)); hold on
scatter3(poslist(:,1),poslist(:,2),poslist(:,3),10,"filled");%体现密度

end