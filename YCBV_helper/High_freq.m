function  [real_path, T_real0]=High_freq(robot_low_freq, dti, dt_object)
%UNTITLED4 此处显示有关此函数的摘要
%   此处显示详细说明


N_interp=floor(dt_object/dti);  %每两帧之间插值多少个

steps=length(robot_low_freq);


T_all=(steps-1)*dt_object;
inter=[];

t=0:dt_object:T_all;

T_real0=eye(4);
T_real0(1:3,:)=robot_low_freq{1,1};

time=0;
real_path(1).T=T_real0;
real_path(1).t=time;

poslist=[];
for i = 1:steps-1


    interlst=interbw2(robot_low_freq{1,i},robot_low_freq{1,i+1},t(i),t(i+1),N_interp);

    for qq=1:N_interp-1
        Ttmp=eye(4);
        Ttmp(1:3,:)=interlst{1,qq};
        real_path(end+1).T=Ttmp;
        poslist(end+1,:)=Ttmp(1:3,4)';
        time=time+dti;
        real_path(end).t=time;
    end
    Ttmp=eye(4);
    Ttmp(1:3,:)=robot_low_freq{1,i+1};
    real_path(end+1).T=Ttmp;
    poslist(end+1,:)=Ttmp(1:3,4)';
    time=time+dti;
    real_path(end).t=time;
end


scatter3(poslist(:,1),poslist(:,2),poslist(:,3),10,'o'); hold on


end