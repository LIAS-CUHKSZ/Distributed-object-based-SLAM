function [Xobup, uplist] = Refine_obj_pose(Xobj, itermax )
%%本函数分为两步：
% 1. 以i为初始估计融合邻居的。
% 2. 把融合结果作为具有耦合性的估计用CI最后融合起来。


%% Xi, Pi代表了全状态

%% 


%第一步，赋值旧的值
Xobup=Xobj;

N=length(Xobj); %估计目标物体的数量
uplist=[];
zz=1;
for i=1:N
    M=length(Xobj(i).tofuse);
    if(M>=6 )
        Init_guess=Xobj(i).T; %用i对于该物体的估计作为初始猜测
        T_list=Xobj(i).tofuse;
        uplist(end+1)=i;
        [T_fuse, P_fuse,  ~]= IndepPoseFusion(Init_guess,T_list,itermax);%融合邻居之间的信息
        Xobup(i).Tfused=T_fuse; 
        Xobup(i).Pfused=P_fuse;
        Xobup(i).tofuse=[]; %清零待融合的邻居信息
    end
    
end  


end