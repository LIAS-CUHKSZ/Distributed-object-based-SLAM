function [X_update, P_update, X_object_update]=Invariant_MSCKF_object_update(X_pred, P_pred, X_object, obj_order, mea_stack, sigma_if,,N_max,Num_see_list, index_max)
%% Input
% Num_see_list: 存放每次看到了多少个特征
% index_max: 最大的特征标签

DIM=length(P_pred);
H_all=[];
%% 在mea_stack中找到共同的特征
R_if=sigma_if*sigma_if*eye(6);

Index_search=zeros(N_max, index_max);
for i =1:N_max
    index_now=mea_stack(i).index;
    Index_search(i, index_now)=1;
end
%% 算出来每个特征的融合的结果

column_sum=sum(Index_search); %对矩阵的每一列求和，不为零的列说明有对应的数据
Regis_object_list=find(column_sum); %返回的是不为0的列。对应的列数就是有相对应观测物体的index

N_regis_ob=length(Regis_object_list); %有效点的数量
res=[];
res_all=[];
H=[];
% 求一个融合的物体pose
for j=1:N_regis_ob
    objec_index=Regis_object_list(j);   %当前物体的index

    
    who_have_tmp=Index_search(:,object_index);
    who_have=find(who_have_tmp); %判断哪些帧有该object_index的物体的测量

    N_see=length(who_have);
    T_list=[];
    P_list=[];
    for jj=1:N_see
        %先取出对应的状态
        Where_see=who_have(jj); %锁定哪一帧
        X_r=Xaug(Where_see).X;
        T_r=eye(4);
        T_r(1:3,1:3)=X_r.Rimu;
        T_r(1:3,4)=X_r.pimu';
        
        obj_order_list=mea_stack(Where_see).m.index;
        Where_obj=find(obj_order_list==objec_index);
        T_mea=mea_stack(Where_see).m(Where_obj).T;
        T_o=T_r*T_mea;

        P_r=P_pred(9+(where_see-1)*6+1: 9+where_see*6,  9+(where_see-1)*6+1: 9+where_see*6);
        P_o = P_r+R_if;


        T_list(jj).T=T_o;
        T_list(jj).P=P_o;
       
    end
    Initial_Guess=T_o;
    [Tfus(j).T, Tfus(j).P]=CorrelatedPoseFusion(Initial_Guess, T_list, itermax);

    %% 计算residual
    
    for jj=1:N_see
        roboT=T_list(jj).T;
        phi=Tfus(j).T * invT(roboT);
        res_now=se3_log(phi);

        H_f=Adx(invT(roboT));
        %%%%%% 直接把物体状态加进去一起估计了呗 
        HF=[HF;H_f];
        res=[res;res_now];
      
        %%没必要投影一起估计了吧很复杂的算法哦











        

    end



end








%% Stack形式的更新
    













end