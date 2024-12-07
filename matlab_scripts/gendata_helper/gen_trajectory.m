function [vehicle, T_real0,p] = gen_trajectory(T_SYSTEM, dt,p0,k,num)
%  Generate real trajectory (100Hz,adjustable)|Output: struct set: velocity, acc, w, T(Homogeneous Transformation Matrix)
x = []; y = []; z = [];
n = 0;

%-----------第一个点是设定好的起始点---------------------------------------------------------------------%---------------

for time = 0:dt:T_SYSTEM+2*dt
    n =n+1;
    vehicle(n).t = time;
    [R, W, position]=get_tria_point(time,p0,k);
    vehicle(n).w=W;
    vehicle(n).T(1:3,1:3)=R;
    x(n)=position(1);
    y(n)=position(2);
    z(n)=position(3);
    
    vehicle(n).T(1,4)=x(n);
    vehicle(n).T(2,4)=y(n);
    vehicle(n).T(3,4)=z(n);
    vehicle(n).T(4,1:4)=[0,0,0,1];

end

%% 降采样，绘制方向
axx=[1;0;0];axy=[0;1;0];axz=[0;0;1];

for m=1:n
    if(mod(m,75)==0)
        R=vehicle(m).T(1:3,1:3);
        p=vehicle(m).T(1:3,4)';
        bx=R*axx;
        by=R*axy;
        bz=R*axz;

        axs_x=quiver3( p(1),p(2),p(3), bx(1), bx(2),bx(3),0.3 );hold on
        axs_x.Color='red';
        axs_x.MaxHeadSize=0.3;
        axs_x.LineWidth=0.1;

        axs_y=quiver3( p(1),p(2),p(3), by(1), by(2),by(3),0.3 );hold on
        axs_y.Color='green';
        axs_y.MaxHeadSize=0.3;
        axs_y.LineWidth=0.1;

        axs_z=quiver3( p(1),p(2),p(3), bz(1), bz(2),bz(3),0.3 );hold on
        axs_z.Color='blue';
        axs_z.MaxHeadSize=0.3;
        axs_z.LineWidth=0.1;
        
    end


end



scatter3(x(1),y(1),z(1),5,"red",'*');hold on
xlabel('x(m)');
ylabel('y(m)');
zlabel('z(m)');
robo=['Robot',num2str(num)]
p=plot3(x,y,z,'LineWidth',2,'DisplayName',robo);hold on;
% legend(p,robo);
text(p0(1),p0(2),p0(3),num);
T_real0=vehicle(1).T;
end