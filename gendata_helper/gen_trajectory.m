function [vehicle, T_real0] = gen_trajectory(T_SYSTEM, dt,p0,k,num)
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

scatter3(x(1),y(1),z(1),5,"red",'*');hold on
plot3(x,y,z,'black','LineWidth',1);hold on;
text(p0(1),p0(2),p0(3),num);
T_real0=vehicle(1).T;
end