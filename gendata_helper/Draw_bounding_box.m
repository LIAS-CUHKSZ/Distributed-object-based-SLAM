function [] = Draw_bounding_box(poses)
%Simulation of bounding boxes;

N=length(poses); 
%随机生成N组长宽高
sizes=1.5+ 5*rand(N,3); %模拟目标物体的长宽高从150cm ~ 650cm分布


for i=1:N
SL=sizes(i,:)';
CV=poses(i).T(1:3,4);
R=poses(i).T(1:3,1:3);


drawCuboid(SL,CV,R,'c',0.2); hold on

dot=scatter3(CV(1), CV(2), CV(3) ,1,'o','filled','MarkerFaceColor',[0.9290 0.6940 0.1250]); hold on
axx=[1;0;0];axy=[0;1;0];axz=[0;0;1];
bx=R*axx; by=R*axy; bz=R*axz;
xx=quiver3(CV(1), CV(2), CV(3), bx(1), bx(2),bx(3)  ,4 );hold on

yy=quiver3(CV(1), CV(2), CV(3),by(1), by(2),by(3)    ,4);hold on
zz=quiver3(CV(1), CV(2), CV(3), bz(1), bz(2),bz(3)    ,4); hold on
xx.Color='red';
xx.MaxHeadSize=4;
yy.Color='green';
yy.MaxHeadSize=4;
zz.Color='blue';
zz.MaxHeadSize=4;
xx.LineWidth=1;
yy.LineWidth=1;
zz.LineWidth=1;
hold on
end



end