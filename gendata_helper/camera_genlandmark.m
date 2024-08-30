
function [landmarks] = camera_genlandmark(n,center,wlh)

%按照矩形生成相机路标，矩形和世界系平行
%输入：矩形质心，长宽高


landmarks=zeros(n,3);
landmarks(1:n,1)=center(1)-wlh(1)+rand(n,1)*wlh(1);
landmarks(1:n,2)=center(1)-wlh(2)+rand(n,1)*wlh(2);
landmarks(1:n,3)=center(1)-wlh(3)+rand(n,1)*wlh(3);




xlabel('x');
ylabel('y');
zlabel('z');




axis equal;
% draw landmarks
scatter3( landmarks(:, 1), landmarks(:, 2), landmarks(:, 3), 20, 'filled','pentagram'); hold on;

hold on;

end