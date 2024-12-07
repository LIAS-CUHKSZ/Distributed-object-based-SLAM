function output = interbw2(odo1,odo2,t1,t2,n)
% 会返回插值的第一个 odo1 和n个插出来的点， 然后最后一个区间的最后一个点手动补一下，
% 371个点  共有370个间隔 ，每个间隔插4个  一共 370*4+371 = 1851个点

output=[];
q1=rotm2quat(odo1(:,1:3));
q2=rotm2quat(odo2(:,1:3));
s1=odo1(:,4);
s2=odo2(:,4);
lambda=1/n:1/(n):1-1/(n);
out=zeros(3,4);


for i=1:length(lambda)
    out(:,1:3)=quat2rotm(slerp(q1,q2,lambda(i)));
    out(:,4)=(1-lambda(i))*s1+lambda(i)*s2;
    output=[output {out}];
end

% lambda   = (to - ti(idx(:, 1)))./(ti(idx(:, 2)) - ti(idx(:, 1)));
% q1=qi(idx(:,1),:);
% q2=qi(idx(:,2),:);
% cos_theta= diag(q1*q2');
% idx_reverse=find(cos_theta<0);
% q2(idx_reverse,:)=q2(idx_reverse,:)*(-1);
% cos_theta=abs(cos_theta);
% 
% 
% sin_theta= sqrt(1-cos_theta.^2);
% angle=acos(cos_theta);
% ratio_a=sin((1-lambda).*angle)./sin_theta;
% ratio_b=sin(lambda.*angle)./sin_theta;
% qo=q1.*ratio_a+q2.*ratio_b;
end

function out=slerp(q1,q2,t)
    if t==1
        out= q2;
    elseif t==0
        out=q1;
    else
        theta=acos(q1*q2');
        out=sin((1-t)*theta)/sin(theta)*q1+sin(t*theta)/sin(theta)*q2;

    end
end