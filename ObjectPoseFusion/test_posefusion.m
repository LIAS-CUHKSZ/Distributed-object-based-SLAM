% 我们做两个实验
% 1. 复现State estimation for robotics 7.3.4。此时各个poses之间没有耦合关系
% 2. Consistent pose fusion，CI思路。            


% True pose

R_true = angle2dcm(pi/12,-pi/12,pi/12);
p_true = [1;5;-1];
T_true = eye(4);
T_true(1:3,1:3)=R_true;
T_true(1:3,4)=p_true;

Init_guess =eye(4);


% 生成不相关扰动
err1=randn(6,1).*[3;2;2;0.5;1;0.5];

mu1 = zeros(6,1);
sigma1 =diag([10,25,25,1,1,1]);
R1 = chol(sigma1);
z1 = mu1 + R1*randn(6,1);
T1 = se3_exp(z1)*T_true;


mu2 = zeros(6,1);
sigma2 =diag([5,15,5,1/2,1/2,1]);
R2 = chol(sigma2);
z2= mu2 + R2*randn(6,1);
T2 = se3_exp(z2)*T_true;

mu3 = zeros(6,1);
sigma3 =diag([1,1,1,1,1,1/2]);
R3 = chol(sigma3);
z3 = mu3 + R3*randn(6,1);
T3 = se3_exp(z3)*T_true;


%生成相关扰动
z4 = (z1+z2)/2;
T4 = se3_exp(z4)*T_true;

z5 = (z1+z2+z3)/3;
T5 = se3_exp(z5)*T_true;

z6 = z1*0.3+ z2 *0.3 + z3*0.3;
T6 = se3_exp(z6)*T_true;


T_list1(1).T = T1;
T_list1(1).P = sigma1;
T_list1(2).T = T2;
T_list1(2).P = sigma2;
T_list1(3).T = T3;
T_list1(3).P = sigma3;

T_list2(1).T = T4;
T_list2(1).P = (sigma1 +sigma2)/4;
T_list2(2).T = T5;
T_list2(2).P = (sigma1+sigma2+sigma3)/9;
T_list2(3).T = T6;
T_list2(3).P = 0.09*(sigma1+sigma2+sigma3);

itermax=4;

[T_fuse1, P_fuse1, rmse1, cost1]= IndepPoseFusion(Init_guess,T_list1,itermax,T_true);
[T_fuse2, P_fuse2, rmse2, cost2]= CorrelatedPoseFusion(Init_guess,T_list1,itermax,T_true);
[T_fuse3, P_fuse3, rmse3, cost3]= IndepPoseFusion(Init_guess,T_list2,itermax,T_true);
[T_fuse4, P_fuse4, rmse4, cost4]= CorrelatedPoseFusion(Init_guess,T_list2,itermax,T_true);



figure();
subplot(2,4,1);
plot(1:length(cost1), cost1);
subtitle('Independent source, independent fuse Cost');
subplot(2,4,2);
plot(1:length(rmse1), rmse1);
subtitle('Independent source, independent fuse RMSE');

subplot(2,4,3);
plot(1:length(cost2), cost2);
subtitle('Independent source, dependent fuse Cost');
subplot(2,4,4);
plot(1:length(rmse2), rmse2);
subtitle('Independent source, dependent fuse RMSE');


subplot(2,4,5);
plot(1:length(cost3), cost3);
subtitle('Dependent source, independent fuse Cost');
subplot(2,4,6);
plot(1:length(rmse3), rmse3);
subtitle('Dependent source, independent fuse RMSE');


subplot(2,4,7);
plot(1:length(cost4), cost4);
subtitle('Dependent source, dependent fuse Cost');
subplot(2,4,8);
plot(1:length(rmse4), rmse4);
subtitle('Dependent source, dependent fuse RMSE');


% T_fuse2 = CorrelatedPoseFusion(Init_guess,T_list2,5);
% T_fuse3= IndepPoseFusion(Init_guess,T_list1,5);
% T_fuse4 = CorrelatedPoseFusion(Init_guess,T_list2,5);



