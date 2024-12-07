function [IMU_data] = IMU_gendata(real_path, b_g,b_a,sigma_g, sigma_a,dt)
%IMUacc_list,IMUw_list,v_init_real, IMU_time
% Generate IMU measurement, add noise to "ground truth".|Output: acc,
% w(angle velocity)
% sigma_g, sigma_a
IMU_data=[];
sz=length(real_path);

former_step_velocity=zeros(3,1);
rotation_diff_former_step=eye(3,3);
g=[0;0;-9.81];

for i = 1:sz-2
    % compute odometry in so(3)
    IMU_data(i).t=real_path(i).t;
    rotationi = real_path(i).T(1:3, 1:3) ;  %poses.orientation((i-1)*3+1:i*3, :);
    rotationi1 = real_path(i+1).T(1:3, 1:3); %poses.orientation(i*3+1:(i+1)*3, :);
    rotation_diff = rotationi'*rotationi1;
    rotation_diff_so3 = so3_log(rotation_diff)';% output 1*3 vector 
    angle_vel=rotation_diff_so3/dt;% imu angle_velocity
     
    v_next = (real_path(i+2).T(1:3,4)-real_path(i+1).T(1:3,4))/dt; %poses.position(:, i+1)-poses.position(:, i);
    v_now=(real_path(i+1).T(1:3,4)-real_path(i).T(1:3,4))/dt;

    a_now_inG=(v_next-v_now)/dt;
    a_now_inI=rotationi'*(a_now_inG-g)+b_a';
    imu_angle_noise=sigma_g*randn(1,3);
    imu_acc_noise=sigma_a*randn(1,3);
    angle_vel=angle_vel+b_g+imu_angle_noise;
    accel=a_now_inI+imu_acc_noise';

    IMU_data(i).a=accel';
    IMU_data(i).w=angle_vel;
    IMU_data(i).v_inglobal_real=v_now';
end

end