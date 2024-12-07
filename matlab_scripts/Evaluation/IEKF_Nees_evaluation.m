function [NEES_pose, NEES_orientation] = IEKF_Nees_evaluation( X_est, P_est, data )
% plot nees for R-EKF
N  = size(X_est,2);

T = 1:N;

NEES_pose=[];
NEES_orientation=[];
for i = T    
    dw=so3_log(X_est(i).Rimu*(data(i).T(1:3,1:3))');
    dv=inv(jaco_left(dw))*(X_est(i).pimu'-X_est(i).Rimu*data(i).T(1:3,1:3)'*data(i).T(1:3,4));
    

     cov_o=P_est(i).P(1:3,1:3);
     cov_pose=P_est(i).P(1:6,1:6);

     invcov_o=eye(3)/cov_o;
     invcov_pose=eye(6)/cov_pose;
    dP=[dw;dv];
    
    NEES_orientation = [NEES_orientation dw'*invcov_o*dw/3];
    NEES_pose=  [NEES_pose dP'*invcov_pose*dP/6];       
end

fprintf( 'mean(NEES:IEKF-Pose) = %f\n', mean(NEES_pose(5:end)) );
fprintf( 'mean(NEES:IEKF-Orientation) = %f\n', mean(NEES_orientation(5:end)) );
