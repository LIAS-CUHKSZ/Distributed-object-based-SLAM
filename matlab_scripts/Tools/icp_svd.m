function [R_new, t] = icp_svd(X, Y)
%  ICP_SVD 
% ICP_SVD computes the transformation between two point clouds using the
% iterative closest point (ICP) algorithm with singular value decomposition
% (SVD).
%
% Inputs:
%   X: Nx3 matrix representing the first point cloud
%   Y: Mx3 matrix representing the second point cloud
%   threshold: threshold for the distance between the correspondences,
%              iterations will stop once the change in distance falls below
%              this value
%
% Outputs:
%   R: 3x3 rotation matrix
%   t: 1x3 translation vector(row vector)
% Initialize transformation parameters
R = eye(3);
t = zeros(3,1);
    
%Compute transformation
X_mean = mean(X,1);
Y_mean = mean(Y,1);
X_centered = X - X_mean;
Y_centered = Y - Y_mean;
H = X_centered'*Y_centered;
[U, ~, V] = svd(H);
R_new= U*V';
t_new= X_mean' - R_new*Y_mean';
t=t_new';
    
end