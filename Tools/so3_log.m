function f = so3_log(R, varargin)

if (nargin>1)
    if (norm(R-eye(3),'fro') < 2*eps)
        f = zeros(3,1);
        return
    end
end

phi = acos(1/2*(trace(R)-1));

if (nargin>1)
    if (abs(phi) < 1e-10)
        f = zeros(3,1);
        return
    end
end

if norm(R-eye(3))>0.00001
    f = so3_hatinv(phi/(2*sin(phi))*(R-R'));
else
    f=[0;0;0];
end

% function result = so3_log(R)
% 
%     theta = acos((trace(R) - 1) / 2);  % 计算旋转角度的弧度
%     
%     if theta < eps
%         % 如果旋转角度接近零，返回零矩阵
%         so3_algebra = zeros(3);
%     else
%         % 计算旋转轴
%         axis = (R - R') / (2 * sin(theta));
%         
%         % 构造反对称矩阵
%         so3_algebra = skew(axis) * theta;
%     end
%     result = so3_hatinv(so3_algebra);
% end