function [result] = jacobian_left(delta)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Function: Overloading function.
%   Method1:   I(3x3) - (1-cos(theta))*skew(delta)/norm(delta)^2 + ...
%             (norm(delta)-sin(norm(delta)))*skew(delta)^2/norm(delta)^3
%   Input1:    delta: 3x1 vector
%   Returns1:  result:     See the document

%   Method2:   See the document
%   Input2:    delta: 6+3N vector
%   Returns2:  result:     See the document

%   Author:   Haoying Li.   09/10/2022
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if(size(delta,1) == 3) %R
    function_mode = 1;
end
if(size(delta,1)==6) %R,p
    function_mode = 2;
end
if(size(delta,1)==9)%R,p,v
    function_mode = 3;
end
%% 3维 R
if(function_mode == 1)
%   Function1  (SO(3))
      theta = norm(delta);
    axis = delta/theta;
    if(theta < 0.00000001)
        result = eye(3,3);
    else
        result =  sin(theta)/theta*eye(3)+(1-sin(theta)/theta)*axis*axis'+(1-cos(theta))/theta*skew(axis); %J
    end
end
%% 6维 R，p
if(function_mode==2)
%   Function2   (SE(3))
    result  = zeros(6,6);
    s_theta = delta(1:3,1);
    theta = norm(s_theta); 
    axis = s_theta/theta;

    if(theta < 0.00000001)
        result(1:3, 1:3) = eye(3,3);
        result(4:6, 4:6)=result(1:3, 1:3);
    else
        result =  sin(theta)/theta*eye(3)+(1-sin(theta)/theta)*axis*axis'+(1-cos(theta))/theta*skew(axis); %J
        result(4:6, 4:6)=result(1:3, 1:3);
    end

    rou=delta(4:6,1);
    Q=computeQ(s_theta,rou);
    result(1:3,4:6)=Q;
end
%% 9维 P,p,v
if(function_mode==3)
    result  = zeros(9,9);
    s_theta = delta(1:3,1);
    theta = norm(s_theta); 
    axis = s_theta/theta;

    if(theta< 0.00000001)
        result(1:3, 1:3) = eye(3,3);
        result(4:6, 4:6)=result(1:3, 1:3);
        result(7:9, 7:9)=result(1:3, 1:3);
    else
        result =  sin(theta)/theta*eye(3)+(1-sin(theta)/theta)*axis*axis'+(1-cos(theta))/theta*skew(axis); %J
        result(4:6, 4:6)= result(1:3, 1:3);
        result(7:9, 7:9)= result(1:3, 1:3);
    end

    p_temp=delta(4:6,1);
    v_temp=delta(7:9,1);

    Qp=computeQ(s_theta, p_temp);
    Qv=computeQ(s_theta, v_temp);
    result(1:3,4:6)=Qp;
    result(1:3,7:9)=Qv;

end

 
end