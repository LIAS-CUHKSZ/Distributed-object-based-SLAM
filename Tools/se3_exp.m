%把se3(6*1)->SE3(4，4)
function [ result ] = se3_exp( x )
    result = zeros(4,4);
    phi = x(1:3);
    R = so3_exp( phi );
    angle = norm(phi);
    axis = phi/angle;

%     J = sin(angle)/angle*eye(3)+(1-sin(angle)/angle)*axis*axis'+(1-cos(angle))/angle*skew(axis);



    result(1:3,1:3)=R;
    
    
    %theta = acos((trace(R)-1)/2);
    J = jacobian_left(phi);
    rou = x(4:6);
    t = J*rou;
    
    result(1:3, 4) = t;
    result(4, :) = [0,0,0,1];
        
    
end
