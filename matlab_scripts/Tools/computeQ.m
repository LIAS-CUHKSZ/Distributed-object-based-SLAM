function Q = computeQ(s_theta,rou)
%A part of getting left Jacobian 
theta=norm(s_theta);
if(theta < 0.0001)
    Q=0.5*skew(rou);
else
    q1=skew(rou)/2;
    q2a=(theta-sin(theta))/theta^3;
    q2b=skew(s_theta)*skew(rou)+skew(rou)*skew(s_theta)+skew(s_theta)*skew(rou)*skew(s_theta);
    q2=q2a*q2b;
    q3a=(theta^2+2*cos(theta)-2)/(2*theta^4);
    q3b=skew(s_theta)*skew(s_theta)*skew(rou)+skew(rou)*skew(s_theta)*skew(s_theta)-3*skew(s_theta)*skew(rou)*skew(s_theta);
    q3=q3a*q3b;
    q4a=(2*theta-3*sin(theta)+theta*cos(theta))/(2*theta^5);
    q4b=skew(s_theta)*skew(rou)*skew(s_theta)*skew(s_theta)+skew(s_theta)*skew(s_theta)*skew(rou)*skew(s_theta);
    q4=q4a*q4b;
    
    Q=q1+q2+q3+q4;
end

end