%Input: time - duration time; x0,y0,z0 - initial position; kx,ky,kz -
%frequency of each axis (suggestion: 1-15).

function [R,W,Position, T] = get_tria_point(time,p0,k)
x0=p0(1); y0=p0(2); z0=p0(3);
kx=k(1); ky=k(2); kz=k(3);
W=[0.02,0.03,0.01];
start_euler=[pi/2, 0, pi/3];
R0=angle2dcm(start_euler(1), start_euler(2), start_euler(3));
R=R0*so3_exp(W*time);
% 
Position=[ 20*sin(0.10*kx*(time))+x0,    10*cos(0.10*ky*(time))+y0, 5*sin(0.10*kz*(time))+z0];
%    

% Position=[x0+0.4*time, y0+0.6*time,z0+0.5*time];
T=eye(4);
T(1:3,4)=Position';
T(1:3, 1:3)=R;
end