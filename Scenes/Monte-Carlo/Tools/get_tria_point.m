%Input: time - duration time; x0,y0,z0 - initial position; kx,ky,kz -
%frequency of each axis (suggestion: 1-15).

function [R,W,Position, T] = get_tria_point(time,p0,k)
x0=p0(1); y0=p0(2); z0=p0(3);
kx=k(1); ky=k(2); kz=k(3);
W=[0.2,0.3,0.1];
start_euler=[pi/2, 0, pi/3];
R0=angle2dcm(start_euler(1), start_euler(2), start_euler(3));
R=R0*so3_exp(W*time);

Position=[ 35*sin(kx*(time))+x0,    40*cos(ky*(time))+y0,     7*sin(kz*(time))+z0];
   
T=eye(4);
T(1:3,4)=Position';
T(1:3, 1:3)=R;
end