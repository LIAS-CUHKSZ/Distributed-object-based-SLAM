
%ref: Handbook of Multisensor Data Fusion: Theory and Practice
function [c,C,omega]=CI(a,A,b,B,H)
% This function implements the CI algorithm and fuses two estimates
% (a,A) and (b,B) together to give a new estimate (c,C) and the value
% of omega, which minimizes the determinant of C. The observation
% matrix is H.

Ai=inv(A);
Bi=inv(B);
% Work out omega using the matlab constrained minimizer function
%fminbnd().
f=inline('1/det(Ai*omega+H''*Bi*H*(1-omega))', 'omega','Ai','Bi','H');
omega=fminbnd(f,0,1,optimset('Display','off'),Ai,Bi,H);
% The unconstrained version of this optimization is:
% omega=fminsearch(f,0.5,optimset('Display','off'),Ai,Bi,H);
% omega=min(max(omega,0),1);
% New covariance
C=inv(Ai*omega+H'*Bi*H*(1-omega));
% New mean
nu=b-H*a;
W=(1-omega)*C*H'*Bi;
c=a+W*nu;

end