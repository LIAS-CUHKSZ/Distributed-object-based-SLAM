function [c,C]=conventionalKF(a,A,b,B,H)
%
% function [c,C,omega]=CI(a,A,b,B,H)
%
% This function implements the CI algorithm and fuses two estimates
% (a,A) and (b,B) together to give a new estimate (c,C) and the value
% of omega, which minimizes the determinant of C. The observation
% matrix is H.
Ai=inv(A);
Bi=inv(B);

% New covariance
C=inv(Ai+H'*Bi*H);
% New mean
nu=b-H*a;
W=C*H'*Bi;

c=a+W*nu;
end