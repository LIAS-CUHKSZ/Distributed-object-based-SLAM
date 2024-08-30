function [invJl] = inv_J_left(xi)
% 6*1 -> 6*6

ad=adjointx(xi);
invJl = eye(6)-1/2*ad+1/6*ad.*ad;

end