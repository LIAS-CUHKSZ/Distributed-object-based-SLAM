function [Tinv] = invT(T)
    R=T(1:3,1:3);
    p=T(1:3,4);

    Tinv=eye(4);
    Tinv(1:3,1:3)=R';
    Tinv(1:3,4)= -R'*p;
end