function [AA] = computeA(A)
%AA=《A》:=-tr(A)*I+A
AA=-trace(A)*eye(length(A))+A;
end