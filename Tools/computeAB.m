function [AB] = computeAB(A,B)
%《A,B》=《A》《B》+《BA》

AB=computeA(A)*computeA(B)+computeA(B*A);

end