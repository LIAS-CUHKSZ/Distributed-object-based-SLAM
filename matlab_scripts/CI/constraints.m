function [g, h ] = constraints(w)
 num = length(w);

 for i= 1:num
     g(i) = -w(i);
 end

 g(num+1)= sum(w)-1;
 h=0;

end