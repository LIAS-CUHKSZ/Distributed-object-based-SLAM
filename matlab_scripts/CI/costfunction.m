function f = costfunction(w, J, p, p_prior, num)


f_content  = w(1)* inv(p_prior);
for i = 2:num
    f_content = f_content + w(i) * J' * inv(p(:,:,i)) * J;
end

f=trace(inv(f_content));





end