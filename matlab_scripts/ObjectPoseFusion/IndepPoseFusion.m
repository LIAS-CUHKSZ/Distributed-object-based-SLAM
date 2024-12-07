function [T_fuse, P_fuse,cost] = IndepPoseFusion(Initial_Guess,T_list,itermax)

num = length(T_list);
costf_value = 10000; 
iternum = 0;

T_op = Initial_Guess;
while(costf_value>=0.00001 && iternum < itermax)
    U = zeros(6,6);
    V = zeros(6,1);
    iternum = iternum+1;
    for k =1:num
        e(k).e = se3_log(T_list(k).T * invT(T_op)); % 6*1
        % Ginv = jacobian_left(-e(k).e);
        % G(k).G=inv(Ginv);
        G(k).G  = inv_J_left(-e(k).e);
        inv_sigma = inv(T_list(k).P);
        V = V+G(k).G'*inv_sigma*e(k).e;
        U = U+G(k).G'*inv_sigma*G(k).G;
    end
    % rmse(iternum)=norm(se3_log(T_op *invT(T_true) )  );
    delta = inv(U)*V;
    T_op = se3_exp(delta)*T_op;
    costf_value = 0;
    for k=1:num
        xi_op = se3_log(T_list(k).T * invT(T_op) );
        v = (e(k).e-G(k).G * xi_op);
        costf_value = costf_value + 1/2*v' * inv(T_list(k).P) * v;
    end
    
    cost(iternum)=costf_value;
end

T_fuse = T_op;
P_fuse = inv(U);

end

