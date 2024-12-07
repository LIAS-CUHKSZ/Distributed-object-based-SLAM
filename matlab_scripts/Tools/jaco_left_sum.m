function result = jaco_left_sum( delta ) %%actually , this is computing J_L.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Method:   See the document
%   Input:    delta: 6+3N vector \theta p v  bg ba \theta_IC p_IC \theta_IL p_IL t_IC t_IL 
%   Returns:  jaco_left_sum
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if(size(delta,1) == 15)
    function_mode = 1;
end

if (size(delta,1) ==9)
     function_mode = 2;
end




if(function_mode==1)
    I = delta(1:9,1);

    result_I=jacobian_left(I);
    result_Ib=eye(6);
%     result_IC=jacobian_left(IC);
%     result_IC_Inv=inv(result_IC);
%     result_IL=jacobian_left(IL);
    result = blkdiag(result_I,result_Ib);
end

if(function_mode ==2)
    I = delta(1:9,1);
    result_I=jacobian_left(I);
    result = blkdiag(result_I);
end


end