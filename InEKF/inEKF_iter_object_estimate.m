function [X_update, P_update] = inEKF_iter_object_estimate(X_prior, P_prior, landmarks, T_mea_feature, sigma_if_i, num_see_feature, itermax)
 
X_update=X_prior;
P_update=P_prior;

X_last=X_prior;
P_last=P_prior; % _last means the last iteration, which is a temporal variant. But _prior will not change, which is the estimate from prediction.

delta_X_last=zeros(15,1);
%% iteration 1 %%%%

j=1;

if(num_see_feature>=1)
    %The first iteration
    [mu_j, costf_value, K, H, delta_X]=inEKF_T_update(X_last, P_last, landmarks, T_mea_feature, sigma_if_i, X_prior, P_prior,delta_X_last);
    costf_value_last=costf_value;
    delta_X_save=delta_X_last;
    delta_X_last=delta_X;
    j=j+1;
    
    % The remaining iterations
    threshold=6*num_see_feature+15;
    while(j<=itermax && costf_value> max(threshold, 0.01*costf_value_last))
        [mu_j, costf_value, K, H, delta_X]=inEKF_T_update(mu_j, P_last, landmarks, T_mea_feature, sigma_if_i, X_prior, P_prior,delta_X_last);

        costf_value_last=costf_value;
        delta_X_save=delta_X_last;
        delta_X_last=delta_X;
        j=j+1;
        
    end
    phi=jaco_left_sum(delta_X_save);
    P_update= (eye(15,15)-K*H)*P_prior;
    P_update=(P_update+P_update')/2;
    X_update=mu_j;
end



end

