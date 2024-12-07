 
function [X_update, P_update, X_object_update] = inEKF_SCI_iter_update(X_prior, P_prior,X_object, obj_order, T_mea_feature, sigma_if_i, num_see_feature)
                                                                                    % X_pred, P_pred, X_object, obj_order, landmarks, T_mea_feature, sigma_if(i), num_see_feature,itermax
X_update=X_prior;
P_update=P_prior;
X_object_update=X_object;

X_last=X_prior;
P_last=P_prior.Pindep; % _last means the last iteration, which is a temporal variant. But _prior will not change, which is the estimate from prediction.

%% iteration 1 %%%%

j=1;


if(num_see_feature>=1)
    %The first iteration
               
    [X_update, K, H, X_object_update]=inEKF_T_update(X_last, P_last, X_object, obj_order, T_mea_feature, sigma_if_i);
    


    P_aug_update=P_last-K*H*P_last;

    P_aug_update=(P_aug_update+P_aug_update')/2;



    P_update.Pindep=P_aug_update;
   P_update.P= P_update.Pindep+ P_update.Pdep;
end



end
