

function [m_data] = gen_measurement_SE3(Eff_Range, landmarks, A, sigma_if, sigma_ij, trigK, Position0, T_WORKING, dt_update)
%GEN_MEA_FEATURE_SE3 generate measurements of agents see feature
% Input:
%Eff_Range: only within this range, the agent can see the feature.
%landmarks: structure-the pose of the feature in the global frame.
%A: adjacency matrix of robots, which describe the relationship of communication between robots.
%sigma_if: the measurement noise (standard deviation) of each robots when measure the feature.
%sigma_ij: the measurement noise (standard deviation) of inter-robot
%measurements.
%trigK, : path character of the robots' path. They are used to
%discretely sampling at update time.
%T_WORKING: the duration time.
%dt_update: update interval.
% Output:
%m_data: structure, storing the measurement of each robots at the update time.

%%
R_NUM=length(A);  %number of robots
LM_NUM=length(landmarks);  %number of landmarks

time=0;
n=1;

%% main loop; time duration

while(time<=T_WORKING+dt_update)
    m_data(n).t=time;
    for i=1:R_NUM
        m_data(n).robot(i).indexlist=[];
        N_eff_feature=0; %effective measurements of the landmark
        m_data(n).robot(i).feature_flag=0;
        for nn=1:LM_NUM
            T_feature = landmarks(nn).T;
            PF=T_feature(1:3,4); 
            % feature measurements
            [~,~,~,Tr]=get_tria_point(time, Position0(i,:),trigK(i,:));
            pr=Tr(1:3,4);
            dis=norm(pr-PF);
            if dis<=Eff_Range
                N_eff_feature=N_eff_feature+1;
                m_data(n).robot(i).feature_flag=m_data(n).robot(i).feature_flag+1; % record how many effective measurements
                n_if= randn(6,1)*sigma_if(i);
                T_f_in_r=se3_exp(n_if)*invT(Tr)*T_feature;
                m_data(n).robot(i).m_feature(N_eff_feature).T=T_f_in_r;
                m_data(n).robot(i).m_feature(N_eff_feature).index=nn;
                m_data(n).robot(i).indexlist(N_eff_feature)=nn;
            end
        
        end
    end

%% agent-to-agent measurements
    for i=1:R_NUM
        %agent-to-agent measurements
        neighbors=find(A(i,:)); % find() will return the non-zero index of the vector, which is the neighbor in the Adjaceny matrix.
        num_neighbor=length(neighbors);
        nnn=1;
        m_data(n).robot(i).index=zeros(num_neighbor-1,1);
        [~,~,~,Tr]=get_tria_point(time, Position0(i,:),trigK(i,:)); % Get the robot i 's pose at time 
        for j_index=1:num_neighbor
            j=neighbors(j_index);
            if(j~=i)
              
                [~,~,~,Tj]=get_tria_point(time, Position0(neighbors(j_index),:),trigK(neighbors(j_index),:)); %Tn is the pose now of the neighbor.
                
                m_data(n).robot(i).index(nnn)=j;
                n_ij=randn(6,1)*sigma_ij(i);
                m_data(n).robot(i).m_inter(nnn).m=invT(Tr)*se3_exp(n_ij)*Tj;
                nnn=nnn+1;
            end
        end
    end
    %% time evolution
    time=time+dt_update;
    n=n+1;
end


end

