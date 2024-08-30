function [] = data_split(data_path, save_path, agent_num, overlap, sigma_ij)
% Step 1: Get the number of *-color.jpg images in data_path
image_files = dir(fullfile(data_path, '*-color.jpg'));
num_images = length(image_files);


images_per_sequence = ceil(num_images / agent_num);
overlap_count = round(images_per_sequence * overlap);
sequences = cell(agent_num, 1);

for i = 1:agent_num
    start_idx = (i-1) * (images_per_sequence - overlap_count) + 1;
    end_idx = min(start_idx + images_per_sequence - 1, num_images);
    sequences{i} = image_files(start_idx:end_idx);
end


original_poses = cell(agent_num, 1);
for i = 1:agent_num
    sequence = sequences{i};
    poses = {};
    obj_poses={};
    object_index={};
    for j = 1:length(sequence)
        [~, name, ~] = fileparts(sequence(j).name);
        name = strrep(name, '-color', ''); % remove
        mat_file = fullfile(data_path, [name, '-meta.mat']);
        data = load(mat_file, 'rotation_translation_matrix');
        R_t = data.rotation_translation_matrix;
        
        T_tt=eye(4);
        T_tt(1:3,:)=R_t; 
        TTTT=invT(T_tt);
        poses{end+1} =TTTT(1:3,:);

        
        object_i=load(mat_file, 'cls_indexes');
        object_index{end+1}=object_i;

        obj_p=load(mat_file, 'poses');

        obj_poses{end+1}=obj_p;

    end
    obj_poses_all{i}=obj_poses;
    obj_index_all{i}=object_index;
    original_poses{i} = poses;
end

relative_poses = cell(agent_num, agent_num);
for i = 1:agent_num
    for j = 1:agent_num
        if i ~= j
            rel_poses = {};
            for k = 1:min(length(original_poses{i}), length(original_poses{j}))
                R_t1 = original_poses{i}{k};
                R_t2 = original_poses{j}{k};
                rel_pose = compute_relative_pose(R_t1, R_t2, 0.0001, sigma_ij(i));
                rel_poses{end+1} = rel_pose;
            end
            relative_poses{i, j} = rel_poses;
        end
    end
end

%% save data
for i = 1:agent_num
    opose=original_poses{i};
    
    indexes=obj_index_all{i};
    objposess=obj_poses_all{i};
    
 


    save(fullfile(save_path, sprintf('agent_%d_original.mat', i)), 'opose');
    save(fullfile(save_path, sprintf('agent_%d_object.mat', i)), 'objposess');
    save(fullfile(save_path, sprintf('agent_%d_index.mat', i)), 'indexes');

    for j = 1:agent_num
        if i ~= j
            rpose = relative_poses{i, j};
            save(fullfile(save_path, sprintf('agent_%d_in_%d_relative.mat', j, i)), 'rpose');
        end
    end
end

%% rename and copy images
for i = 1:agent_num
    agent_dir = fullfile(save_path, num2str(i));
    if ~exist(agent_dir, 'dir')
        mkdir(agent_dir);
    end
    
    sequence = sequences{i};
    for j = 1:length(sequence)
        src_file = fullfile(data_path, sequence(j).name);
        dest_file = fullfile(agent_dir, sprintf('%d.jpg', j-1));
        copyfile(src_file, dest_file);
    end
end
end

function relative_pose = compute_relative_pose(R_t1, R_t2, r_sig, sigma_ij)
    R1 = R_t1(:, 1:3);
    t1 = R_t1(:, 4);
    R2 = R_t2(:, 1:3);
    t2 = R_t2(:, 4);

    T1=eye(4);
    T2=eye(4);
    T1(1:3,1:3)=R1;
    T1(1:3,4)=t1;

    T2(1:3,1:3)=R2;
    T2(1:3,4)=t2;

    n_ij= randn(6,1)*sigma_ij;
    T_f_in_r=invT(T1)*se3_exp(n_ij)*T2;

    % r_noise = normrnd(0, r_sig, 3, 1);
    % r_noise_matrix = expm(skew_symmetric(r_noise));
    % R1_noise = R1 * r_noise_matrix; % right or left?
    % 
    % t_noise = normrnd(0, t_sig, 3, 1);
    % t1_noise = t1 + t_noise;
    % 
    % R_rel = R2 * R1_noise';
    % t_rel = t2 - R_rel * t1_noise;
    R_rel=T_f_in_r(1:3,1:3);
    t_rel=T_f_in_r(1:3,4);
    relative_pose = [R_rel, t_rel];
    end
    
% % function skew_matrix = skew_symmetric(vector)
% % skew_matrix = [0, -vector(3), vector(2);
% %                 vector(3), 0, -vector(1);
% %                 -vector(2), vector(1), 0];
% % end
    