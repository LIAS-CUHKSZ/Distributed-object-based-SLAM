

%% load yml

jsons=loadjson('jsons/27j.json');

subname(1).name='x0x31_3';
subname(2).name='x0x31_4';
subname(3).name='x0x31_7';
subname(4).name='x0x31_8';
subname(5).name='x0x31_9';


%% 存一个新的meta.mat放到原来的数据集文件夹下面。
colorlist=[1 0 0;
    0 1 0;
    0 0 1;
    0 1 1;
    1 0 1;
    1 1 0;
    0 0 0;
    0 0.4470 0.7410;
    0.8500 0.3250 0.0980;
    0.9290 0.6940 0.1250;
    0.4940 0.1840 0.5560;
    0.4660 0.6740 0.1880;
    0.3010 0.7450 0.9330;
    0.6350 0.780 0.1840;
    0.7 0 1;
    0.4,0.4,0.9;
    0.9,0.5,1;
    0.6,0,0.6;
    1,0.3,0.7;
    0.4, 0.5,0.3;
    0,0.7,0.6;
    1,0.5,0.2];


i=0;
% 
field=['x0x30_',sprintf('%05d',1)];
% json.field

number=length( fieldnames(jsons ));

for i=1:number

    field=['x0x30_',sprintf('%05d',i)];
    
    frame=getfield(jsons,field);

    original_meta_name=fullfile("E:\datasets\YCB-V\data3\data\0027",sprintf('%06d-meta.mat',i));
    found_meta_name=fullfile("E:\datasets\YCB-V\data3\data\0027",sprintf('%06d-metaf.mat',i));
    original_meta=load(original_meta_name);

    new_meta=original_meta;
    cls_indexes=original_meta.cls_indexes;
    rotation_translation_matrix=original_meta.rotation_translation_matrix;
    poses=original_meta.poses;
    
    % T_cam_i=eye(4);
    % T_cam_i(1:3,:) = rotation_translation_matrix;
    % T_cam=invT(T_cam_i);
    
    % 
    % for u=1:5
    %     subnow=subname(u).name;
    %     T=getfield(frame, subnow);
    %     poses(:,:,u)=T(1:3,:);
    % 
    %     T_ow = T_cam * T;
    %     scatter3(T_ow(1,4),T_ow(2,4),T_ow(3,4)); hold on
    % end

    save(found_meta_name,'poses','cls_indexes','rotation_translation_matrix'  );

end
