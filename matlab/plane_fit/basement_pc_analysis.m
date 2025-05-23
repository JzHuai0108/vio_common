function basement_pc_analysis()
% Whole analysis pipeline
% Preparation
% 1. LIO and GBA to get undistorted lidar pc and poses
% 2. aggregate the pc files with poses using the c++ code in FAST-LIO2
% 3. align the overall pc by GICP to the TLS ref which is aligned to gravity and
% normals to the xy axes, using align_clouds_to_tls.m
% This function
% 1. Load reference TLS LAS and its SE3 pose;
% 2. Load multiple LIO PCDs and their SE3 poses;
% 3. Downsample, refine with GICP, and compute cloud-to-cloud stats
% 4. Evaluate plane flatness on floor/wall surfels
% 5. Save c2c_stats.csv and plane_stats.csv
close all;
%% Configuration
result_dir        = '/media/jhuai/ExtremeSSD/jhuai/livox_phone/results';
ref_pc_file       = fullfile(result_dir, 's22plus_xt32', 'fastlio2', 'ref_tls', 'basement.las');
ref_transform_txt = fullfile(result_dir, 's22plus_xt32', 'fastlio2', 'ref_tls', 'transform_TLS.txt');

pc_files = { ...
    fullfile(result_dir,'s22plus_xt32','fastlio2','2025_04_30_11_26_00','aggregated_cloud.pcd'), ...
    fullfile(result_dir,'s22plus_xt32','fastlio2','2025_04_30_11_28_58','aggregated_cloud.pcd'), ...
    fullfile(result_dir,'k60pro_livox','2025_02_07_10_25_05','front','aggregated_cloud.pcd'), ...
    fullfile(result_dir,'k60pro_livox','2025_02_07_10_28_32','front','aggregated_cloud.pcd') ...
};
pc_transforms = { ...
    fullfile(result_dir,'s22plus_xt32','fastlio2','2025_04_30_11_26_00','transform_LIO_refined.txt'), ...
    fullfile(result_dir,'s22plus_xt32','fastlio2','2025_04_30_11_28_58','transform_LIO_refined.txt'), ...
    fullfile(result_dir,'k60pro_livox','2025_02_07_10_25_05','front','transform_LIO_refined.txt'), ...
    fullfile(result_dir,'k60pro_livox','2025_02_07_10_28_32','front','transform_LIO_refined.txt') ...
};

% Axis-aligned crop limits [xmin xmax; ymin ymax; zmin zmax]
xyz_limits      = [-24, 31; -12.5, 19; 1.5, 9.3];
surfel_files    = { ...
    fullfile(result_dir,'s22plus_xt32','fastlio2','ref_tls', 'floors.txt'), ...
    fullfile(result_dir,'s22plus_xt32','fastlio2','ref_tls', 'walls.txt') ...
};

ds_voxel_size      = 0.05;   % downsample grid size (m)
c2c_distance_thresh = 0.15;
completion_thresh = 0.05;   % m for completion ratio
p2p_dist_thresh   = 0.01;   % 1 cm point-to-plane

% Prepare result containers
c2c_stats   = struct('filename',{},'accuracy_cm',{},'completion_cm',{},'completion_ratio_pct',{});
plane_stats = struct('filename',{},'surfel_file',{},'box_id',{},'rmse_cm',{},'inlier_ratio_pct',{});

%% 1. Reference point cloud setup
fprintf('Loading reference TLS scan...\n');
refReader = lasFileReader(ref_pc_file);
refCloud  = readPointCloud(refReader);

pq_ref   = readmatrix(ref_transform_txt,'Delimiter',' ');
T_ref    = T_from_Pq(pq_ref);
tformRigid = rigidtform3d(T_ref(1:3,1:3), T_ref(1:3,4));
ref_aligned = pctransform(refCloud, tformRigid);
pc_ref = pcdownsample(ref_aligned, 'gridNearest', ds_voxel_size);
loc_ref = pc_ref.Location;
mask_ref = ...
    loc_ref(:,1) > xyz_limits(1,1) & loc_ref(:,1) < xyz_limits(1,2) & ...
    loc_ref(:,2) > xyz_limits(2,1) & loc_ref(:,2) < xyz_limits(2,2) & ...
    loc_ref(:,3) > xyz_limits(3,1) & loc_ref(:,3) < xyz_limits(3,2);
idx = find(mask_ref);
pc_ref = select(pc_ref, idx);
%% 2. Loop over source point clouds for C2C
if 1
fprintf('Starting C2C analysis...\n');
for k = 1:numel(pc_files)
    src_file = pc_files{k};
    fprintf('Processing %s\n', src_file);
    srcCloud = pcread(src_file);

    % Load and apply LIO SE3
    in_tf_file = pc_transforms{k};
    pq_src = readmatrix(in_tf_file,'Delimiter',' ');
    T_src  = T_from_Pq(pq_src);
    tformRigid = rigidtform3d(T_src(1:3,1:3), T_src(1:3,4));
    src_aligned = pctransform(srcCloud, tformRigid);
    pc_src = pcdownsample(src_aligned,'gridNearest',ds_voxel_size);
    % loc_src = pc_src.Location;
    % mask_src = ...
    %     loc_src(:,1) > xyz_limits(1,1) & loc_src(:,1) < xyz_limits(1,2) & ...
    %     loc_src(:,2) > xyz_limits(2,1) & loc_src(:,2) < xyz_limits(2,2) & ...
    %     loc_src(:,3) > xyz_limits(3,1) & loc_src(:,3) < xyz_limits(3,2);
    % idx = find(mask_src);
    % pc_src = select(pc_src, idx);

    figure;
    pcshowpair(pc_ref, pc_src);
    % Fine refine align with point-to-plane GICP
    tfId = rigidtform3d();
    tformICP = pcregistericp(pc_src, pc_ref, 'Metric','pointToPlane',...
        'InlierDistance', 0.3, 'InitialTransform', tfId, 'Tolerance', [0.001,0.05]);
    % Assert small relative motion: rotation <2°, translation <20cm
    R = tformICP.R;
    angleRad = acos((trace(R)-1)/2);
    angleDeg = rad2deg(angleRad);
    translation = tformICP.Translation;
    transMag = norm(translation);
    
    disp(tformICP.A);
    pc_src_refined = pctransform(pc_src, tformICP);
    % we can save the refined tf for future use.
    % refined_tf = rigidtform3d(tformICP.A * tformRigid.A);
    % out_tf_file = [in_tf_file(1:end-4), '_refined.txt'];
    % save_tform(refined_tf, out_tf_file);

    figure;
    pcshowpair(pc_ref, pc_src_refined);
    assert(angleDeg < 2,  'ICP rotation too large: %.2f°', angleDeg);
    assert(transMag < 0.2, 'ICP translation too large: %.2f m', transMag);

    stats = evaluate_c2c(pc_ref, pc_src_refined, xyz_limits, completion_thresh, c2c_distance_thresh);

    c2c_stats(end+1) = struct( ...
        'filename', src_file, ...
        'accuracy_cm', stats.accuracy_cm, ...
        'completion_cm', stats.completion_cm, ...
        'completion_ratio_pct', stats.completion_ratio_pct ...
    );
end
% Write C2C results
c2c_table = struct2table(c2c_stats);
writetable(c2c_table, fullfile(result_dir,'c2c_stats.csv'));
end

%% 3. Plane flatness analysis
% Check the planes first
for s = 1:numel(surfel_files)
    file_s = surfel_files{s};
    bboxes = load_bboxes(file_s);
    figure;
    pcshow(pc_ref);
    hold on;
    show_boxes_on_pc(bboxes, pc_ref);
end

fprintf('Starting plane flatness evaluation...\n');
pc_files = [pc_files, {ref_pc_file}];
pc_transforms = [pc_transforms, {ref_transform_txt}];

for k = 1:numel(pc_files)
    src_file = pc_files{k};
    fprintf('Working on %s...\n', src_file);
    if endsWith(src_file, '.las')
        reader = lasFileReader(src_file);
        srcCloud  = readPointCloud(reader);
    else
        srcCloud = pcread(src_file);
    end
    pq_src    = readmatrix(pc_transforms{k},'Delimiter',' ');
    T_src     = T_from_Pq(pq_src);
    tformRigid = rigidtform3d(T_src(1:3,1:3), T_src(1:3,4));
    src_aligned = pctransform(srcCloud, tformRigid);
    pc_src_ds = pcdownsample(src_aligned, 'gridNearest', ds_voxel_size);

    for s = 1:numel(surfel_files)
        file_s = surfel_files{s};
        fprintf('loading surfels from %s...\n', file_s);
        bboxes = load_bboxes(file_s);
        for b = 1:numel(bboxes)
            verts = bboxes{b};
            fprintf('Evaluating bbox %d...\n', b);
            [pstats, ~] = evaluate_plane(pc_src_ds, verts, p2p_dist_thresh);
            plane_stats(end+1) = struct( ...
                'filename', src_file, ...
                'surfel_file', file_s, ...
                'box_id', b, ...
                'rmse_cm', pstats.rmse_cm, ...
                'inlier_ratio_pct', pstats.inlier_ratio_pct ...
            );
        end
    end
end
% Write plane stats
plane_table = struct2table(plane_stats);
writetable(plane_table, fullfile(result_dir,'plane_stats.csv'));

fprintf('Analysis complete. Results saved under %s\n', result_dir);
end
