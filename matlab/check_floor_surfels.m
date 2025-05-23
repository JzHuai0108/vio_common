function check_floor_surfels()
close all;
result_dir        = '/media/jhuai/ExtremeSSD/jhuai/livox_phone/results';
pc_files = { ...
    fullfile(result_dir,'s22plus_xt32','fastlio2','2025_04_30_11_26_00','aggregated_cloud.pcd'), ...
    fullfile(result_dir,'s22plus_xt32','fastlio2','2025_04_30_11_28_58','aggregated_cloud.pcd')
};
pc_transforms = { ...
    fullfile(result_dir,'s22plus_xt32','fastlio2','2025_04_30_11_26_00','transform_LIO_refined.txt'), ...
    fullfile(result_dir,'s22plus_xt32','fastlio2','2025_04_30_11_28_58','transform_LIO_refined.txt')
};

% Axis-aligned crop limits [xmin xmax; ymin ymax; zmin zmax]
xyz_limits      = [-24, 31; -12.5, 19; 0.0, 3.0];
surfel_file    = fullfile(result_dir,'s22plus_xt32','fastlio2','ref_tls', 'floors.txt');
ds_voxel_size = 0.08;
for k = 1:numel(pc_files)
    src_file = pc_files{k};
    fprintf('Working on %s...\n', src_file);
    srcCloud = pcread(src_file);
    pq_src    = readmatrix(pc_transforms{k},'Delimiter',' ');
    T_src     = T_from_Pq(pq_src);
    tformRigid = rigidtform3d(T_src(1:3,1:3), T_src(1:3,4));
    src_aligned = pctransform(srcCloud, tformRigid);
    pc_src = pcdownsample(src_aligned, 'gridNearest', ds_voxel_size);
    loc_src = pc_src.Location;
    mask_src = ...
        loc_src(:,1) > xyz_limits(1,1) & loc_src(:,1) < xyz_limits(1,2) & ...
        loc_src(:,2) > xyz_limits(2,1) & loc_src(:,2) < xyz_limits(2,2) & ...
        loc_src(:,3) > xyz_limits(3,1) & loc_src(:,3) < xyz_limits(3,2);
    idx = find(mask_src);
    pc_src = select(pc_src, idx);

    file_s = surfel_file;
    fprintf('loading surfels from %s...\n', file_s);
    bboxes = load_bboxes(file_s);
    
    figure;
    pcshow(pc_src);
    hold on;
    show_boxes_on_pc(bboxes, pc_src);
    hold off;
    
    xlabel('X (m)');
    ylabel('Y (m)');
    zlabel('Z (m)');
    axis equal;
    view(0, 90);
end
end