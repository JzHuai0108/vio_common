function plot_wall_and_floor_patches()
% plot the wall and floor patch bounding boxes for publication
% Note we disabled the numeric labels in show_boxes_on_pc for clarity.
% After the plot, we save the figures by using File > Save as png from the
% matlab Figure, 
% and then crop the png images by using the Windows Photos app.

close all;

result_dir        = '/media/jhuai/ExtremeSSD/jhuai/livox_phone/results';
surfel_files    = { ...
    fullfile(result_dir,'s22plus_xt32','fastlio2','ref_tls', 'floors.txt'), ...
    fullfile(result_dir,'s22plus_xt32','fastlio2','ref_tls', 'walls.txt') ...
};

ref_pc_file       = fullfile(result_dir, 's22plus_xt32/fastlio2/ref_tls/tls_transformed.ply');

xyz_limits      = [-24, 31; -12.5, 19; 1.5, 9.3];
xyz_limits2      = [-24, 31; -12.5, 19; 1.5, 4.5];

pc_raw = pcread(ref_pc_file);
loc_ref = pc_raw.Location;
mask_ref = ...
    loc_ref(:,1) > xyz_limits(1,1) & loc_ref(:,1) < xyz_limits(1,2) & ...
    loc_ref(:,2) > xyz_limits(2,1) & loc_ref(:,2) < xyz_limits(2,2) & ...
    loc_ref(:,3) > xyz_limits(3,1) & loc_ref(:,3) < xyz_limits(3,2);
idx = find(mask_ref);
pc_ref = select(pc_raw, idx);

mask_ref2 = ...
    loc_ref(:,1) > xyz_limits2(1,1) & loc_ref(:,1) < xyz_limits2(1,2) & ...
    loc_ref(:,2) > xyz_limits2(2,1) & loc_ref(:,2) < xyz_limits2(2,2) & ...
    loc_ref(:,3) > xyz_limits2(3,1) & loc_ref(:,3) < xyz_limits2(3,2);
idx2 = find(mask_ref2);
pc_notop = select(pc_raw, idx2);

for s = 1:numel(surfel_files)
    file_s = surfel_files{s};
    bboxes = load_bboxes(file_s);
    figure;
    pcshow(pc_notop);
    hold on;
    show_boxes_on_pc(bboxes, pc_ref);
    hold off;
    xlabel('X (m)');
    ylabel('Y (m)');
    zlabel('Z (m)');
    axis equal;
    view(0, 90);
end

end
