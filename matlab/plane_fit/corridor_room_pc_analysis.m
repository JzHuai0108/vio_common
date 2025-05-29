function corridor_room_pc_analysis()
las_dir = 'F:\jhuai\lidar-phone-construction\labelcloud\pointclouds';
% The resulting pictures are saved by first rotating the error point cloud
% interactively in the matlab figure, and then saving the plot using the
% save button on the top right corner of the plot.

if 0
% Sadly, we have to convert las to ply because the remote desktop matlab does not
% show the colorbar and the local desktop matlab does not have lidar
% toolbox installed in matlab and the local desktop does not have
% permission to install matlab lidar toolbox.
room_tls_las = fullfile(las_dir, '会议室整体模型/las/436.las');
corridor_tls_las = fullfile(las_dir, '整体0105/las/New Model_New Cloud.las');
las2ply(room_tls_las);
las2ply(corridor_tls_las);
end

room_tls_ply = fullfile(las_dir, '436.ply');
corridor_tls_ply = fullfile(las_dir, 'New Model_New Cloud.ply');

lio_dir = 'F:\jhuai\lidar-phone-construction\labelcloud\pointclouds';
room_lio_pcd = fullfile(lio_dir, '2024_12_05_16_33_55/original_aligned.pcd');
corridor_lio_pcd = fullfile(lio_dir, '2024_12_05_16_11_23/original_aligned.pcd');
room_tls_tform_txt = fullfile(lio_dir, '2024_12_05_16_33_55/transform_TLS.txt');
room_lio_tform_txt = fullfile(lio_dir, '2024_12_05_16_33_55/transform_LIO.txt');

corridor_tls_tform_txt = fullfile(lio_dir, '2024_12_05_16_11_23/transform_TLS.txt');
corridor_lio_tform_txt = fullfile(lio_dir, '2024_12_05_16_11_23/transform_LIO.txt');

c2ccsv = fullfile(lio_dir, 'c2c_stats.csv'); % output c2c stats
close all;

room_z_limits = [-2, 3;];
room_polygon3d = [-13.254    5.6747    2.0290;
    1.634    5.5428    1.7058;
    1.669   -2.8283    2.1348;
  -13.2771   -2.876    1.9404];

corridor_z_limits = [-6.0, 4.2];
corridor_polygon3d = [ 9.7083   0.3576    1.9722;
   -5.6206    0.4645    1.3023;
   -6.3619  -36.0255   -0.1975;
  -12.5410  -36.5770   -2.8595;
  -12.5423  -42.1529    0.3646;
    1.0353  -42.3641   -4.3962;
    1.0285   -6.1198   -4.3397;
    9.8127   -5.7070   -3.0677];

c2c_stats   = struct('filename',{},'accuracy_cm',{},'completion_cm',{},'completion_ratio_pct',{});
statsr = c2c_analysis(room_tls_ply, room_lio_pcd, room_tls_tform_txt, room_lio_tform_txt, room_z_limits, room_polygon3d);
c2c_stats(end+1) = struct( ...
    'filename', room_lio_pcd, ...
    'accuracy_cm', statsr.accuracy_cm, ...
    'completion_cm', statsr.completion_cm, ...
    'completion_ratio_pct', statsr.completion_ratio_pct ...
);

statsc = c2c_analysis(corridor_tls_ply, corridor_lio_pcd, corridor_tls_tform_txt, corridor_lio_tform_txt, corridor_z_limits, corridor_polygon3d);
c2c_stats(end+1) = struct( ...
    'filename', corridor_lio_pcd, ...
    'accuracy_cm', statsc.accuracy_cm, ...
    'completion_cm', statsc.completion_cm, ...
    'completion_ratio_pct', statsc.completion_ratio_pct ...
);

% Write C2C results
c2c_table = struct2table(c2c_stats);
writetable(c2c_table, c2ccsv);
end

function stats = c2c_analysis(tls_ply, lio_pcd, tls_tform_txt, lio_tform_txt, z_limits, polygon3d) 
ds_voxel_size         = 0.05;   % downsample gridNearest
completion_thresh     = 0.05;   % for completion ratio
c2c_distance_thresh   = 0.15;   

% 1. Load & transform TLS
% tls = readPointCloud(lasFileReader(tls_las));
tls = pcread(tls_ply);
if ~isempty(tls_tform_txt)
    pq   = readmatrix(tls_tform_txt, 'Delimiter',' ');
    tform= rigidtform3d(quat2rotm([pq(7), pq(4:6)]), pq(1:3));
    tls = pctransform(tls, tform);
end

% 2. Downsample & crop TLS
tls_ds = pcdownsample(tls,'gridAverage',ds_voxel_size);
tls_ds = mask_pc_by_range(tls_ds, z_limits);
tls_ds = crop_pc_by_polygon(tls_ds, polygon3d(:,1), polygon3d(:,2));

if 0
figure;
pcshow(tls_ds);
axis equal;
set(gca, 'Color', 'white');
set(gcf, 'Color', 'white');
axis off;
view([-93, 65]);
end

% 3. Load LIO (assumed roughly aligned)
lio = pcread(lio_pcd);
if ~isempty(lio_tform_txt)
    pq   = readmatrix(lio_tform_txt,'Delimiter',' ');
    T    = T_from_Pq(pq);
    tform= rigidtform3d(T(1:3,1:3), T(1:3,4)');
    lio = pctransform(lio, tform);
end
lio_ds = pcdownsample(lio,'gridAverage',ds_voxel_size);
lio_ds = mask_pc_by_range(lio_ds, z_limits);
lio_ds = crop_pc_by_polygon(lio_ds, polygon3d(:,1), polygon3d(:,2));

% Fine refine align with point-to-plane GICP
tfId = rigidtform3d();
tformICP = pcregistericp(lio_ds, tls_ds, 'Metric','pointToPlane',...
    'InlierDistance', 0.3, 'InitialTransform', tfId, 'Tolerance', [0.001,0.05]);
% Assert small relative motion: rotation <2°, translation <20cm
R = tformICP.R;
angleRad = acos((trace(R)-1)/2);
angleDeg = rad2deg(angleRad);
translation = tformICP.Translation;
transMag = norm(translation);

disp(tformICP.A);
lio_refined = pctransform(lio_ds, tformICP);

figure;
pcshowpair(tls_ds, lio_refined);
assert(angleDeg < 2,  'ICP rotation too large: %.2f°', angleDeg);
assert(transMag < 0.2, 'ICP translation too large: %.2f m', transMag);

% 4. Compute cloud-to-cloud stats
xyz_limits = [-inf, inf; -inf, inf; -inf, inf];
stats = evaluate_c2c(tls_ds, lio_refined, xyz_limits, completion_thresh, c2c_distance_thresh);
fprintf('C2C: accuracy=%.2fcm, completion=%.2fcm, ratio=%.1f%%\n', ...
    stats.accuracy_cm, stats.completion_cm, stats.completion_ratio_pct);

% 5. Visualize error colormap on LIO
plot_c2c(tls_ds, lio_refined, xyz_limits, c2c_distance_thresh);

end
