function register_basement_tls_scans()
close all;
datadir='/media/jhuai/BackupPlus/jhuai/data/homebrew/whu_tls/project2';
result_dir = '/media/jhuai/ExtremeSSD/jhuai/livox_phone/results';
out_pc_file = fullfile(result_dir, 's22plus_xt32/fastlio2/ref_tls/refined_basement.ply');
out_pose_file = fullfile(result_dir, 's22plus_xt32/fastlio2/ref_tls/refined_basement_poses.csv');

chosen_lasfiles = {'27.las', '28.las', '29.las', '30.las', '31.las', '32.las'};
lasfiles = fullfile(datadir, chosen_lasfiles);

% the reference file is generated by aligning the aggregated basement pc's
% normals to the xy axes. It was used to generate the cuboids. Since we
% want to keep using these cuboids, we have to align the refined basement
% to this point cloud.
ref_pc_file = fullfile(result_dir, 's22plus_xt32/fastlio2/ref_tls/tls_transformed.ply');
ref_transform = fullfile(result_dir, 's22plus_xt32/fastlio2/ref_tls/transform_TLS.txt');
pq_src = readmatrix(ref_transform,'Delimiter',' ');
T_src  = T_from_Pq(pq_src);
tformInitial = rigidtform3d(T_src(1:3,1:3), T_src(1:3,4));


pc_ref = pcread(ref_pc_file);
mergedPoints = [];
mergedColors = [];
poses = [];
voxel_size = 0.1;
for i = 1:numel(lasfiles)
    lfile = lasfiles{i};
    lasReader = lasFileReader(lfile);
    pc = readPointCloud(lasReader);
    pc_src = pcdownsample(pc, 'gridNearest', voxel_size);

    tformICP = pcregistericp(pc_src, pc_ref, 'Metric', 'pointToPlane',...
                             'InlierDistance', 0.5, 'Tolerance', [0.001,0.05],...
                             'InitialTransform', tformInitial);
    % Compute relative difference
    dR = tformICP.R' * tformInitial.R;
    dt = tformICP.Translation - tformInitial.Translation;
    aa = rotm2axang(dR);
    rotAngleDeg = rad2deg(aa(4));
    transNorm = norm(dt);
    fprintf('File %s: Rotation difference = %.2f deg, Translation difference = %.2f m\n', chosen_lasfiles{i}, rotAngleDeg, transNorm);

    % Transform and merge aligned point cloud
    pc_aligned = pctransform(pc_src, tformICP);
    mergedPoints = [mergedPoints; pc_aligned.Location];
    mergedColors = [mergedColors; pc_aligned.Color];

    % Store poses (filename, translation, quaternion)
    R = tformICP.R;
    q = rotm2quat(R);
    t = tformICP.Translation;
    poses = [poses; {chosen_lasfiles{i}, t(1), t(2), t(3), q(2), q(3), q(4), q(1)}];

    % Visualization
    figure;
    pcshowpair(pc_ref, pc_aligned);
    title(sprintf('Alignment of %s', chosen_lasfiles{i}), 'Interpreter', 'none');
end

% Save the merged point cloud
mergedPC = pointCloud(mergedPoints, 'Color', mergedColors);
pcwrite(mergedPC, out_pc_file);

writecell(poses, out_pose_file);
end
