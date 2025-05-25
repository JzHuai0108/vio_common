function corridor_room_align_to_tls()
las_dir = '/media/jhuai/ExtremeSSD/jhuai/livox_phone/tls/北京建筑大学TLS图';
room_tls_las = fullfile(las_dir, '会议室整体模型/las/436.las');
corridor_tls_las = fullfile(las_dir, '整体0105/las/New Model_New Cloud.las');
room_tls_tform = fullfile(las_dir, '会议室整体模型/las/tls_transform.txt');
corridor_tls_tform = fullfile(las_dir, '整体0105/las/tls_transform.txt');

lio_dir = '/media/jhuai/ExtremeSSD/jhuai/livox_phone/results/s22plus_livox/20241205';
room_lio_pcd = fullfile(lio_dir, '2024_12_05_16_33_55/original_aligned.pcd');
corridor_lio_pcd = fullfile(lio_dir, '2024_12_05_16_11_23/original_aligned.pcd');

%% --- Process Room ---
if 0
fprintf('\n=== Room Alignment ===\n');
[outdir, n, e] = fileparts(room_lio_pcd);
% Note here we align tls to lio which is xyz aligned.
align_cloud_to_tls(room_lio_pcd, room_tls_las, [0, 0, -1], [0, 0, -1], outdir, true);
swap_files(fullfile(outdir, 'transform_TLS.txt'), fullfile(outdir, 'transform_LIO.txt'));
end

fprintf('\n=== Corridor Alignment ===\n');
[outdir, n, e] = fileparts(corridor_lio_pcd);
% Note here we align tls to lio which is xyz aligned.
align_cloud_to_tls(corridor_lio_pcd, corridor_tls_las, [0, 0, -1], [0, 0, -1], outdir, true);
swap_files(fullfile(outdir, 'transform_TLS.txt'), fullfile(outdir, 'transform_LIO.txt'));
end
