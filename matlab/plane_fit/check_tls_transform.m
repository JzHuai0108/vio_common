function check_tls_transform()
% check the consistency of the tls transform saved for basement las
result_dir        = '/media/jhuai/ExtremeSSD/jhuai/livox_phone/results';
ref_pc_file       = fullfile(result_dir, 's22plus_xt32', 'fastlio2', 'ref_tls', 'basement.las');
ref_transform_txt = fullfile(result_dir, 's22plus_xt32', 'fastlio2', 'ref_tls', 'transform_TLS.txt');

fprintf('Loading reference TLS scan...\n');
refReader = lasFileReader(ref_pc_file);
refCloud  = readPointCloud(refReader);

pq_ref   = readmatrix(ref_transform_txt,'Delimiter',' ');
T_ref    = T_from_Pq(pq_ref);
tformRigid = rigidtform3d(T_ref(1:3,1:3), T_ref(1:3,4));
ref_aligned = pctransform(refCloud, tformRigid);
ds_voxel_size      = 0.05;   % downsample grid size (m)
pc_ref = pcdownsample(ref_aligned, 'gridNearest', ds_voxel_size);

pcfile = '/media/jhuai/ExtremeSSD/jhuai/livox_phone/results/s22plus_xt32/fastlio2/ref_tls/tls_transformed.ply'; % basement
pc_src = pcread(pcfile);

tfId = rigidtform3d();
tformICP = pcregistericp(pc_src, pc_ref, 'Metric','pointToPlane',...
        'InlierDistance', 0.3, 'InitialTransform', tfId);

fprintf('The estimated transform between transformed orig pc and the saved transform pc (should be very close to I_4):\n');
disp(tformICP.A);

end