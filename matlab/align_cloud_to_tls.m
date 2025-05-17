function align_cloud_to_tls(tlsLasFile, lioPcdFile, g_LIO, outdir, lockTls)
% Align a LIO-generated PCD to a TLS-generated LAS using random downsampling,
% gravity, median shift, normal-based yaw alignment with voxel downsampling,
% and ICP refinement, with intermediate visualizations and timing messages.

%% Parameters (override by function arguments if needed)
% lockTls: fix TLS point cloud, if false, a yaw rotation will be applied.
if nargin < 5
    lioPcdFile = "/media/jhuai/ExtremeSSD/jhuai/livox_phone/results/s22plus_xt32/fastlio2/2025_04_30_11_26_00/aggregated.pcd";
    tlsLasFile = "/media/jhuai/ExtremeSSD/jhuai/livox_phone/results/s22plus_xt32/fastlio2/ref_tls/basement.las";
    g_LIO      = [-9.767300, -0.086530, -0.899358];  % gravity in world read from the LIO result or the IMU data of the rosbag stationary section
    [outdir, n, e] = fileparts(lioPcdFile);
    lockTls = false;
    zshift = 5; % shift z values by this amount, to avoid negative values,
    % only takes effectt when lockTls is false.
end

fprintf('If the yaw alignment does not look good, you may add or subtract pi/2 or pi from the yaw diff angle.\n');
fprintf('If the ICP does not look good, you may enlarge the InlierDistance.\n');

close all;
%% 1. Load point clouds
fprintf('Loading point clouds...\n');
startTotal = tic;
dsLAS      = lasFileReader(tlsLasFile);
tlsPtCloud = readPointCloud(dsLAS);
lioOrig    = pcread(lioPcdFile);
fprintf('Loaded TLS (%.0f pts) and LIO (%.0f pts) clouds.\n', tlsPtCloud.Count, lioOrig.Count);

totalLoadTime = toc(startTotal);
fprintf('Loading completed in %.2f s.\n\n', totalLoadTime);

%% 2. Random downsampling to limit to ~20k points
fprintf('Starting random downsampling...\n');
startStep = tic;
maxPts = 50000;
% LIO
pcLIOraw = pointCloud(lioOrig.Location);
if pcLIOraw.Count > maxPts
    frac = maxPts / pcLIOraw.Count;
    pcLIOraw = pcdownsample(pcLIOraw, 'random', frac);
end
% TLS
pcTLSraw = tlsPtCloud;
if pcTLSraw.Count > maxPts
    frac2 = maxPts / pcTLSraw.Count;
    pcTLSraw = pcdownsample(pcTLSraw, 'random', frac2);
end
fprintf('Downsampled to LIO: %.0f pts, TLS: %.0f pts.\n', pcLIOraw.Count, pcTLSraw.Count);
fprintf('Random downsampling done in %.2f s.\n\n', toc(startStep));

figure; pcshowpair(pcLIOraw, pcTLSraw);
title('Randomly Downsampled Raw LIO (red) vs TLS (green)');

%% 3. Gravity-vector alignment on LIO
fprintf('Starting gravity-vector alignment...\n');
startStep = tic;
v1      = g_LIO(:)/norm(g_LIO);
vDown   = [0;0;-1];
axisRot = cross(v1, vDown);
if norm(axisRot)<eps
    R_grav = eye(3);
else
    axisRot = axisRot/norm(axisRot);
    ang     = acos(dot(v1, vDown));
    K       = [0,-axisRot(3),axisRot(2); axisRot(3),0,-axisRot(1); -axisRot(2),axisRot(1),0];
    R_grav  = eye(3)+sin(ang)*K+(1-cos(ang))*(K*K);
end
lioGrav   = (R_grav * pcLIOraw.Location')';
fprintf('Gravity alignment done in %.2f s.\n\n', toc(startStep));
figure; pcshowpair(pointCloud(lioGrav), pcTLSraw);
title('After Gravity Alignment');

%% 4. Median-center translation
fprintf('Starting median-center translation...\n');
startStep = tic;
tlsPts    = pcTLSraw.Location;
medLIO    = median(lioGrav,1);
medTLS    = median(tlsPts,1);
shift     = medTLS - medLIO;
lioTrans  = lioGrav + shift;
fprintf('Translation vector [%.2f, %.2f, %.2f].\n', shift);
fprintf('Median shift done in %.2f s.\n\n', toc(startStep));
figure; pcshowpair(pointCloud(lioTrans), pcTLSraw);
title('After Median Shift (Translation)');


%% 5. Compute normals & voxel downsampling
fprintf('Starting normal estimation and voxel downsampling...\n');
startStep = tic;
pcLIOtrans   = pointCloud(lioTrans);
pcLIOtrans.Normal = pcnormals(pcLIOtrans, 20);
pcTLSraw.Normal = pcnormals(pcTLSraw, 20);
voxelSize    = 0.1;
pcLIO_ds     = pcdownsample(pcLIOtrans, 'gridAverage', voxelSize);
pcTLS_ds     = pcdownsample(pcTLSraw,   'gridAverage', voxelSize);
fprintf('Downsampled normals: LIO %d pts, TLS %d pts.\n', pcLIO_ds.Count, pcTLS_ds.Count);
fprintf('Normal & voxel downsampling done in %.2f s.\n\n', toc(startStep));
figure; quiver3(pcLIO_ds.Location(:,1), pcLIO_ds.Location(:,2), pcLIO_ds.Location(:,3), ...
               pcLIO_ds.Normal(:,1), pcLIO_ds.Normal(:,2), pcLIO_ds.Normal(:,3));
title('LIO Downsampled Normals');
figure; quiver3(pcTLS_ds.Location(:,1), pcTLS_ds.Location(:,2), pcTLS_ds.Location(:,3), ...
               pcTLS_ds.Normal(:,1), pcTLS_ds.Normal(:,2), pcTLS_ds.Normal(:,3));
title('TLS Downsampled Normals');


%% 6. Normal-based yaw alignment
fprintf('Starting normal-based yaw alignment...\n');
startStep = tic;
deltaAngles = linspace(-pi,pi,181);
countsL = histcounts(atan2(pcLIO_ds.Normal(:,2), pcLIO_ds.Normal(:,1)), deltaAngles);
countsT = histcounts(atan2(pcTLS_ds.Normal(:,2), pcTLS_ds.Normal(:,1)), deltaAngles);

% Plot histograms
figure;
bar(deltaAngles(1:end-1), countsL, 'histc');
title('LIO Normal Angle Histogram');
xlabel('Angle (rad)'); ylabel('Count');
figure;
bar(deltaAngles(1:end-1), countsT, 'histc');
title('TLS Normal Angle Histogram');
xlabel('Angle (rad)'); ylabel('Count');

[~,iL] = max(countsL); peakL = mean(deltaAngles(iL:iL+1));
[~,iT] = max(countsT); peakT = mean(deltaAngles(iT:iT+1));
% Compute and apply individual yaw corrections for LIO and TLS clouds
% (shifting about medTLS so rotation is about the common center)


if lockTls
    yawLIO   = peakT - (peakL + pi/2);
    tlsOrient = pcTLSraw.Location;
    R_LIO    = [ cos(yawLIO), -sin(yawLIO), 0;
             sin(yawLIO),  cos(yawLIO), 0;
             0,            0,           1 ];
    lioOrient = (R_LIO * (lioTrans - medTLS)')' + medTLS;
else
    yawLIO   = -peakL + pi/2;
    yawTLS   = -peakT;
    R_TLS    = [ cos(yawTLS), -sin(yawTLS), 0;
             sin(yawTLS),  cos(yawTLS), 0;
             0,            0,           1 ];
    tlsOrient = (R_TLS * (pcTLSraw.Location - medTLS)')';
    R_LIO    = [ cos(yawLIO), -sin(yawLIO), 0;
             sin(yawLIO),  cos(yawLIO), 0;
             0,            0,           1 ];
    lioOrient = (R_LIO * (lioTrans - medTLS)')';
end

% Compute and report the yaw offset between the two clouds
yawDiff = peakT - peakL;
fprintf('PeakT %.3f°, PeakL %.3f°, Yaw difference: %.3f rad (%.2f°)\n',...
    rad2deg(peakT), rad2deg(peakL), yawDiff, rad2deg(yawDiff));
fprintf('Yaw alignment completed in %.2f s.\n\n', toc(startStep));

% Visualize the result
figure;
pcshowpair( ...
    pointCloud(lioOrient), ...
    pointCloud(tlsOrient) ...
);
title('After Normal-based Yaw Alignment');
xlabel('X'); ylabel('Y'); zlabel('Z');

%% 7. Fine refinement with ICP
fprintf('Starting ICP refinement...\n');
startStep = tic;
tmpInit = pointCloud(lioOrient);
tmpTLS  = pointCloud(tlsOrient);
tformICP = pcregistericp(tmpInit, tmpTLS, 'Metric','pointToPlane', ...
    'InlierDistance', 5.0, ...
    'MaxIterations', 50, 'Tolerance',[0.001,0.005]);
lioAligned = pctransform(tmpInit, tformICP);
fprintf('ICP done in %.2f s.\n\n', toc(startStep));

%% 8. Save transformation matrices
fprintf('Saving transforms...\n');
% Homogeneous: gravity
Tgrav = eye(4); Tgrav(1:3,1:3)=R_grav;
Tshift = eye(4); Tshift(1:3,4)=shift';
% yaw LIO about medTLS
TyL = eye(4); TyL(1:3,1:3)=R_LIO; 

% yaw TLS
TyT = eye(4);
Tz = eye(4);
if ~lockTls
    TyL(1:3,4)=-R_LIO*medTLS';
    TyT(1:3,1:3)=R_TLS; TyT(1:3,4)=-R_TLS*medTLS';
    Tz(3, 4) = zshift;
else
    TyL(1:3,4)=(eye(3)-R_LIO)*medTLS';
end
% ICP
tfICP = tformICP.A;
% total LIO
T_LIO = Tz * tfICP * TyL * Tshift * Tgrav;
T_TLS = Tz * TyT;

% Save transforms with 15-decimal precision
fid = fopen(fullfile(outdir,'transform_LIO.txt'),'w');
q = rotm2quat(T_LIO(1:3, 1:3)); % wxyz
fprintf(fid,'%.9f %.9f %.9f %.15f %.15f %.15f %.15f\n', ....
    T_LIO(1,4), T_LIO(2,4), T_LIO(3,4), q(2), q(3), q(4), q(1));
fclose(fid);

fid = fopen(fullfile(outdir,'transform_TLS.txt'),'w');
q = rotm2quat(T_TLS(1:3, 1:3));
fprintf(fid,'%.9f %.9f %.9f %.15f %.15f %.15f %.15f\n', ....
    T_TLS(1,4), T_TLS(2,4), T_TLS(3,4), q(2), q(3), q(4), q(1));
fclose(fid);
fprintf('Transforms saved.\n\n');

%% 9. Final visualization & save
fprintf('Visualizing and saving final aligned cloud...\n');
figure; pcshowpair(lioAligned, tmpTLS);
title('Final ICP Aligned LIO (red) vs TLS (green)');
fprintf('Alignment complete. Total time: %.2f s.\n', toc(startTotal));

%% 10. Reload transforms from files, apply to originals, and save
fprintf('Reloading transforms and applying to originals...\n');
% Read saved transforms
format longg
pq_LIO_loaded = readmatrix(fullfile(outdir,'transform_LIO.txt'), 'Delimiter', ' ');
pq_TLS_loaded = readmatrix(fullfile(outdir,'transform_TLS.txt'), 'Delimiter', ' ');

T_LIO_loaded = T_from_Pq(pq_LIO_loaded);
T_TLS_loaded = T_from_Pq(pq_TLS_loaded);

% Reload original clouds
tmpLIO_orig = pcread(lioPcdFile);
if tmpLIO_orig.Count > maxPts
    frac = maxPts / tmpLIO_orig.Count;
    tmpLIO_orig = pcdownsample(tmpLIO_orig, 'random', frac);
end

R = T_LIO_loaded(1:3,1:3);
t = T_LIO_loaded(1:3,4);
tformRigid = rigidtform3d(R, t);
pcLIO_tf = pctransform(pointCloud(tmpLIO_orig.Location), tformRigid);
pcwrite(pcLIO_tf, fullfile(outdir, 'lio_transformed.pcd'));

if ~lockTls
    tmpTLS_orig = readPointCloud(lasFileReader(tlsLasFile));
    R = T_TLS_loaded(1:3,1:3);
    t = T_TLS_loaded(1:3,4);
    tformRigid = rigidtform3d(R, t);
    pcTLS_tf = pctransform(tmpTLS_orig, tformRigid);
    % If Color is M×3 uint16 in [0…65535], convert to uint8 [0…255]
    clr16 = pcTLS_tf.Color;                       % M×3 uint16
    if max(max(clr16)) <= 255
        clr8  = uint8(clr16);
    else
        clr8  = convertcolor(clr16);   % scale into 0–255
    end
    pcTLS_tf.Color = clr8;
    % If there are still too many points, downsample
    voxelSize = 0.1;          % for voxel‐grid downsampling (meters)
    if pcTLS_tf.Count > maxPts
        pcTLS_tf = pcdownsample(pcTLS_tf, 'gridNearest', voxelSize);
    end
    pcwrite(pcTLS_tf, fullfile(outdir,'tls_transformed.ply'), ...
        'Encoding','binary');
end

fprintf('Transformed PCDs saved from file transforms.\nTotal time: %.2fs\n', toc(startTotal));

%% 11. final display
tmpLIO = pcread(fullfile(outdir, 'lio_transformed.pcd'));
if ~lockTls
    tmpTLS = pcread(fullfile(outdir, 'tls_transformed.ply'));
else
    tmpTLS = readPointCloud(lasFileReader(tlsLasFile));
end

figure; pcshowpair(tmpLIO, tmpTLS);
title('Reloaded ICP Aligned LIO (red) vs TLS (green)');
view([0, 0]);
end

function T = T_from_Pq(pq)
%pq: tx ty tz qx qy qz qw
T = eye(4);
T(1:3, 1:3) = quat2rotm([pq(7), pq(4:6)]);
T(1:3, 4) = pq(1:3)';
end


function c8 = convertcolor(c16)
    % fprintf('Converting uint16 color to uint8...\n');
    c8 = uint8(c16);
    for i=1:3
        x = max(c16(:, i));
        n = min(c16(:, i));
        c8(:, i) = (double(c16(:, i)) - double(n)) * 255 / double(x - n);
    end
end
