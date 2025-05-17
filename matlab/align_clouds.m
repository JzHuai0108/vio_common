function align_clouds(refPcdFile, srcPcdFile, outDir)
%ALIGN_STATIC_CLOUDS  Align a source PCD to a reference PCD using GICP
%
%   align_static_clouds(refPcdFile, srcPcdFile)
%   align_static_clouds(refPcdFile, srcPcdFile, outDir)
%
%   refPcdFile   – path to the reference point cloud (PCD)
%   srcPcdFile   – path to the source point cloud (PCD) to be aligned
%   outDir       – (optional) directory to save results (defaults to src file folder)
%
%   This script:
%     1. Loads both PCDs.
%     2. Downsamples each to ≤50 k points (random).
%     3. Visualizes the raw downsampled clouds.
%     4. Runs GICP (point-to-plane ICP).
%     5. Applies the resulting transform to the full-resolution source.
%     6. Saves the aligned PCD and the 7-DOF transform (xyz + quaternion).
%     7. Displays the final alignment.

    if nargin < 3
        outDir = fileparts(srcPcdFile);
    end
    if ~exist(outDir, 'dir')
        mkdir(outDir);
    end

    fprintf('--- Loading point clouds ---\n');
    tTotal = tic;
    refPC = pcread(refPcdFile);
    srcPC = pcread(srcPcdFile);
    fprintf(' Reference: %d points\n Source:    %d points\n', ...
            refPC.Count, srcPC.Count);
    fprintf('Load time: %.2f s\n\n', toc(tTotal));

    %% Downsample
    maxPts = 50000;
    refDS = refPC;
    if refDS.Count > maxPts
        frac = maxPts / refDS.Count;
        refDS = pcdownsample(refDS, 'random', frac);
    end
    srcDS = srcPC;
    if srcDS.Count > maxPts
        frac = maxPts / srcDS.Count;
        srcDS = pcdownsample(srcDS, 'random', frac);
    end
    fprintf('Downsampled to:\n Reference: %d pts\n Source:    %d pts\n\n', ...
            refDS.Count, srcDS.Count);

    %% Visualize raw
    figure; 
    pcshowpair(srcDS, refDS);
    title('Raw downsampled: Source (red) vs Reference (green)');
    drawnow;

    %% ICP registration
    fprintf('--- Running GICP refinement ---\n');
    tStep = tic;
    tform = pcregistericp( ...
        srcDS, refDS, ...
        'Metric','pointToPlane', ...
        'InlierDistance',5.0, ...
        'MaxIterations',50, ...
        'Tolerance',[0.001, 0.005] ...
    );
    fprintf('GICP time: %.2f s\n\n', toc(tStep));

    %% Apply to full-resolution source
    srcAligned = pctransform(srcDS, tform);

    %% Save results
    fprintf('--- Saving outputs ---\n');
    alignedFile   = fullfile(outDir, 'src_ds_aligned.pcd');
    transformFile = fullfile(outDir, 'transform_src.txt');

    % Write PCD
    pcwrite(srcAligned, alignedFile, 'Encoding','binary');
    fprintf('Aligned cloud saved to:\n  %s\n', alignedFile);

    % Write transform XYZ + quaternion (w,x,y,z)
    fprintf('tform:\n');
    disp(tform.A);
    save_transform(tform, transformFile);
    fprintf('Transform saved to:\n  %s\n', transformFile);
    fprintf('Total runtime: %.2f s\n\n', toc(tTotal));

    %% Final visualize
    figure;
    pcshowpair(pctransform(srcDS, tform), refDS);
    title('Final alignment: Source (red) vs Reference (green)');
end


function save_transform(tform, filename)
%SAVE_TRANSFORM  Write a 7-DOF rigid-body transform to a text file
%   [x y z qx qy qz qw] on one line, with 15-digit precision.

    % Extract translation
    T = tform.A;               % 4×4 homogeneous
    t = T(1:3, 4)';

    % Extract rotation quaternion [w x y z]
    q = rotm2quat(T(1:3,1:3));

    % Reorder to [x y z qx qy qz qw]
    out = [t, q(2), q(3), q(4), q(1)];

    fid = fopen(filename, 'w');
    if fid < 0
        error('Cannot open %s for writing.', filename);
    end
    fprintf(fid, '%.9f %.9f %.9f %.15f %.15f %.15f %.15f\n', out);
    fclose(fid);
end
