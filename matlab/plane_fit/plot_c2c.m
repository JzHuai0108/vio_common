function plot_c2c(pc_ref, pc_src, xyz_limits, distance_thresh, maxz_in_plot)
%plot_c2c visualize src to ref distances.
%   stats = c2c_analysis(pc_ref, pc_src, xyz_limits) crops both
%   point clouds pc_ref and pc_src to the axis‐aligned box defined by
%   xyz_limits = [xmin xmax; ymin ymax; zmin zmax], then computes:
%   distance (cm) from src points → ref
%   and visualize them on src points as a colormap
%
%   Inputs:
%     pc_ref     - reference pointCloud object
%     pc_src     - source   pointCloud object
%     xyz_limits - 3×2 matrix: [xmin xmax; ymin ymax; zmin zmax]
%     completion_thresh - e.g., 0.05 m
%     distance_thresh  - distance threshold in meters for ignoring points in accuracy/completion (optional)
    if nargin < 5
        maxz_in_plot = Inf;
    end

    if nargin < 4
        distance_thresh = Inf;
    end

    %--- 1) Crop both clouds to the ROI -------------------------------
    loc_ref = pc_ref.Location;
    loc_src = pc_src.Location;

    mask_ref = ...
        loc_ref(:,1) > xyz_limits(1,1) & loc_ref(:,1) < xyz_limits(1,2) & ...
        loc_ref(:,2) > xyz_limits(2,1) & loc_ref(:,2) < xyz_limits(2,2) & ...
        loc_ref(:,3) > xyz_limits(3,1) & loc_ref(:,3) < xyz_limits(3,2);

    mask_src = ...
        loc_src(:,1) > xyz_limits(1,1) & loc_src(:,1) < xyz_limits(1,2) & ...
        loc_src(:,2) > xyz_limits(2,1) & loc_src(:,2) < xyz_limits(2,2) & ...
        loc_src(:,3) > xyz_limits(3,1) & loc_src(:,3) < xyz_limits(3,2);

    loc_ref_c  = loc_ref(mask_ref, :);
    loc_src_c  = loc_src(mask_src, :);

    %--- 2) Build KD‐trees and find nearest neighbors -----------------
    kd_ref = KDTreeSearcher(loc_ref_c);

    % Visualize error colormap on src
    [~, d_src2ref] = knnsearch(kd_ref, loc_src_c);
    err_pc = pointCloud(loc_src_c, 'Intensity', d_src2ref);
    
    mask = d_src2ref <= distance_thresh;
    mask2 = loc_src_c(:, 3) < maxz_in_plot;
    idx  = find(mask & mask2);
    pc_inThresh = select(err_pc, idx);
    intensity_cm = pc_inThresh.Intensity * 100;

    figure;
    pcshow(pc_inThresh.Location, intensity_cm, 'MarkerSize', 50);
    colormap parula;
    cb = colorbar;
    cb.Color = [0 0 0];  % black tick labels for visibility
    cb.Label.String = 'Distance to TLS (cm)';
    % cb.Position = cb.Position .* [1 1 0.8 0.8];  % Shrink width and
    % height, but does not work well.

    xlabel('X (m)', 'Color', 'k');
    ylabel('Y (m)', 'Color', 'k');
    zlabel('Z (m)', 'Color', 'k');
    
    % Set white background
    ax = gca;
    ax.Color = 'w';       % axes background
    ax.XColor = 'k';      % axis lines in black
    ax.YColor = 'k';
    ax.ZColor = 'k';
    ax.GridColor = 'k';   % optional grid color
    
    set(gcf, 'Color', 'w'); % figure background

end
