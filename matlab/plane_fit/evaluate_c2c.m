function stats = evaluate_c2c(pc_ref, pc_src, xyz_limits, completion_thresh, distance_thresh)
%C2C_ANALYSIS  Compute cloud‐to‐cloud accuracy, completion, and completion ratio.
%   stats = c2c_analysis(pc_ref, pc_src, xyz_limits) crops both
%   point clouds pc_ref and pc_src to the axis‐aligned box defined by
%   xyz_limits = [xmin xmax; ymin ymax; zmin zmax], then computes:
%     stats.accuracy_cm         = mean distance (cm) from src points → ref
%     stats.completion_cm       = mean distance (cm) from ref points → src
%     stats.completion_ratio_pct= percentage of ref points within 5 cm of src
%
%   Inputs:
%     pc_ref     - reference pointCloud object
%     pc_src     - source   pointCloud object
%     xyz_limits - 3×2 matrix: [xmin xmax; ymin ymax; zmin zmax]
%     completion_thresh - e.g., 0.05 m
%     distance_thresh  - distance threshold in meters for ignoring points in accuracy/completion (optional)
%   Output:
%     stats - struct with fields: accuracy_cm, completion_cm, completion_ratio_pct
    if nargin < 4
        completion_thresh = 0.05;
    end
    if nargin < 5
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
    kd_src = KDTreeSearcher(loc_src_c);

    % distances from each src‐point to its nearest ref‐point
    [~, d_src2ref] = knnsearch(kd_ref, loc_src_c);
    % distances from each ref‐point to its nearest src‐point
    [~, d_ref2src] = knnsearch(kd_src, loc_ref_c);

    % Apply distance threshold filtering
    valid_src2ref = d_src2ref <= distance_thresh;
    valid_ref2src = d_ref2src <= distance_thresh;

    accuracy_cm = mean(d_src2ref(valid_src2ref)) * 100;
    completion_cm = mean(d_ref2src(valid_ref2src)) * 100;

    completion_ratio_pct = 100 * sum(d_ref2src <= completion_thresh) / numel(d_ref2src);

    %--- 4) Compute statistics ----------------------------------------
    stats.accuracy_cm           = accuracy_cm;
    stats.completion_cm         = completion_cm;
    stats.completion_ratio_pct  = completion_ratio_pct;
end
