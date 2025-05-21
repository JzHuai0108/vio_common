function [stats, poly_pts] = evaluate_plane(pc, verts, dist_thresh_m)
%EVALUATE_PLANE  Fit and evaluate and visualize a planar segment in a point cloud.
%   [stats, poly_pts] = evaluate_plane(pc, verts, dist_thresh_m) extracts
%   points within the cuboid defined by the 8×3 `verts` hull from pointCloud `pc`,
%   fits a plane via PCA, computes point-to-plane errors, and returns:
%     stats.normal              - unit normal of the fitted plane
%     stats.centroid            - centroid of the inlier points
%     stats.rmse_cm             - RMSE of distances (cm)
%     stats.inlier_ratio_pct    - % of points within `dist_thresh_m` (m)
%     stats.intersection        - Nx3 vertices of the polygon where the fitted
%                                 plane intersects the cuboid
%   poly_pts is identical to stats.intersection for plotting.
%
%   Inputs:
%     pc             - pointCloud object
%     verts          - 8×3 matrix of cuboid corner coordinates
%     dist_thresh_m  - distance threshold in meters (e.g. 0.01 for 1 cm)
%
%   Output:
%     stats          - struct with plane parameters and error metrics
%     poly_pts       - ordered hull vertices of the plane patch

    % 1) Extract points in the cuboid --------------------------------
    idxs = find_points_in_box(pc, verts);
    if isempty(idxs)
        error('No points found inside the specified cuboid.');
    end
    pts = pc.Location(idxs, :);

    % 2) Fit plane via PCA ---------------------------------------------
    centroid = mean(pts, 1);
    C = pts - centroid;
    [V, ~] = eig(C' * C);
    normal = V(:,1);
    normal = normal(:) / norm(normal);
    if normal(3) < 0, normal = -normal; end

    % 3) Compute signed distances to plane -----------------------------
    signed_dist = (pts - centroid) * normal;
    d_abs = abs(signed_dist);

    % 4) Compute RMSE (in cm) -------------------------------------------
    rmse_m = sqrt(mean(d_abs .^ 2));
    rmse_cm = rmse_m * 100;

    % 5) Compute inlier ratio -----------------------------------------
    inlier_ratio = sum(d_abs <= dist_thresh_m) / numel(d_abs);
    inlier_pct = inlier_ratio * 100;

    % 6) Compute intersection polygon of plane & cuboid ---------------
    % Plane: normal · (X - centroid) = 0
    % Cuboid edges defined by 8 verts
    Vc = verts;
    E = [1 2;1 3;1 5;2 4;2 6;3 4;3 7;4 8;5 6;5 7;6 8;7 8];
    pts_int = [];
    for i = 1:size(E,1)
        p1 = Vc(E(i,1),:);
        p2 = Vc(E(i,2),:);
        u = p2 - p1;
        denom = dot(normal, u);
        if abs(denom) > eps
            t = dot(normal, centroid - p1) / denom;
            if t >= 0 && t <= 1
                pts_int(end+1, :) = p1 + t * u; %#ok<AGROW>
            end
        end
    end
    % Remove duplicates and order via convex hull
    if isempty(pts_int)
        poly_pts = [];
    else
        [~, ia] = unique(round(pts_int,8), 'rows', 'stable');
        pts_int = pts_int(ia, :);
        K = convhull(pts_int(:,1), pts_int(:,2), pts_int(:,3));
        poly_pts = pts_int(K, :);
    end

    % Pack stats
    stats.normal       = normal;
    stats.centroid     = centroid;
    stats.rmse_cm      = rmse_cm;
    stats.inlier_ratio_pct = inlier_pct;
    stats.intersection = poly_pts;
end
