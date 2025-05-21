function plot_fitted_plane(pc, verts, poly_pts)
%PLOT_FITTED_PLANE  Visualize points in a cuboid and the fitted plane patch.
%   plot_fitted_plane(pc, verts, poly_pts) creates two subplots:
%     1) The raw points inside the cuboid defined by 8×3 `verts`.
%     2) The fitted plane limited by `poly_pts` (intersection polygon).
%
%   Inputs:
%     pc       - pointCloud object
%     verts    - 8×3 matrix of cuboid corner coordinates
%     poly_pts- K×3 ordered vertices of the intersection polygon on the plane

    % Extract points inside the cuboid
    idxs = find_points_in_box(pc, verts);
    locs = pc.Location(idxs, :);
    hasColor = ~isempty(pc.Color);
    if hasColor
        cols = pc.Color(idxs, :);
    end

    % Create a figure with two side-by-side subplots
    figure;

    % 1) Points in cuboid
    subplot(1,2,1);
    if hasColor
        pcshow(locs, cols);
    else
        pcshow(locs);
    end
    title('Points Inside Cuboid');
    xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
    axis equal;
    grid on;

    % 2) Fitted plane patch
    subplot(1,2,2);
    % Compute faces via 3D convex hull
    if size(poly_pts,1) >= 3
        K = convhull(poly_pts(:,1), poly_pts(:,2), poly_pts(:,3));
        patch('Vertices', poly_pts, 'Faces', K, ...
              'FaceColor', [0.8 0.1 0.1], 'FaceAlpha', 0.5, 'EdgeColor', 'none');
    end
    hold on;
    % Draw cuboid edges based on vertex order: bottom 1-4 cw, top 5-8 cw
    E = [1 2; 2 3; 3 4; 4 1;    % bottom square
         5 6; 6 7; 7 8; 8 5;    % top square
         1 5; 2 6; 3 7; 4 8];   % vertical edges
    for iEdge = 1:size(E,1)
        p1 = verts(E(iEdge,1),:);
        p2 = verts(E(iEdge,2),:);
        plot3([p1(1), p2(1)], [p1(2), p2(2)], [p1(3), p2(3)], 'k-', 'LineWidth', 1);
    end
    title('Fitted Plane Patch');
    xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
    axis equal;
    view(3);
    grid on;
end
