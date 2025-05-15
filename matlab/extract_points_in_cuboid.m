function subCloud = extract_points_in_cuboid(ptCloud, V)
% V is your 8×3 array of corner coordinates,
% The bounding box vertices are ordered clockwise from bottom to top starting at the origin. 
shp = alphaShape(V, Inf);             % convex hull of the 8 vertices
pts  = ptCloud.Location;              % N×3 array
in   = inShape(shp, pts);             % N×1 logical: true if inside

% Visualize
figure;
pcshow(ptCloud.Location);
title('TLS Cloud with Cuboid Hull');
xlabel('X'); ylabel('Y'); zlabel('Z');
hold on;

% Plot hull as red wireframe (no faces) on top
h = plot(shp);
h.FaceColor   = 'none';    % turn off faces
h.EdgeColor   = 'r';       % red edges
h.LineWidth   = 2;         % thicker edge lines
h.FaceAlpha   = 0.1;       % (optional) very faint face shading

hold off;

subPts   = pts(in, :);
subColors = [];                        % if you have colors
if ~isempty(ptCloud.Color)
    subColors = ptCloud.Color(in,:);
end

subCloud = pointCloud(subPts, ...
                     'Color', subColors);
end
