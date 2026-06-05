function subCloud = extract_points_in_cuboid(ptCloud, V)
% V: 8×3 corner coords of the cuboid (arbitrary orientation OK)

% alphaShape expects double
V = double(V);
shp = alphaShape(V, Inf);  % convex hull of the 8 vertices
loc = ptCloud.Location;

if ndims(loc) == 3
    % Organized cloud (M×N×3)
    [M,N,~] = size(loc);
    pts = reshape(loc, [], 3);
    in  = inShape(shp, double(pts));     % convert to double
    mask = reshape(in, [M, N]);          % logical mask M×N
    subCloud = select(ptCloud, mask);    % keep in-cuboid points
else
    % Unorganized cloud (K×3)
    in  = inShape(shp, double(loc));
    subCloud = select(ptCloud, in);
end

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

% subPts   = pts(in, :);
% subColors = [];                        % if you have colors
% if ~isempty(ptCloud.Color)
%     subColors = ptCloud.Color(in,:);
% end
% 
% subCloud = pointCloud(subPts, ...
%                      'Color', subColors);
end
