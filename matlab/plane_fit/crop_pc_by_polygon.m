function cropped_pc = crop_pc_by_polygon(pc, xv, yv)
% pc   : input pointCloud object
% xv,yv: vertices of the polygon (e.g., from ginput or manual input)

% Extract XY coordinates
xy = pc.Location(:, 1:2);

% Check which points are inside the polygon
in = inpolygon(xy(:,1), xy(:,2), xv, yv);

% Filter points
cropped_locations = pc.Location(in, :);
if ~isempty(pc.Color)
    cropped_colors = pc.Color(in, :);
    cropped_pc = pointCloud(cropped_locations, 'Color', cropped_colors);
else
    cropped_pc = pointCloud(cropped_locations);
end
end
