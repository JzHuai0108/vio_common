function idxs = find_points_in_box(pc, verts)
% verts 8×3

% build a 3D triangulation of the 8 corners
DT = delaunayTriangulation(verts);
% figure;
% tetramesh(DT,'FaceAlpha',0.3);

% find for each query point which tetrahedron contains it
pts = double(pc.Location);
tetraID    = DT.pointLocation(pts);
insideMask = ~isnan(tetraID);  % true for points inside the hull

idxs = find(insideMask);
end
