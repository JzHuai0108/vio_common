function check_plane(pc, cuboids)
%CHECK_PLANE  Validate normals & distances inside each cuboid, per‐box warnings
%
%   check_plane(pc, cuboids)
%
%   pc       - pointCloud object
%   cuboids  - 1×K cell array, each an 8×3 array of box‐corner vertices

    % thresholds & normal estimation
    badNormalAngleDeg = 3;    % degrees
    badPlaneDist      = 0.03;   % same units as your point cloud
    numNormals        = 30;    % neighbors for pcnormals

    % pre-compute normals for every point
    normals = pcnormals(pc, numNormals);

    % loop over each cuboid
    for i = 1:numel(cuboids)
        verts = cuboids{i};         % 8×3

        % build a 3D triangulation of the 8 corners
        DT = delaunayTriangulation(verts);
        % figure;
        % tetramesh(DT,'FaceAlpha',0.3);

        % find for each query point which tetrahedron contains it
        tetraID    = DT.pointLocation(double(pc.Location));
        insideMask = ~isnan(tetraID);  % true for points inside the hull

        idxs = find(insideMask);

        if isempty(idxs)
            warning('check_plane:cuboid%d_empty', i, ...
                'Cuboid %d contains no points – skipping.', i);
            continue;
        end

        % extract the points & normals in this cuboid
        pts  = pc.Location(idxs, :);    % M×3
        nmls = normals(idxs, :);        % M×3

        % fit a best‐fit plane by PCA
        ctr = mean(pts,1);
        C   = pts - ctr;
        [~,~,V] = svd(C, 'econ');
        pNorm = V(:,3)';                % row vector normal
        pNorm = pNorm / norm(pNorm);
        pD    = -dot(pNorm, ctr);

        % compute per‐point deviations
        angs   = acosd(abs(nmls * pNorm'));          % M×1 angles
        dists  = abs(pts * pNorm' + pD);              % M×1 distances

        % count “bad” ones
        kBadNormals = sum(angs  > badNormalAngleDeg);
        kBadDists   = sum(dists > badPlaneDist);
        maxAng = max(angs);
        maxDist = max(dists);

        % warn per cuboid
        if kBadNormals > 0
            warning('check_plane:badNormals, Cuboid %d: %d points out of %d deviate > %.1f° normals with max off %.1f°.\n', ...
                 i, kBadNormals, length(idxs), badNormalAngleDeg, maxAng);
        end
        if kBadDists > 0
            warning('check_plane:badDists Cuboid %d: %d points out of %d > %.3f m from plane with max dist %.3f m.\n', ...
                 i, kBadDists, length(idxs), badPlaneDist, maxDist);
        end
    end
end
