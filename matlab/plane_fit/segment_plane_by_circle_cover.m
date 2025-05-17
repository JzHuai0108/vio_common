function [regionIndicesList, planeModels] = segment_plane_by_circle_cover(pc, seedPoints)
% SEGMENT_PLANE_BY_CIRCLE_COVER  Segment planar patches by expanding circles
%
% [regionIndicesList, planeModels] = segment_plane_by_circle_cover(pc, seedPoints)
%
% Inputs:
%   pc          - pointCloud object
%   seedPoints  - K×3 array of seed coordinates
%
% Outputs:
%   regionIndicesList  - K×1 cell array of linear indices for each patch
%   planeModels        - 1×K struct array with fields .Normal (1×3) and .D

    % PARAMETERS (tune as needed)
    neighborRadius    = 0.2;    % initial circle radius (m)
    radiusStep        = 0.1;    % grow by this each iteration
    maxRadius         = 4.0;    % maximum allowed radius (m)
    angleThresholdDeg = 20;      % for final selection
    distThreshold     = 0.2;   % for final selection (m)
    numNormals        = 30;     % for pcnormals
    denoise           = true;   % post‐filter the final region

    % Precompute
    dataPts = pc.Location;                     % N×3
    Mdl     = KDTreeSearcher(dataPts);         % for range queries
    normals = pcnormals(pc, numNormals);       % N×3
    K       = size(seedPoints,1);

    regionIndicesList = cell(K,1);
    planeModels       = repmat(struct('Normal',[], 'D',[]), 1, K);
    angleThreshCos    = cosd(angleThresholdDeg);

    for k = 1:K
        % 1) initialize plane from seed
        seedPt     = seedPoints(k,:);
        seedIdx    = knnsearch(Mdl, seedPt);
        n0         = normals(seedIdx,:);
        planeNormal= n0 / norm(n0);
        planeD     = -dot(planeNormal, dataPts(seedIdx,:));
        planeModels(k) = struct('Normal',planeNormal,'D',planeD);

        % 2) expand circle until failure
        radius     = neighborRadius;
        finalRadius= 0;
        while radius <= maxRadius
            nbrs = rangesearch(Mdl, seedPt, radius);
            nbrs = nbrs{1};
            pts  = dataPts(nbrs,:);
            nvecs= normals(nbrs,:);

            cosines = abs(nvecs * planeNormal');
            dists   = abs(pts * planeNormal' + planeD);

            % if every point in this circle is good, keep growing
            if all(cosines >= angleThreshCos) && all(dists <= distThreshold)
                finalRadius = radius;
                radius = radius + radiusStep;
            else
                break;
            end
        end

        % 3) collect final region at finalRadius
        if finalRadius > 0
            nbrs = rangesearch(Mdl, seedPt, finalRadius);
            nbrs = nbrs{1};
            pts  = dataPts(nbrs,:);
            nvecs= normals(nbrs,:);

            cosines = abs(nvecs * planeNormal');
            dists   = abs(pts * planeNormal' + planeD);
            keep    = (cosines >= angleThreshCos) & (dists <= distThreshold);

            region = nbrs(keep);
        else
            % no growth → just the seed itself
            region = seedIdx;
        end

        % 4) optional denoising
        if denoise && numel(region)>1
            tmpPC = pointCloud(dataPts(region,:));
            [~, inliers] = pcdenoise(tmpPC, 'NumNeighbors',50, 'Threshold',1.0);
            region = region(inliers);
        end

        regionIndicesList{k} = region;
    end
end
