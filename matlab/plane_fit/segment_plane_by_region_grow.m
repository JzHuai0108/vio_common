function [regionIndicesList, planeModels] = segment_plane_by_region_grow(pc, seedPoints)
% segment_plane_by_region_grow Segment planar regions from multiple seeds
%   [regionIndicesList, planeModels] = segment_plane_by_region_grow(pc, seedPoints)
%   pc:                MATLAB pointCloud object
%   seedPoints:        Kx3 array of [x,y,z] seed coordinates
%   regionIndicesList: Kx1 cell array of linear index vectors for each region
%   planeModels:       1xK struct array with fields:
%                         Normal: 1x3 unit normal
%                         D:      scalar offset (Normal*X + D = 0)

% Parameters (tune as needed)
neighborRadius    = 0.2;    % meters
maxRadius         = 4;      % max point dist to seed pt, meters
angleThresholdDeg = 2;      % degrees

distThreshold     = 0.02;   % max point dist to seed plane, meters
numNormals        = 30;     % neighbors for normal estimation

badNormalAngleDeg = 15;  % a point with normal differ to the seed normal by this value will abort search in a small neighborhood.
badPlaneDist = 0.2;      % a point with plane distance by this value will abort search in a small neighborhood.

% Validate inputs
if ~isa(pc, 'pointCloud')
    error('pc must be a pointCloud object.');
end
if size(seedPoints,2) ~= 3
    error('seedPoints must be a Kx3 array.');
end

% Extract point coordinates
dataPts = pc.Location;

% Build KD-tree for neighbor queries
Mdl = KDTreeSearcher(dataPts);

% Estimate normals for all points
normals = pcnormals(pc, numNormals);

K = size(seedPoints,1);
regionIndicesList = cell(K,1);
planeModels       = repmat(struct('Normal',[], 'D',[]), 1, K);
angleThreshCos    = cosd(angleThresholdDeg);
badAngleThreshCos = cosd(badNormalAngleDeg);

for k = 1:K
    % Find nearest point for this seed using KD-tree
    seedPoint = seedPoints(k,:);
    seedIdx   = knnsearch(Mdl, seedPoint);
    
    % Initialize plane model from seed
    seedNormal = normals(seedIdx, :);
    planeNormal = seedNormal / norm(seedNormal);
    planeD      = -dot(planeNormal, dataPts(seedIdx,:));
    planeModels(k) = struct('Normal', planeNormal, 'D', planeD);
    
    % Prepare region growing structures
    nPts       = size(dataPts,1);
    visited    = false(nPts,1);
    visited(seedIdx) = true;
    queue      = seedIdx;
    regionIdxs = seedIdx;  % store linear indices
    
    % BFS region growing
    while ~isempty(queue)
        curr = queue(1);
        queue(1) = [];
        % Range search neighbors
        nbrsCell = rangesearch(Mdl, dataPts(curr,:), neighborRadius);
        nbrs = nbrsCell{1};
        % 1. check if the neighborhood is good
        kOffNormal = 0;
        kLargePlaneDist = 0;
        for i = 1:numel(nbrs)
            idx = nbrs(i);
            if visited(idx)
                continue;
            end
            % max dist check
            pt = dataPts(idx,:);
            if norm(pt - seedPoint) > maxRadius
                continue;
            end
            % Normal similarity check
            nvec = normals(idx,:);
            ct = dot(nvec,planeNormal)/(norm(nvec)*norm(planeNormal));
            if abs(ct) < badAngleThreshCos
                kOffNormal = kOffNormal + 1;
            end
            % Plane distance check
            pt = dataPts(idx,:);
            if abs(dot(planeNormal, pt) + planeD) > badPlaneDist
                kLargePlaneDist = kLargePlaneDist + 1;
            end
        end
        if kOffNormal > 0 || kLargePlaneDist > 0
            continue;
        end
        % 2. select points in the neighborhood
        for i = 1:numel(nbrs)
            idx = nbrs(i);
            if visited(idx)
                continue;
            end
            % max dist check
            pt = dataPts(idx,:);
            if norm(pt - seedPoint) > maxRadius
                continue;
            end
            % Normal similarity check
            nvec = normals(idx,:);
            ct = dot(nvec,planeNormal)/(norm(nvec)*norm(planeNormal));
            if abs(ct) < angleThreshCos
                continue;
            end
            % Plane distance check
            pt = dataPts(idx,:);
            if abs(dot(planeNormal, pt) + planeD) > distThreshold
                continue;
            end
            % Accept point
            visited(idx) = true;
            queue(end+1) = idx;      %#ok<AGROW>
            regionIdxs(end+1) = idx; %#ok<AGROW>
        end
    end
    denoise = true;
    if denoise
        pts   = pc.Location(regionIdxs, :);
        tmpPC = pointCloud(pts);
        % adjust NumNeighbors & Threshold to taste
        [~, inlierLoc] = pcdenoise(tmpPC, ...
                                  'NumNeighbors',50, ...
                                  'Threshold',1.0);
        regionIndicesList{k} = regionIdxs(inlierLoc);
    else
        regionIndicesList{k} = regionIdxs;
    end
end
end
