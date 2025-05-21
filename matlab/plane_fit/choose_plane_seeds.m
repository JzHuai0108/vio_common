function choose_plane_seeds(pcfile, maxz)
% This function is used to pick plane seeds or overall bounding box corners
% on the gravity and normal aligned point cloud without ceiling.
close all;
if nargin < 2
    maxz = 4.5;
end
if nargin < 1
    pcfile = '/media/jhuai/ExtremeSSD/jhuai/livox_phone/results/s22plus_xt32/fastlio2/ref_tls/tls_transformed.ply'; % basement
    % pcfile = '/media/jhuai/ExtremeSSD/jhuai/livox_phone/results/s22plus_livox/20241205/wall-distance/2024_12_05_15_55_11/mid360_10_30_rgb.pcd'; % corridor pc of 2024_12_05_15_55_11
end

scenario = 'awall'; % floors or walls or awall. awall is in corridor.
% note for awall in corridor, we also edit the line in plane_bbox to
% ymax = max(y) - xy_trim - 0.8;

[folder, baseName, ext] = fileparts(pcfile);
ext = lower(ext);  % make extension case‐insensitive

% Load only supported formats
if ismember(ext, {'.ply', '.pcd'})
    % myCloud = pcread(pcfile);
    % mask = myCloud.Location(:,2) >= -15; % this is for basement pc only.
    % pc = select(myCloud, mask);
    pc = pcread(pcfile);
    maxPts = 1e5;
    if pc.Count > maxPts
        gridStep = 0.08; % for corridor awall pc only.
        pc = pcdownsample(pc, 'gridAverage', gridStep);
    end
else
    error('Unsupported point cloud format: %s', ext);
end

figure;
mask = pc.Location(:,3) < maxz;
cullPc = pointCloud( ...
    pc.Location(mask, :), ...
    'Color', pc.Color(mask, :) ...
);

% pcshow(pc);
pcshow(cullPc);

% The below limits the basement point cloud for cloud to cloud (c2c) analysis.
xyz_limits = [-24, 31; -12.5, 19; 2.0, 9.0];
loc = pc.Location;
mask = ...
    loc(:,1) > xyz_limits(1,1) & loc(:,1) < xyz_limits(1,2) & ...  % X within limits
    loc(:,2) > xyz_limits(2,1) & loc(:,2) < xyz_limits(2,2) & ...  % Y within limits
    loc(:,3) > xyz_limits(3,1) & loc(:,3) < xyz_limits(3,2);       % Z within limits

% Cull the point cloud
cullPc2 = pointCloud( ...
    loc(mask, :), ...
    'Color', pc.Color(mask, :) ...
);

% pcshow(cullPc2);

% view(0, 0);

xlabel('X (m)');
ylabel('Y (m)');
zlabel('Z (m)');
title('Point Cloud: Z < 6 m');
axis equal;

writeout = true;

if strcmp(scenario, 'walls')
seedPoints = [
    -10.39, 5.68, 3.61;
    -10.7, -12.34, 3.42;
    -23.31, 11.90, 3.43;
    0.81, 18.54, 3.43;
    9.96, 18.7, 4.24;
    16.1, 1.38, 4.0;
    30.6, -1.0, 4.24;
    -11.69, 18.35, 3.70;
    -7.80, 1.0, 3.3;
    29.7, -5.49, 3.82;];
[regionIdxList, planeModels] = segment_plane_by_region_grow(pc, seedPoints);
elseif strcmp(scenario, 'floors')
seedPoints = [
    -6.1, 10.38, 2.14;
    -20.2, 10.1, 2.14;
    2.0, 10.6, 2.15;
    16.2, 11.2, 2.14;
    2.0, 5.6, 2.15;
    -12.6, -1.12, 2.16;
    -5.1, -3.2, 2.16;
    -12.1, -9.14, 2.15;
    6.5, -3.0, 2.13;
    28.0, -0.32, 3.0;
    ];
[regionIdxList, planeModels] = segment_plane_by_circle_cover(pc, seedPoints);
elseif strcmp(scenario, 'awall')
    seedPoints = [-0.162, -4.633, 1.905];
    [regionIdxList, planeModels] = segment_plane_by_region_grow(pc, seedPoints);
else
    fprintf('Unknown scenario %s\n', scenario);
    return;
end

numRegions = numel(regionIdxList);
colors = uint8([ zeros(numRegions,1), randi([128,255],numRegions,1), zeros(numRegions,1) ]);

% 1) Pick a random HSV triplet with high saturation & brightness
h = rand;                      % random hue
s = 0.7 + 0.3*rand;            % saturation between 0.7–1.0
v = 0.7 + 0.3*rand;            % value (brightness) between 0.7–1.0
rgbColor = hsv2rgb([h, s, v]); % 1×3 in [0,1]
colors = uint8( repmat(rgbColor*255, numRegions, 1) );

% Preallocate containers for all region points and colors
allSegmentPts   = [];
allSegmentCols  = [];

% Collect points and assign colors
xy_trim = 0.3;
z_thickness = 0.6;
chosenIds = [];
bboxes = cell(1, numRegions);
for k = 1:numRegions
    idxs = regionIdxList{k};            % linear indices for this region
    pts  = pc.Location(idxs, :);        % Nx3 coordinates
    col  = repmat(colors(k,:), numel(idxs), 1);  % Nx3 RGB
    
    allSegmentPts  = [allSegmentPts;  pts];           %#ok<AGROW>
    allSegmentCols = [allSegmentCols; col];           %#ok<AGROW>
    chosenIds = [chosenIds, idxs];
    bboxes{k} = plane_bbox(pts, planeModels(k), xy_trim, z_thickness);
end

mask = true(pc.Count, 1);   % start with all points included
mask(chosenIds) = false;    % drop the chosen regions

restPc = pointCloud( ...
    pc.Location(mask, :), ...
    'Color',      pc.Color(mask, :) ...
);

chosenPc = pointCloud(allSegmentPts, 'Color', allSegmentCols);

if writeout
    [outdir, n, e] = fileparts(pcfile);
    out_txt = fullfile(outdir, [scenario, '.txt'])
    save_bboxes(bboxes, out_txt, 'w');
    nbboxes = load_bboxes(out_txt);
else
    nbboxes = bboxes;
end

areas = compute_bbox_areas(nbboxes);
disp(table((1:numel(areas))', areas, ...
    'VariableNames',{'Region','Area'}));
areaMean   = mean(areas);             % arithmetic mean
areaMedian = median(areas);           % median
areaStd    = std(areas);              % sample (unbiased) standard deviation

% display results
fprintf('BBox area: mean = %.4f, median = %.4f, std = %.4f\n', ...
        areaMean, areaMedian, areaStd);

figure;
pcshow(cullPc);
% pcshow(restPc);

hold on;
allBoxPts   = [];
allBoxCols  = [];
% draw each bounding box (green edges)
for k = 1:numel(nbboxes)
    C = nbboxes{k};  % 8×3: first 1–4 bottom, 5–8 top
    idxs = find_points_in_box(pc, C);
    pts  = pc.Location(idxs, :);        % Nx3 coordinates
    col  = repmat(colors(k,:), numel(idxs), 1);  % Nx3 RGB

    allBoxPts  = [allBoxPts;  pts];           
    allBoxCols = [allBoxCols; col];           

    % define the 12 edges of a box
    E = [1 2; 2 3; 3 4; 4 1; ...    % bottom face
         5 6; 6 7; 7 8; 8 5; ...    % top face
         (1:4)' (5:8)'];            % vertical edges
    for e = 1:size(E,1)
        p1 = C(E(e,1), :);
        p2 = C(E(e,2), :);
        line([p1(1) p2(1)], [p1(2) p2(2)], [p1(3) p2(3)], ...
             'Color',rgbColor,'LineWidth',1.5);
    end
end
chosenBoxPc = pointCloud(allBoxPts, 'Color', allBoxCols);
pcshow(chosenBoxPc);
% pcshow(chosenPc, 'MarkerSize', 50);

hold off;

xlabel('X (m)');
ylabel('Y (m)');
zlabel('Z (m)');
ylim([-15, 15]);
% title([scenario, ' patches']);
axis equal;
view(0, 90);

% This is no better than manually saving from the figure dialog.
% hFigure = gcf;
% exportgraphics(hFigure, [outdir, '/', scenario, '.pdf'], ...
%                'BackgroundColor', 'none', ...
%                'ContentType',    'vector');

check_plane(pc, nbboxes);

