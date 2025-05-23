function show_boxes_on_pc(nbboxes, pc, colors)
if nargin < 3
    % Pick a random HSV triplet with high saturation & brightness
    h = rand;                      % random hue
    s = 0.7 + 0.3*rand;            % saturation between 0.7–1.0
    v = 0.7 + 0.3*rand;            % value (brightness) between 0.7–1.0
    rgbColor = hsv2rgb([h, s, v]); % 1×3 in [0,1]
    numRegions = numel(nbboxes);
    colors = uint8( repmat(rgbColor*255, numRegions, 1) );
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
             'Color',colors(k,:),'LineWidth',1.5);
    end
    % Compute annotation position (top-center of each box)
    boxTopCenter = mean(C(5:8,:), 1);

    % Annotate box index
    text(boxTopCenter(1), boxTopCenter(2), boxTopCenter(3), ...
        sprintf('%d', k), 'FontSize', 140, 'FontWeight', 'bold', ...
        'Color', colors(k,:), 'HorizontalAlignment', 'center', ...
        'VerticalAlignment', 'bottom');
end
chosenBoxPc = pointCloud(allBoxPts, 'Color', allBoxCols);
pcshow(chosenBoxPc);
% pcshow(chosenPc, 'MarkerSize', 50);

check_plane(pc, nbboxes);

end