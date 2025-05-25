function plot_plane_heatmaps(csv_file)
% Plot RMSE and inlier ratio heatmaps for wall and floor separately
if nargin < 1
    csv_file = 'F:\jhuai\lidar-phone-construction\labelcloud\plane_stats.csv';
end
close all;

T = readtable(csv_file, 'Delimiter', ',');
fprintf('field names:\n');
disp(T.Properties.VariableNames);

T.type = repmat("", height(T), 1);
T.cloud = repmat("", height(T), 1);
for i = 1:height(T)
    path = T.surfel_file{i};
    if contains(path, 'walls', 'IgnoreCase', true)
        T.type(i) = "wall";
    elseif contains(path, 'floors', 'IgnoreCase', true)
        T.type(i) = "floor";
    end
    path_parts = strsplit(T.filename{i}, '/');
    match_idx = find( ...
        startsWith(path_parts, '2025') | strcmp(path_parts, 'ref_tls'), 1, 'first');
    if ~isempty(match_idx)
        T.cloud(i) = path_parts{match_idx};
    else
        T.cloud(i) = "unknown";
    end
end

% Get unique cloud names for rows (assume 5)
clouds = unique(T.cloud, 'stable');
fprintf('The found clouds of names\n');
disp(clouds);
if numel(clouds) ~= 5
    fprintf('Error: the found clouds are not equal to 5.\n');
    return;
end
cloud_indices = containers.Map(clouds, 1:length(clouds));

% Initialize 4 matrices (5×10)
rmse_wall = nan(5,10);
ratio_wall = nan(5,10);
rmse_floor = nan(5,10);
ratio_floor = nan(5,10);

% Fill matrices
for i = 1:height(T)
    row = cloud_indices(T.cloud(i));
    col = T.box_id(i);
    if T.type(i) == "wall"
        rmse_wall(row, col) = T.rmse_cm(i);
        ratio_wall(row, col) = T.inlier_ratio_pct(i);
    elseif T.type(i) == "floor"
        rmse_floor(row, col) = T.rmse_cm(i);
        ratio_floor(row, col) = T.inlier_ratio_pct(i);
    end
end

% Initialize result table
mean_table = table;
mean_table.Cloud = clouds;
mean_table.RMSE_Wall = nan(height(mean_table), 1);
mean_table.Inlier_Wall = nan(height(mean_table), 1);
mean_table.RMSE_Floor = nan(height(mean_table), 1);
mean_table.Inlier_Floor = nan(height(mean_table), 1);

% Compute means
for i = 1:height(mean_table)
    cloud_name = mean_table.Cloud(i);
    mask_wall = (T.cloud == cloud_name) & (T.type == "wall");
    mask_floor = (T.cloud == cloud_name) & (T.type == "floor");

    mean_table.RMSE_Wall(i) = mean(T.rmse_cm(mask_wall), 'omitnan');
    mean_table.Inlier_Wall(i) = mean(T.inlier_ratio_pct(mask_wall), 'omitnan');
    mean_table.RMSE_Floor(i) = mean(T.rmse_cm(mask_floor), 'omitnan');
    mean_table.Inlier_Floor(i) = mean(T.inlier_ratio_pct(mask_floor), 'omitnan');
end

% Round values: RMSE to 2 decimals, inlier ratio to 1 decimal
mean_table.RMSE_Wall     = round(mean_table.RMSE_Wall, 2);
mean_table.RMSE_Floor    = round(mean_table.RMSE_Floor, 2);
mean_table.Inlier_Wall   = round(mean_table.Inlier_Wall, 1);
mean_table.Inlier_Floor  = round(mean_table.Inlier_Floor, 1);

% Display the summary
disp(mean_table);

% Plotting
plot_heatmap(rmse_wall, 'RMSE (cm) - Wall', false, 'rmse_wall.png');
plot_heatmap(ratio_wall, 'Inlier Ratio (%) - Wall', true, 'inlier_wall.png');
plot_heatmap(rmse_floor, 'RMSE (cm) - Floor', false, 'rmse_floor.png');
plot_heatmap(ratio_floor, 'Inlier Ratio (%) - Floor', true, 'inlier_floor.png');

end

function plot_heatmap(data, titleStr, is_inverted, filename)
    figure;
    imagesc(data);
    colorbar;
    if is_inverted
        colormap(flipud(hot));  % Invert colormap for "lower is better"
    else
        colormap(hot);          % Normal colormap for "higher is better"
    end
    axis equal tight;
    xlabel('Surfel');
    ylabel('Cloud');
    title(titleStr);
    set(gca, 'XTick', 1:10, 'YTick', 1:5);

    if nargin > 3 && ~isempty(filename)
        exportgraphics(gcf, filename, 'BackgroundColor', 'none', 'Resolution', 300);

    end
end
