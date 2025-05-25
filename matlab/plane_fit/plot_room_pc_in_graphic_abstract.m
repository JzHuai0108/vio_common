function plot_room_pc_in_graphic_abstract()

result_dir = 'F:\jhuai\lidar-phone-construction\labelcloud\pointclouds';
room = fullfile(result_dir, '2024_12_05_16_33_55\color-0.02\aggregated_white_only_aligned.ply');
room_intensity = fullfile(result_dir, '2024_12_05_16_33_55\original_aligned.pcd');
addpath('E:\jhuai\tools\export_fig');

close all;
pc = pcread(room);
mask = pc.Location(:,3) < 1.8;
pcNoTop = pointCloud(pc.Location(mask, :), 'Color', pc.Color(mask, :));


figure;
pcshow(pcNoTop);
xlabel('X (m)');
ylabel('Y (m)');
zlabel('Z (m)');
axis equal;
xlim([-13, 3]);
ylim([-3, 6]);

% Set figure background to sky blue
set(gcf, 'Color', [0.53, 0.81, 0.92]);

% Optional: also set axes background to match
ax = gca;
ax.Color = [0.53, 0.81, 0.92];
axis off

set(gca, 'Color', 'none');
set(gcf, 'Color', 'none');
drawnow;
export_fig('room_color.png', '-png', '-transparent');

ptCloud = pcread(room_intensity);
xyz = ptCloud.Location;
intensity = ptCloud.Intensity;

% Normalize intensity to [0, 1]
intensity = double(intensity);  % ensure it's float
intensity = (intensity - min(intensity)) / (max(intensity) - min(intensity));

% Map intensity to color using a colormap
colormapName = jet(256);  % Or 'hot', 'parula', etc.
colorIdx = round(intensity * 255) + 1;
colorIdx(isnan(colorIdx)) = 1;  % handle NaNs
colorMapped = uint8(colormapName(colorIdx, :) * 255);
ptCloudColored = pointCloud(xyz, 'Color', colorMapped);

mask = xyz(:,3) < 1.8;
pcIntNoTop = pointCloud(ptCloudColored.Location(mask, :), 'Color', ptCloudColored.Color(mask, :));

figure;
pcshow(pcIntNoTop);
axis equal;
xlim([-13, 3]);
ylim([-3, 6]);

% Set figure background to sky blue
set(gcf, 'Color', [0.53, 0.81, 0.92]);

% Optional: also set axes background to match
ax = gca;
ax.Color = [0.53, 0.81, 0.92];
axis off

set(gca, 'Color', 'none');
set(gcf, 'Color', 'none');
drawnow;
export_fig('room_intensity.png', '-png', '-transparent');
end
