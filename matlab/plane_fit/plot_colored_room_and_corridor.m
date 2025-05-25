function plot_colored_room_and_corridor()
result_dir = 'F:\jhuai\lidar-phone-construction\labelcloud\pointclouds';
room = fullfile(result_dir, '2024_12_05_16_33_55\color-0.02\aggregated_white_only_aligned.ply');
corridor = fullfile(result_dir, '2024_12_05_16_11_23\color-0.02\aggregated_white_only_aligned.ply');
addpath('E:\jhuai\tools\export_fig');

close all;
pc = pcread(corridor);
polygon3d = [ 9.7083   0.3576    1.9722;
   -5.6206    0.4645    1.3023;
   -6.3619  -36.0255   -0.1975;
  -12.5410  -36.5770   -2.8595;
  -12.5423  -42.1529    0.3646;
    1.0353  -42.3641   -4.3962;
    1.0285   -6.1198   -4.3397;
    9.8127   -5.7070   -3.0677];
cropped_pc = crop_pc_by_polygon(pc, polygon3d(:, 1), polygon3d(:, 2));

if 0
figure;
pcshow(cropped_pc); 
hold on;
% plot3(polygon3d(:,1), polygon3d(:,2), polygon3d(:,3), 'ro', 'MarkerSize', 6, 'LineWidth', 1.5);
% k = [1:size(polygon3d,1), 1];
% plot3(polygon3d(k,1), polygon3d(k,2), polygon3d(k,3), 'r-', 'LineWidth', 2);
% for i = 1:size(polygon3d, 1)
%     text(polygon3d(i,1), polygon3d(i,2), polygon3d(i,3), ...
%         sprintf('  %d', i), 'Color', 'yellow', 'FontSize', 10, 'FontWeight', 'bold');
% end

xlabel('X (m)', 'Color', 'k');
ylabel('Y (m)', 'Color', 'k');
zlabel('Z (m)', 'Color', 'k');
axis equal;
xlim([-15, 10]);
ylim([-45, 10]);
zlim([-6, 5]);
view([-89.5664, 13.76]); % this can be obtained by [az, el] = view; after interactively rotating the figure
set(gcf, 'Color', 'white');
axis off

end

if 0
pc = pcread(room);
figure;
pcshow(pc);
xlabel('X (m)', 'Color', 'k');
ylabel('Y (m)', 'Color', 'k');
zlabel('Z (m)', 'Color', 'k');
axis equal;
xlim([-13, 3]);
ylim([-3, 6]);

set(gcf, 'Color', 'white');
axis off
end


if 1
pc = pcread(room);
figure;
mask = pc.Location(:,3) < 1.8;
if ~isempty(pc.Color)
    filtered_pc = pointCloud(pc.Location(mask, :), 'Color', pc.Color(mask, :));
else
    filtered_pc = pointCloud(pc.Location(mask, :));
end
pcshow(filtered_pc);
xlabel('X (m)', 'Color', 'k');
ylabel('Y (m)', 'Color', 'k');
zlabel('Z (m)', 'Color', 'k');
axis equal;
xlim([-13, 3]);
ylim([-3, 6]);

set(gcf, 'Color', 'white');
axis off
end
end
