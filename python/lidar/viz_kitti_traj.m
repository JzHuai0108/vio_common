% MATLAB script to visualize KITTI trajectory with labeled points

% Path to KITTI poses file
poses_file = '/media/jhuai/BackupPlus1/jhuai/data/KITTI/data_odometry_poses/dataset/poses/00.txt';

% Read the poses: each row has 12 numbers (3x4 matrix flattened row-wise)
poses_data = dlmread(poses_file);

num_frames = size(poses_data, 1);

% Initialize arrays for trajectory
traj = zeros(num_frames, 3); % x, y, z positions

% Extract translation (last column of each 3x4 matrix)
for i = 1:num_frames
    T = reshape(poses_data(i,:), 4, 3)'; % reshape to 3x4
    % Actually, dlmread gives row-wise 12 numbers: first 3 numbers are first row of 3x4
    % So we can extract directly
    T_matrix = [poses_data(i,1:4); poses_data(i,5:8); poses_data(i,9:12); 0 0 0 1];
    traj(i,:) = T_matrix(1:3,4)';
end

% Plot trajectory
figure;
plot(traj(:,1), traj(:,3), 'b-', 'LineWidth', 1.5); % x vs z for top-down view
hold on;
grid on;
xlabel('X (meters)');
ylabel('Z (meters)');
title('KITTI Sequence 00 Trajectory (Top-Down View)');

% Label points: 1, 5, 10, 15, ..., num_frames
label_step = [1,5,10,15]; % can dynamically generate
label_step = 1:5:num_frames; % every 5 frames

for i = label_step
    text(traj(i,1), traj(i,3), num2str(i), 'FontSize', 8, 'Color', 'r');
end

axis equal;