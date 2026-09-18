function [fig, stats] = plot_sensor_bag_header_offsets(...
    seq_dir, output_file, jump_window_frames)
%PLOT_SENSOR_BAG_HEADER_OFFSETS Plot ROS bag time minus sensor header time.
%
%   plot_sensor_bag_header_offsets(SEQ_DIR) reads the four *_times.txt files
%   produced by extract_sensor_times_from_rosbag.py and plots:
%       LiDAR:       t_bag - t_lidar_header
%       IMU:         t_bag - t_imu_header
%       Camera L/R:  t_bag - t_camera_header
%
%   [FIG, STATS] = plot_sensor_bag_header_offsets(SEQ_DIR, OUTPUT_FILE,
%   JUMP_WINDOW_FRAMES) additionally saves the figure when OUTPUT_FILE is
%   nonempty. The camera jump is the largest adjacent delay change. Its
%   reported magnitude is the difference between the means of the immediately
%   preceding and following JUMP_WINDOW_FRAMES (default: 20) camera frames.
%   Delays are plotted in milliseconds. The x axes use a common sensor-header
%   time origin, computed from split sec/nsec columns to avoid epoch precision
%   loss.
%
% Example:
%   seq_dir = '/media/user/data1/shared/2026-07-26-lidar-contest-data/seq1';
%   plot_sensor_bag_header_offsets(seq_dir);

if nargin < 1 || isempty(seq_dir)
    seq_dir = ...
        '/media/user/data1/shared/2026-07-26-lidar-contest-data/seq1';
end
if nargin < 2
    output_file = '';
end
if nargin < 3 || isempty(jump_window_frames)
    jump_window_frames = 20;
end
validateattributes(jump_window_frames, {'numeric'}, ...
    {'scalar', 'integer', 'positive', 'finite'}, mfilename, ...
    'jump_window_frames');

seq_dir = char(seq_dir);
lidar = load_time_file(fullfile(seq_dir, 'lidar_times.txt'), true);
imu = load_time_file(fullfile(seq_dir, 'imu_times.txt'), false);
camera_left = load_time_file(...
    fullfile(seq_dir, 'camera_left_times.txt'), false);
camera_right = load_time_file(...
    fullfile(seq_dir, 'camera_right_times.txt'), false);

% Form a common, precise time origin without first constructing large epoch
% timestamps as doubles.
base_sec = min([lidar.header_sec(1), imu.header_sec(1), ...
                camera_left.header_sec(1), camera_right.header_sec(1)]);
first_times = [(lidar.header_sec(1) - base_sec) + ...
                   lidar.header_nsec(1) * 1e-9; ...
               (imu.header_sec(1) - base_sec) + ...
                   imu.header_nsec(1) * 1e-9; ...
               (camera_left.header_sec(1) - base_sec) + ...
                   camera_left.header_nsec(1) * 1e-9; ...
               (camera_right.header_sec(1) - base_sec) + ...
                   camera_right.header_nsec(1) * 1e-9];
t0 = min(first_times);
lidar.t = relative_seconds(lidar, base_sec, t0);
% FastLIO trajectory poses are timestamped at the scan end, which is the
% maximum per-point timestamp saved in lidar_times.txt.
lidar.pose_t = relative_sec_nsec(...
    lidar.pose_sec, lidar.pose_nsec, base_sec, t0);
imu.t = relative_seconds(imu, base_sec, t0);
camera_left.t = relative_seconds(camera_left, base_sec, t0);
camera_right.t = relative_seconds(camera_right, base_sec, t0);

stats.lidar = summarize_delay(lidar.delay_ms);
stats.imu = summarize_delay(imu.delay_ms);
stats.camera_left = summarize_delay(camera_left.delay_ms);
stats.camera_right = summarize_delay(camera_right.delay_ms);
stats.camera_left.jump = detect_largest_jump(...
    camera_left, jump_window_frames);
stats.camera_right.jump = detect_largest_jump(...
    camera_right, jump_window_frames);
stats.camera_jump.mean_left_right_ms = mean(...
    [stats.camera_left.jump.mean_jump_ms, ...
     stats.camera_right.jump.mean_jump_ms]);
stats.camera_jump.elapsed_s = mean(...
    [stats.camera_left.jump.elapsed_s, ...
     stats.camera_right.jump.elapsed_s]);
stats.lidar_pose_bracket = find_lidar_pose_bracket(...
    lidar, stats.camera_jump.elapsed_s);

fprintf('Bag-minus-header delay statistics for %s (ms):\n', seq_dir);
print_stats('LiDAR', stats.lidar);
print_stats('IMU', stats.imu);
print_stats('Camera left', stats.camera_left);
print_stats('Camera right', stats.camera_right);
fprintf(['Largest camera delay step, using %d frames immediately before ' ...
         'and after the boundary:\n'], jump_window_frames);
print_jump('Camera left', stats.camera_left.jump);
print_jump('Camera right', stats.camera_right.jump);
fprintf('  Mean left/right step: %.6f ms\n', ...
        stats.camera_jump.mean_left_right_ms);
print_lidar_pose_bracket(stats.lidar_pose_bracket);

fig = figure('Name', 'ROS bag minus sensor header time', ...
             'Color', 'w');
layout = tiledlayout(fig, 3, 1, 'TileSpacing', 'compact', ...
                     'Padding', 'compact');

ax(1) = nexttile(layout);
plot(lidar.t, lidar.delay_ms, '-', 'LineWidth', 0.8, ...
     'Color', [0.0000, 0.4470, 0.7410]);
grid on;
ylabel('Delay (ms)');
title(sprintf('LiDAR (%d scans)', stats.lidar.count));

ax(2) = nexttile(layout);
plot(imu.t, imu.delay_ms, '-', 'LineWidth', 0.6, ...
     'Color', [0.8500, 0.3250, 0.0980]);
grid on;
ylabel('Delay (ms)');
title(sprintf('IMU (%d samples)', stats.imu.count));

ax(3) = nexttile(layout);
plot(camera_left.t, camera_left.delay_ms, '-', 'LineWidth', 0.8, ...
     'DisplayName', 'Left', 'Color', [0.4660, 0.6740, 0.1880]);
hold on;
plot(camera_right.t, camera_right.delay_ms, '-', 'LineWidth', 0.8, ...
     'DisplayName', 'Right', 'Color', [0.4940, 0.1840, 0.5560]);
xline(stats.camera_jump.elapsed_s, '--k', ...
      sprintf('mean step %.3f ms', stats.camera_jump.mean_left_right_ms), ...
      'LineWidth', 1.0, 'HandleVisibility', 'off', ...
      'LabelOrientation', 'horizontal', ...
      'LabelHorizontalAlignment', 'left', ...
      'LabelVerticalAlignment', 'bottom');
hold off;
grid on;
xlabel('Elapsed sensor header time (s)');
ylabel('Delay (ms)');
title(sprintf('Cameras (left %d, right %d frames)', ...
      stats.camera_left.count, stats.camera_right.count));
legend('Location', 'best');

linkaxes(ax, 'x');
title(layout, sprintf('%s: t_{bag} - t_{header}', ...
      get_sequence_name(seq_dir)), 'Interpreter', 'tex');

if ~isempty(output_file)
    output_file = char(output_file);
    output_parent = fileparts(output_file);
    if ~isempty(output_parent) && ~isfolder(output_parent)
        mkdir(output_parent);
    end
    exportgraphics(fig, output_file, 'Resolution', 180);
    fprintf('Saved figure to %s\n', output_file);
end
end


function stream = load_time_file(filename, is_lidar)
if ~isfile(filename)
    error('Timestamp file does not exist: %s', filename);
end
data = readmatrix(filename, 'FileType', 'text', 'CommentStyle', '#');
if isempty(data)
    error('Timestamp file contains no data rows: %s', filename);
end

if is_lidar
    % LiDAR columns 7/8 are header sec/nsec; column 16 is delay_ns.
    expected_columns = 16;
    header_sec_column = 7;
    header_nsec_column = 8;
    bag_sec_column = 9;
    bag_nsec_column = 10;
    pose_sec_column = 13;
    pose_nsec_column = 14;
else
    % IMU/camera columns 5/6 are header sec/nsec; column 9 is delay_ns.
    expected_columns = 9;
    header_sec_column = 5;
    header_nsec_column = 6;
    bag_sec_column = 7;
    bag_nsec_column = 8;
end
if size(data, 2) ~= expected_columns
    error(['Unexpected column count in %s: got %d, expected %d. ' ...
           'Regenerate it with extract_sensor_times_from_rosbag.py.'], ...
          filename, size(data, 2), expected_columns);
end

stream.header_sec = data(:, header_sec_column);
stream.header_nsec = data(:, header_nsec_column);
stream.bag_sec = data(:, bag_sec_column);
stream.bag_nsec = data(:, bag_nsec_column);
stream.index = data(:, 1);
if is_lidar
    stream.pose_sec = data(:, pose_sec_column);
    stream.pose_nsec = data(:, pose_nsec_column);
end
stream.delay_ms = data(:, end) * 1e-6;
if any(~isfinite(stream.header_sec)) || ...
        any(~isfinite(stream.header_nsec)) || ...
        any(~isfinite(stream.bag_sec)) || ...
        any(~isfinite(stream.bag_nsec)) || ...
        any(~isfinite(stream.index)) || ...
        any(~isfinite(stream.delay_ms))
    error('Non-finite timestamp data found in %s', filename);
end
if is_lidar && (any(~isfinite(stream.pose_sec)) || ...
        any(~isfinite(stream.pose_nsec)))
    error('Non-finite LiDAR pose timestamp data found in %s', filename);
end
end


function t = relative_seconds(stream, base_sec, t0)
t = relative_sec_nsec(stream.header_sec, stream.header_nsec, base_sec, t0);
end


function t = relative_sec_nsec(sec, nsec, base_sec, t0)
t = (sec - base_sec) + nsec * 1e-9 - t0;
end


function summary = summarize_delay(delay_ms)
summary.count = numel(delay_ms);
summary.min_ms = min(delay_ms);
summary.max_ms = max(delay_ms);
summary.mean_ms = mean(delay_ms);
summary.median_ms = median(delay_ms);
summary.std_ms = std(delay_ms);
end


function jump = detect_largest_jump(stream, window_frames)
if numel(stream.delay_ms) < 2
    error('At least two samples are required to detect a jump.');
end
[~, before_index] = max(abs(diff(stream.delay_ms)));
after_index = before_index + 1;
before_count = min(window_frames, before_index);
after_count = min(window_frames, numel(stream.delay_ms) - after_index + 1);
before_rows = (before_index - before_count + 1):before_index;
after_rows = after_index:(after_index + after_count - 1);

jump.row_index_zero_based = after_index - 1;
jump.elapsed_s = stream.t(after_index);
jump.header_sec = stream.header_sec(after_index);
jump.header_nsec = stream.header_nsec(after_index);
jump.bag_sec = stream.bag_sec(after_index);
jump.bag_nsec = stream.bag_nsec(after_index);
jump.adjacent_jump_ms = ...
    stream.delay_ms(after_index) - stream.delay_ms(before_index);
jump.before_count = before_count;
jump.after_count = after_count;
jump.before_mean_ms = mean(stream.delay_ms(before_rows));
jump.after_mean_ms = mean(stream.delay_ms(after_rows));
jump.mean_jump_ms = jump.after_mean_ms - jump.before_mean_ms;
end


function bracket = find_lidar_pose_bracket(lidar, jump_elapsed_s)
before_row = find(lidar.pose_t < jump_elapsed_s, 1, 'last');
after_row = find(lidar.pose_t > jump_elapsed_s, 1, 'first');
if isempty(before_row) || isempty(after_row)
    error('Camera jump is not bracketed by LiDAR pose timestamps.');
end
bracket.jump_elapsed_s = jump_elapsed_s;
bracket.before = make_lidar_pose_entry(lidar, before_row, jump_elapsed_s);
bracket.after = make_lidar_pose_entry(lidar, after_row, jump_elapsed_s);
end


function entry = make_lidar_pose_entry(lidar, row, jump_elapsed_s)
entry.index = lidar.index(row);
entry.sec = lidar.pose_sec(row);
entry.nsec = lidar.pose_nsec(row);
entry.elapsed_s = lidar.pose_t(row);
entry.delta_ms = (lidar.pose_t(row) - jump_elapsed_s) * 1e3;
end


function print_stats(name, summary)
fprintf(['  %-12s n=%-7d min=%9.3f max=%9.3f mean=%9.3f ' ...
         'median=%9.3f std=%9.3f\n'], ...
        name, summary.count, summary.min_ms, summary.max_ms, ...
        summary.mean_ms, summary.median_ms, summary.std_ms);
end


function print_jump(name, jump)
fprintf(['  %-12s row=%d header=%s bag=%s elapsed=%.9f s\n' ...
         '               adjacent=%+.6f ms, before_mean(%d)=%' ...
         '.6f ms, after_mean(%d)=%.6f ms, mean_step=%+.6f ms\n'], ...
        name, jump.row_index_zero_based, ...
        format_ros_time(jump.header_sec, jump.header_nsec), ...
        format_ros_time(jump.bag_sec, jump.bag_nsec), jump.elapsed_s, ...
        jump.adjacent_jump_ms, jump.before_count, jump.before_mean_ms, ...
        jump.after_count, jump.after_mean_ms, jump.mean_jump_ms);
end


function print_lidar_pose_bracket(bracket)
fprintf('Closest LiDAR pose timestamps bracketing the camera jump:\n');
fprintf(['  Before: index=%.0f time=%s elapsed=%.9f s ' ...
         '(pose-jump=%+.6f ms)\n'], ...
        bracket.before.index, ...
        format_ros_time(bracket.before.sec, bracket.before.nsec), ...
        bracket.before.elapsed_s, bracket.before.delta_ms);
fprintf(['  After:  index=%.0f time=%s elapsed=%.9f s ' ...
         '(pose-jump=%+.6f ms)\n'], ...
        bracket.after.index, ...
        format_ros_time(bracket.after.sec, bracket.after.nsec), ...
        bracket.after.elapsed_s, bracket.after.delta_ms);
end


function text = format_ros_time(sec, nsec)
text = sprintf('%.0f.%09.0f', sec, nsec);
end


function name = get_sequence_name(seq_dir)
[~, name] = fileparts(seq_dir);
if isempty(name)
    name = seq_dir;
end
end
