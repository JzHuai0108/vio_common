function precompute_transforms()
% Read each initial_poses file, compute the combined 4x4 transform, and
% save it as *_precomputed.txt for use by the main evaluation script.
%
% Transform conventions (from the pose file headers):
%   2-block files: p_ref = B * A * p_est   ->  T = B * A
%   3-block files: p_ref = C * B * A * p_est -> T = C * B * A
%
% Output: one *_precomputed.txt per pose file, written to the same folder.

poses_dir = '/media/jhuai/T5EVO/jhuai/results/rss26/hiltislam2022/initial_poses';

pose_files = dir(fullfile(poses_dir, '*_est.txt'));
if isempty(pose_files)
    error('No pose files found in %s', poses_dir);
end

for k = 1:numel(pose_files)
    fname = fullfile(poses_dir, pose_files(k).name);
    T = load_combined_transform(fname);

    [d, n, ~] = fileparts(fname);
    out_file = fullfile(d, [n '_precomputed.txt']);
    writematrix(T, out_file, 'Delimiter', ' ');
    fprintf('Saved %s\n', out_file);
end
end

% -------------------------------------------------------------------------
function T = load_combined_transform(fname)
% Parse a pose file, skip comment lines (#), collect 4x4 matrix blocks,
% then compute T = M_last * ... * M_2 * M_1.

fid = fopen(fname, 'r');
if fid < 0
    error('Cannot open %s', fname);
end
data_lines = {};
while ~feof(fid)
    line = strtrim(fgetl(fid));
    if ischar(line) && ~isempty(line) && line(1) ~= '#'
        data_lines{end+1} = line; %#ok<AGROW>
    end
end
fclose(fid);

n_lines = numel(data_lines);
if mod(n_lines, 4) ~= 0
    error('%s: expected a multiple of 4 data lines, got %d', fname, n_lines);
end
n_blocks = n_lines / 4;

matrices = cell(n_blocks, 1);
for b = 1:n_blocks
    M = zeros(4, 4);
    for r = 1:4
        vals = sscanf(data_lines{(b-1)*4 + r}, '%f');
        if numel(vals) ~= 4
            error('%s block %d row %d: expected 4 values', fname, b, r);
        end
        M(r, :) = vals';
    end
    matrices{b} = M;
end

% Multiply right-to-left so that T = M_n * ... * M_1
% e.g. for 3 blocks: T = C * B * A = matrices{3} * matrices{2} * matrices{1}
T = eye(4);
for b = n_blocks:-1:1
    T = T * matrices{b};
end
end
