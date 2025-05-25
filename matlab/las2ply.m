function las2ply(lasfile)
% Convert a .las file to .ply and save it in the same folder

% Extract file parts
[folder, name, ~] = fileparts(lasfile);
plyfile = fullfile(folder, [name, '.ply']);

% Read LAS file
lasReader = lasFileReader(lasfile);
ptCloud = readPointCloud(lasReader);

% Write to PLY
pcwrite(ptCloud, plyfile, 'Encoding', 'binary');  % use 'binary' for smaller size
fprintf('Saved PLY to: %s\n', plyfile);
end
