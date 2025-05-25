function tform = load_tform(tform_txt)
% Load transformation from file (expects x y z qx qy qz qw after an optional header)
fid = fopen(tform_txt, 'r');
line = fgetl(fid);
while ischar(line) && (isempty(line) || startsWith(strtrim(line), '#') || any(isnan(str2double(strsplit(line)))))
    line = fgetl(fid);  % Skip header or non-numeric lines
end
params = sscanf(line, '%f');
fclose(fid);

% Ensure we have exactly 7 values (x y z qx qy qz qw)
if numel(params) ~= 7
    error('Transformation file must contain a line with 7 values: x y z qx qy qz qw');
end

% Create rigid transformation object
translation = params(1:3)';
quaternion = [params(7); params(4:6)]';
rot = quat2rotm(quaternion);
tform = rigidtform3d(rot, translation);
end
