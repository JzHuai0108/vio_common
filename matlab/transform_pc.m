function transform_pc(pc_file, tform_txt)
%TRANSFORM_PC Transforms a point cloud using a given transformation.
%   transform_pc(pc_file, tform_txt) reads the point cloud from pc_file,
%   applies the transformation defined in tform_txt, and saves the
%   transformed point cloud to a new file with '_aligned' suffix.

tform = load_tform(tform_txt);
pc = pcread(pc_file);
pc_transformed = pctransform(pc, tform);

[filepath, name, ext] = fileparts(pc_file);
out_pc_file = fullfile(filepath, [name, '_aligned', ext]);
pcwrite(pc_transformed, out_pc_file);

fprintf('Transformed point cloud saved to %s\n', out_pc_file);
end
