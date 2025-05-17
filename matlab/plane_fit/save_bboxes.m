function save_bboxes(bboxes, out_txt, mode)
    %SAVE_BBOXES  Write a cell-array of 8×3 bbox corners to text
    %
    % Usage:
    %   save_bboxes(bboxes, filename, 'w');  % overwrite
    %   save_bboxes(bboxes, filename, 'a');  % append
    %
    fid = fopen(out_txt, mode);
    if fid == -1
        error('Could not open %s for writing.', out_txt);
    end

    n = numel(bboxes);
    for k = 1:n
        B = bboxes{k};                    % 8×3
        coords = reshape(B', 1, []);      % 1×24: [x1 y1 z1 … x8 y8 z8]
        fmt    = [repmat('%f ', 1, numel(coords)) '\n'];
        fprintf(fid, fmt, coords);
    end

    fclose(fid);
end
