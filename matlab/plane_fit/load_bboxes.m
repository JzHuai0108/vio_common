
function bboxes = load_bboxes(in_txt)
    %LOAD_BBOXES  Read back bboxes from a text file written by save_bboxes
    %
    % Usage:
    %   bboxes = load_bboxes(filename);
    %
    % Returns a cell array of size N×1, each entry an 8×3 double.

    fid = fopen(in_txt, 'r');
    if fid == -1
        error('Could not open %s for reading.', in_txt);
    end

    bboxes = {};
    k = 0;
    while ~feof(fid)
        line = fgetl(fid);
        if ~ischar(line)
            break;
        end

        vals = sscanf(line, '%f');     % should be 24×1
        if numel(vals) ~= 24
            warning('Skipping line %d: found %d values (expected 24).', ...
                    k+1, numel(vals));
            continue;
        end

        k = k + 1;
        % reshape into 3×8 then transpose → 8×3
        bboxes{k} = reshape(vals, 3, 8)';  
    end

    fclose(fid);
end
