function swap_files(fileA, fileB)
    % Ensure both files exist
    if ~isfile(fileA)
        error('File %s does not exist.', fileA);
    end
    if ~isfile(fileB)
        error('File %s does not exist.', fileB);
    end

    % Create a temporary filename that won't collide
    tmpFile = [tempname(fileparts(fileA)), '_tmp_swap'];

    % Rename A to tmp, B to A, then tmp to B
    movefile(fileA, tmpFile);
    movefile(fileB, fileA);
    movefile(tmpFile, fileB);
end
