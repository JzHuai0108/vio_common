
function c8 = convertcolor(c16)
    % fprintf('Converting uint16 color to uint8...\n');
    c8 = uint8(c16);
    for i=1:3
        x = max(c16(:, i));
        n = min(c16(:, i));
        c8(:, i) = (double(c16(:, i)) - double(n)) * 255 / double(x - n);
    end
end