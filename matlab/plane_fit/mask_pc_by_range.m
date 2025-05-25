function pcOut = mask_pc_by_range(pcIn, zLim)
    loc = pcIn.Location;
    mask = loc(:,3) >= zLim(1) & loc(:,3) <= zLim(2);
    idx  = find(mask);
    pcOut = select(pcIn, idx);
end
