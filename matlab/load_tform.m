function tform = load_tform(infile)
    pq_src = readmatrix(infile,'Delimiter',' ');
    T_src  = T_from_Pq(pq_src);
    tform = rigidtform3d(T_src(1:3,1:3), T_src(1:3,4));
end
