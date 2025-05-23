function save_tform(tform, outfile)
fid = fopen(outfile,'w');
q = rotm2quat(tform.R);
p = tform.Translation;
fprintf(fid,'%.9f %.9f %.9f %.15f %.15f %.15f %.15f\n', ....
    p(1), p(2), p(3), q(2), q(3), q(4), q(1));
fclose(fid);
end
