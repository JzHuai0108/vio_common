function cropImageSeries(photo_dir)
% crop a series of images where the crop regions are drawn interactively.

% Once a crop region is drawn, you may double click on the region or
% right click to crop it, the cropped image will be saved along the
% original image.
% If the crop operation is aborted somewhere, no crop will be saved.

photo_list = dir([photo_dir, '*.jpg']);
photos = {photo_list.name};
for i = 1:length(photos)
    fn = [photo_dir, photos{i}];
    I = imread(fn);
    imshow(I);
    [J, ~] = imcrop(I);
    if ~isempty(J)
        ofn = [photo_dir, photos{i}(1:end-4), '-crop.jpg'];
        imwrite(J, ofn);
    end
end
close all; % close the last shown image.
end
