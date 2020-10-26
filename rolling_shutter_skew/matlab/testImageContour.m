function testImageContour()
% test finding contours in an image.
% It involves morphological operations, corners, and contours.
imageName = '../data/024.png';

originalBW = imread(imageName);
close all;
figure(1);
imshow(originalBW);
se = strel('disk',200); % a value greater than 100 usually suffices,
% but it does not harm to enlarge it.

closeBW = imclose(originalBW,se);
figure(2);
imshow(closeBW);

nb = im2bw(closeBW, graythresh(closeBW));
labeledImage = bwlabel(nb);
measurements = regionprops(labeledImage, 'Area');
allAreas = [measurements.Area];
figure;
imshow(nb)

corners = corner(nb, 'Harris', 'SensitivityFactor', 0.04);
hold on
plot(corners(:,1),corners(:,2),'r*');

[C, h] = imcontour(closeBW);
coutourSets = cell(1000, 1);
areas = [1000, 1];
count = 0;
index = 1;
while index < size(C, 2) && count < 1000
    length = int32(C(2, index));
    borderPoints = C(:, index : index + length);
    index = index + length + 1;
    count = count + 1;
    coutourSets{count} = borderPoints;
    areas(count) = polyarea(borderPoints(1, 2:end), borderPoints(2, 2:end));
end
% each contour first column, (level, length)
coutourSets = coutourSets(1:count);
disp(coutourSets);
disp(areas);

end
