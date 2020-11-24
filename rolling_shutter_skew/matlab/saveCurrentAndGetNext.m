function saveCurrentAndGetNext(hObject, eventdata, imageList, outputDir)
interactiveStatFile = 'interactive-rolling-shutter-skew.txt';
global currentIndex;
imageName = imageList{currentIndex};
global t_r;
global t_r_h;
if t_r == -1
    fprintf('We do not save the image as t_r is not calculated.\n');
    return;
end

[~, name, ~] = fileparts(imageName);
outputfig = [outputDir, '/', name, '.jpg'];
if exist(outputfig, 'file')==2
    delete(outputfig);
end
set(gcf, 'Color', 'w');
saveas(gcf, outputfig);
global numDrawnImages;
numDrawnImages = numDrawnImages + 1;
fileID = fopen([ outputDir, '/', interactiveStatFile], 'a');
fprintf(fileID, '%s %.4f ms %.4f ms %d\n', imageName, t_r, t_r_h, numDrawnImages);
fclose(fileID);

getNextFrame(hObject, eventdata, imageList);
end
