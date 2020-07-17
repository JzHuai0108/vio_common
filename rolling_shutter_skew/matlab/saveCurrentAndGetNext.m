function saveCurrentAndGetNext(hObject, eventdata, imageList, outputDir)

global currentIndex;
imageName = imageList{currentIndex};
global t_r;
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

fileID = fopen([outputDir, '/interactive-rs-skew-estimate.txt'], 'a');
fprintf(fileID, '%s %.4f ms\n', imageName, t_r);
fclose(fileID);
global numDrawnImages;
numDrawnImages = numDrawnImages + 1;

getNextFrame(hObject, eventdata, imageList);
end
