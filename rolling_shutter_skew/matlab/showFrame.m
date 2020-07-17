function found = showFrame(imageList, frameIndex)
if frameIndex > length(imageList) || frameIndex < 1
    fprintf('Invalid frame number %d.\n', frameIndex);
    found = 0;
    return;
end

imageName = imageList{frameIndex};
if isfile(imageName)
    I = imread(imageName);
    imshow(I);
    
    delete(findall(gcf,'type','annotation'));
    helpStr = ['After click Draw lines, you should draw two sloping lines and left and right vertical ', ...
        'lines for the current frame', newline, 'before you can move on to another ', ...
        'frame. Otherwise,you may need to', newline, 'close many spurious windows ', ...
        'when you are quitting the program.'];
    text(.5,0, helpStr, ...
        'horiz','center',...
        'vert','top',...
        'FontSize',10,...
        'units','normalized');
    
    [~, name, ~] = fileparts(imageName);
    global numDrawnImages;
    str = sprintf('Image %s #Saved images %d', name, numDrawnImages);
    title(str);
    global t_r;
    t_r = -1; % reset t_r
    found = 1;
else
    fprintf('Frame %s does not exist!\n', imageName);
    found = 0;
end
end