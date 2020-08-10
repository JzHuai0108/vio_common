function getPreviousFrame(hObject, eventdata, imageList)
global currentIndex;
previousIndex = currentIndex - 1;
found = showFrame(imageList, previousIndex);
if found
    currentIndex = previousIndex;
end
end
