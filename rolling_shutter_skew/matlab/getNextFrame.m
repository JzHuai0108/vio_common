function getNextFrame(hObject, eventdata, imageList)
global currentIndex;
nextIndex = currentIndex + 1;
found = showFrame(imageList, nextIndex);
if found
    currentIndex = nextIndex;
end
end
