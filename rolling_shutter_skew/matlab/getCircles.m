function [CircleCenter1_X, CircleCenter1_Y, CircleCenter2_X, CircleCenter2_Y, Ave_dist] = getCircles( imageName )
%getCircles :Quickly find the circle, extract the center of the highlight, and obtain the detection circle information
%
%You can use the tool imdistline(d) to determine the search range 
%   to change Rmin and Rmax to achieve good results
%
%You need to adjust RadiusDiff(The accuracy of different distance_X)
%   to achieve good results
%
%Input£º
%   imageName£ºinput image
%Output£º
%   CircleCenter1_X£ºCircle1_X
%   CircleCenter1_Y£ºCircle1_Y
%   CircleCenter2_X£ºCircle2_X
%   CircleCenter2_Y£ºCircle2_Y
%   Ave_dist: Average distance of the abscissa of the center of the circle
%Text ex£º
%   [x1,y1,x2,y2,d] = getCircles('C:\Users\admin\Desktop\raw\00001.jpg')

img = imread(imageName);
imshow(img)

%   Determine the search range tool
d = imdistline;
%   delete(d)

%   You maybe need to change Rmin and Rmax to achieve good results
Rmin = 20;
Rmax = 30;
% Rmin = 10;
% Rmax = 20;

%   Circle detection
[centersBright, radiiBright] = imfindcircles(img,[Rmin Rmax],'ObjectPolarity',...
    'bright', 'Sensitivity',0.8);

viscircles(centersBright, radiiBright,'EdgeColor','b');
hold on;

%   Sort by circle position
[circle_A,~] = sort(centersBright);
CircleCenter1_X = circle_A(1,1);
CircleCenter1_Y = circle_A(1,2);

plot(CircleCenter1_X, CircleCenter1_Y,'.');
hold on;

[circle_B,~] = sort(centersBright,'descend');
CircleCenter2_X = circle_B(1,1);
CircleCenter2_Y = circle_B(1,2);

plot(CircleCenter2_X, CircleCenter2_Y,'.');
hold on;

%   Calculate the distance between circles
CirclesNum = size(circle_A,1);  %   Number of detected circles
Radius = circle_A(1,1);         
RadiusDiff = 10.0;              %   The accuracy of different distance_X
count = 1;                      %   counter of different distance_X circles

for i=2:CirclesNum
    if abs( circle_A(i,1) - Radius ) > RadiusDiff
        count = count + 1;
        Radius = circle_A(i,1);
    end
end

%   Calculate the average of spacing
Ave_dist = abs( circle_A(1,1) - circle_A(CirclesNum,1) ) / ( count - 1 );


end




