function [CircleCenter1_X, CircleCenter1_Y, CircleCenter2_X, CircleCenter2_Y] = Imfindcircles( imageName )
%Imfindcircles 快速的提取亮点的圆心
%输入参数：
%   imageName：输入图像
%输出参数：
%   CircleCenter1_X：左上方圆心x坐标
%   CircleCenter1_Y：左上方圆心y坐标
%   CircleCenter2_X：右下方圆心x坐标
%   CircleCenter2_Y：右下方圆心y坐标
%测试函数：
%   [x1,y1,x2,y2] = Imfindcircles('00001.jpg')

img = imread(imageName);

imshow(img)

Rmin = 20;
Rmax = 30;
[centersBright, radiiBright] = imfindcircles(img,[Rmin Rmax],'ObjectPolarity','bright',...
    'Sensitivity',0.8);

viscircles(centersBright, radiiBright,'EdgeColor','b');
hold on;

[circle_A,index_A] = sort(centersBright);
CircleCenter1_X = circle_A(1,1);
CircleCenter1_Y = circle_A(1,2);

plot(CircleCenter1_X, CircleCenter1_Y,'x');
hold on;


[circle_B,index_B] = sort(centersBright,'descend');
CircleCenter2_X = circle_B(1,1);
CircleCenter2_Y = circle_B(1,2);

plot(CircleCenter2_X, CircleCenter2_Y,'x');
hold on;


end




