function [CircleCenter1_X, CircleCenter1_Y, CircleCenter2_X, CircleCenter2_Y] = Imfindcircles( imageName )
%Imfindcircles ���ٵ���ȡ�����Բ��
%���������
%   imageName������ͼ��
%���������
%   CircleCenter1_X�����Ϸ�Բ��x����
%   CircleCenter1_Y�����Ϸ�Բ��y����
%   CircleCenter2_X�����·�Բ��x����
%   CircleCenter2_Y�����·�Բ��y����
%���Ժ�����
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




