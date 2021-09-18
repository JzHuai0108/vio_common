path = '/usr/local/MATLAB/R2019b/toolbox/images/imdata/moon.tif';
path = '/led-panel-rscalib/honorv10/expo05ms/2020_07_15_19_21_19/raw/00010.jpg';
path = '/ueye_led_panel/rs1/00234.jpg';

I=imread(path);
[x,y,z]=size(I);
X=1:x;
Y=1:y;
[xx,yy]=meshgrid(Y,X);
i=im2double(I);
size(i)
close all;

figure;mesh(xx,yy,i(:, :, 1));
colorbar
figure;imshow(i(:, :, 1));