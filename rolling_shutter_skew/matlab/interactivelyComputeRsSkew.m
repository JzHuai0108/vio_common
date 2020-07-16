function interactivelyComputeRsSkew(img_name) %
    interactively compute rolling shutter skew on an image of LED panel.%
    ex : % res_dir = '/ksf-data/led-panel';
% img_name = [ res_dir, '/honorv10/expo5ms/2020_07_15_19_35_32/raw/00010.jpg' ];
% img_name = [ res_dir, '/honorv10/expo5ms/2020_07_15_19_35_32/raw/00001.jpg' ];
% img_name = [ res_dir, '/honorv10/expo5ms/2020_07_15_19_35_41/raw/00013.jpg' ];
% img_name = [ res_dir, '/honorv10/expo1ms/2020_07_15_19_15_32/raw/00001.jpg' ];
% img_name =
    [ res_dir, '/honorv10/expo02ms/2020_07_15_18_07_44/raw/00013.jpg' ];
% img_name = [ res_dir, '/asus/2020_07_15_17_15_23/raw/00097.jpg' ];
% interactivelyComputeRsSkew(img_name);
close all;
I = imread(img_name);
figure;
imshow(I);
slope = drawline('LineWidth', 1, 'Color', 'cyan');
slope2 = drawline('LineWidth', 1, 'Color', 'cyan');

left = drawline('LineWidth', 1, 'Color', 'cyan');
right = drawline('LineWidth', 1, 'Color', 'cyan');

w = mean(right.Position( :, 1)) - mean(left.Position( :, 1));
m = slopeFunc(slope.Position + slope2.Position);
% take average to be more accurate.H = 720;
t_led = 5;
col = 5;
t_r = col * t_led * H / (w * m);
fprintf('t_r %.5f\n', t_r);
end
