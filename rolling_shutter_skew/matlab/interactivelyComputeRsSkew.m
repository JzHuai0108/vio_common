function interactivelyComputeRsSkew(t_led, led_gap_px)
% interactively compute rolling shutter skew on an image of LED panel.
% first draw two inclined lines for the rolling shutter effect at the
% bottom and at the top, then draw two vertical lines through the LED light
% centers, first left, second right.

% ex :
% res_dir = '/ksf-data/led-panel';
% img_name = [ res_dir, '/honorv10/expo5ms/2020_07_15_19_35_32/raw/00010.jpg' ];
% img_name = [ res_dir, '/honorv10/expo5ms/2020_07_15_19_35_32/raw/00001.jpg' ];
% img_name = [ res_dir, '/honorv10/expo5ms/2020_07_15_19_35_41/raw/00013.jpg' ];
% img_name = [ res_dir, '/honorv10/expo1ms/2020_07_15_19_15_32/raw/00001.jpg' ];
% img_name = [ res_dir, '/honorv10/expo02ms/2020_07_15_18_07_44/raw/00013.jpg' ];
% img_name = [ res_dir, '/asus/2020_07_15_17_15_23/raw/00097.jpg' ];
% interactivelyComputeRsSkew(img_name);
if nargin < 2 
    led_gap_px = 60;
end
if nargin < 1
    t_led = 1;
end
sloping1 = drawline('LineWidth', 1, 'Color', 'cyan');
sloping2 = drawline('LineWidth', 1, 'Color', 'cyan');

left = drawline('LineWidth', 1, 'Color', 'cyan');
right = drawline('LineWidth', 1, 'Color', 'cyan');

addlistener(sloping1,'ROIMoved',@drawLineEvents);
addlistener(sloping2,'ROIMoved',@drawLineEvents);
addlistener(left,'ROIMoved',@drawLineEvents);
addlistener(right,'ROIMoved',@drawLineEvents);

drawLineEvents(0, 0);

    function drawLineEvents(src, evt)
        % rolling shutter skew computation.
        w = mean(right.Position(:, 1)) - mean(left.Position(:, 1));
        m = slopeFunc(sloping1.Position + sloping2.Position); % take average to be more accurate.
        H = 720;
        col = round(w / led_gap_px);
        global t_r;
        t_r = col * t_led * H / (w * m);
        
        str = sprintf(['left x %.1f, right x %.1f\n', ...
            'led-step(px) %d, led col steps %d\n', ...
            'sloping1 (%d, %d; %d, %d)\n', ...
            'sloping2 (%d, %d; %d, %d)\n', ...
            'H %d, t-led %.4f ms, t-r %.4f ms\n'], ...
            mean(left.Position(:, 1)), mean(right.Position(:, 1)), ...
            led_gap_px, col, ...
            sloping1.Position(1,1), sloping1.Position(1,2), ...
            sloping1.Position(2,1), sloping1.Position(2,2), ...
            sloping2.Position(1,1), sloping2.Position(1,2), ...
            sloping2.Position(2,1), sloping2.Position(2,2), ...
            H, t_led, t_r);
        dim = [.65 .6 .25 .25];
        t = annotation('textbox', dim, 'String',str, 'FitBoxToText','on');
        t.BackgroundColor = 'w';
        drawnow;
    end
end
