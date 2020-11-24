function interactivelyComputeRsSkew(t_led, led_gap_px, H, homography)
% interactively compute rolling shutter skew on an image of LED panel.
% first draw two inclined lines for the rolling shutter effect at the
% bottom and at the top, then draw two vertical lines through the LED light
% centers, first left, second right.

if nargin < 4
    homography = eye(3);
    useHomography = 0;
else
    if isequal(homography, eye(3))    
        useHomography = 0;
    else
        useHomography = 1;
    end
end
if nargin < 3
    H = 720;
end
if nargin < 2 
    led_gap_px = 60;
end
if nargin < 1
    t_led = 1; % millisec
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
       
        col = round(w / led_gap_px);
        global t_r;
        t_r = col * t_led * H / (w * m);
        global t_r_h;
        if (useHomography)
            d1 = lineDelayByHomography(sloping1.Position, homography, t_led);
            d2 = lineDelayByHomography(sloping2.Position, homography, t_led);
            t_r_h = (d1 + d2) * 0.5 * H;
        else
            t_r_h = t_r;
        end
        str = sprintf(['left x %.1f, right x %.1f\n', ...
            'led-step(px) %.1f, led col steps %d led-step-est %.1f\n', ...
            'sloping1 (%.1f, %.1f; %1.f, %1.f)\n', ...
            'sloping2 (%.1f, %.1f; %.1f, %1.f)\n', ...
            'H %d, t-led %.4f ms, t-r %.4f ms t-r-h %.4f ms\n'], ...
            mean(left.Position(:, 1)), mean(right.Position(:, 1)), ...
            led_gap_px, col, w / col, ...
            sloping1.Position(1, 1), sloping1.Position(1, 2), ... 
            sloping1.Position(2, 1), sloping1.Position(2, 2), ... 
            sloping2.Position(1, 1), sloping2.Position(1, 2), ...
            sloping2.Position(2, 1), sloping2.Position(2, 2), ...
            H, t_led, t_r, t_r_h);
        dim = [.65 .6 .25 .25];
        t = annotation('textbox', dim, 'String',str, 'FitBoxToText','on');
        t.BackgroundColor = 'w';
        drawnow;
    end
    function d = lineDelayByHomography(slopeSegment, homography, durationLed)       
        % slopeSegment (x1, y1; x2, y2). x is from left to right for 
        % columns, y is from top to bottom for rows of the image.
        segment = homography * [slopeSegment, [1.0; 1.0]]';
        segment(:, 1) = segment(:, 1) / segment(3, 1);
        segment(:, 2) = segment(:, 2) / segment(3, 2);
        d = durationLed * (segment(1, 2) - segment(1, 1)) / ...
            (slopeSegment(2, 2) - slopeSegment(1, 2));
    end
end
