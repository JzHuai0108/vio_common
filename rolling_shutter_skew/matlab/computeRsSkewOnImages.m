function computeRsSkewOnImages(imageDir, outputDir, t_led, led_gap_px, H)
if nargin < 5
    H = 720;
end
if nargin < 4
    led_gap_px = 60; % number of pixels between two LED columns' centerlines
end
if nargin < 3
    t_led = 1; % the time a LED light is lit in millisecs.
end

close all;

% gather images in a dir
imageStructs = dir(fullfile(imageDir,'*.jpg'));
imageList = cell(1, length(imageStructs));
for i = 1: length(imageStructs)
    imageList{i} = fullfile(imageDir, imageStructs(i).name);
end

% show the window
cell_list = cell(1, 5);
cell_list{1, 1} = {'Previous', 'getPreviousFrame'};
cell_list{1, 2} = {'Refresh', 'getCurrentFrame'};
cell_list{1, 3} = {'Next', 'getNextFrame'};
cell_list{1, 4} = {'Draw lines', 'interactivelyComputeRsSkewWrap'};
cell_list{1, 5} = {'Save and Next', 'saveCurrentAndGetNext'};

imageHeight = 720;
imageWidth = 1280;
border = 50;
gap_x = 20;
gap_y = 0;

homographyTxt = [imageDir, '/homography.txt'];
if isfile(homographyTxt)
    homography = readmatrix(homographyTxt, 'NumHeaderLines', 1);
    disp(['Load homography from ', homographyTxt]);
    disp(homography);
else
    homography = eye(3);
end
global currentIndex;
currentIndex = 1;

global numDrawnImages;
numDrawnImages = 0;

global tr_estimate; % modified by interactivelyComputeRsSkew, and saved by saveCurrentAndGetNext

if ~exist('fig_number')
    fig_number = 1;
end
if ~exist('title_figure')
    title_figure = '';
end
if ~exist('x_size')
    x_size = 120;
end
if ~exist('y_size')
    y_size = 20;
end
if ~exist('gap_x')
    gap_x = 0;
end
if ~exist('font_name')
    font_name = 'clean';
end
if ~exist('font_size')
    font_size = 12;
end

figure(fig_number); clf;
pos = get(fig_number,'Position');

[n_row,n_col] = size(cell_list);

fig_size_x = max(imageWidth + border, x_size*n_col+(n_col + 1)*gap_x);
fig_size_y = imageHeight + border + y_size*n_row+(n_row+1)*gap_y;

set(fig_number,'Units','points', ...
    'BackingStore','off', ...
    'Color',[0.8 0.8 0.8], ...
    'MenuBar','none', ...
    'Resize','off', ...
    'Name',title_figure, ...
    'Position',[pos(1) pos(2) fig_size_x fig_size_y], ...
    'NumberTitle','off'); %,'WindowButtonMotionFcn',['figure(' num2str(fig_number) ');']);

h_mat = zeros(n_row,n_col);

posx = zeros(n_row,n_col);
posy = zeros(n_row,n_col);

for i=n_row:-1:1
    for j = n_col:-1:1
        posx(i,j) = gap_x + border + (j-1)*(x_size+gap_x);
        posy(i,j) = fig_size_y - i*(gap_y+y_size) - border;
    end
end

for i=n_row:-1:1
    for j = n_col:-1:1
        if ~isempty(cell_list{i,j})
            if ~isempty(cell_list{i,j}{1}) && ~isempty(cell_list{i,j}{2})
                switch j
                    case {1, 2, 3}
                        h_mat(i,j) = uicontrol('Parent',fig_number, ...
                            'Units','points', ...
                            'Callback',{cell_list{i,j}{2}, imageList}, ...
                            'ListboxTop', 0, ...
                            'Position',[posx(i,j)  posy(i,j)  x_size   y_size], ...
                            'String',cell_list{i,j}{1}, ...
                            'fontsize',font_size,...
                            'fontname',font_name,...
                            'Tag','Pushbutton1');
                    case 4 % draw
                        h_mat(i,j) = uicontrol('Parent',fig_number, ...
                            'Units','points', ...
                            'Callback',{cell_list{i,j}{2}, t_led, led_gap_px, H, homography}, ...
                            'ListboxTop', 0, ...
                            'Position',[posx(i,j)  posy(i,j)  x_size   y_size], ...
                            'String',cell_list{i,j}{1}, ...
                            'fontsize',font_size,...
                            'fontname',font_name,...
                            'Tag','Pushbutton1');
                    case 5 % save
                        h_mat(i,j) = uicontrol('Parent',fig_number, ...
                            'Units','points', ...
                            'Callback',{cell_list{i,j}{2}, imageList, outputDir}, ...
                            'ListboxTop', 0, ...
                            'Position',[posx(i,j)  posy(i,j)  x_size   y_size], ...
                            'String',cell_list{i,j}{1}, ...
                            'fontsize',font_size,...
                            'fontname',font_name,...
                            'Tag','Pushbutton1');
                    otherwise
                        fprintf('Unsupported command %s\n', cell_list{i,j}{2});
                end
            end
        end
    end
end

showFrame(imageList, currentIndex);

