function plot_quality_check_projections()

lio_dir = 'F:\jhuai\lidar-phone-construction\labelcloud\pointclouds';
corridor_lio_pcd = fullfile(lio_dir, '2024_12_05_16_11_23\original_aligned.pcd');
addpath('F:\jhuai\tools\export_fig');

close all;
figure;
lio = pcread(corridor_lio_pcd);
pcshow(lio);
xlabel('X (m)');
ylabel('Y (m)');
zlabel('Z (m)');

xyz_limits = [0.5, 9.0; -3.5, -1.8; -7.5, -3.0];
lio1 = mask_pc_by_cube(lio, xyz_limits);
dy = xyz_limits(2, 2) - xyz_limits(2, 1);
plot_xz(lio1, dy, 'stairway_lower_xz.png');

xyz_limits = [0.5, 8.5; -1, -0.1; -4.0, -1.0];
lio1 = mask_pc_by_cube(lio, xyz_limits);
dy = xyz_limits(2, 2) - xyz_limits(2, 1);
plot_xz(lio1, dy, 'stairway_upper_xz.png');

xyz_limits = [-5, -3.6; -35, 5.0; -2, -1];
lio1 = mask_pc_by_cube(lio, xyz_limits);
dx = xyz_limits(1, 2) - xyz_limits(1, 1);
plot_yz(lio1, dx, 'floor_yz.png');

xyz_limits = [-4, 1.0; -33, -5; -0.68, 1.12];
lio1 = mask_pc_by_cube(lio, xyz_limits);
figure
pcshow(lio1);
xlabel('x');
ylabel('y');

dz = xyz_limits(3, 2) - xyz_limits(3, 1);
plot_xy(lio1, dz, 'columns_xy.png');

xyz_limits = [-12, 1.0; -41.7, -30; -0.2, 1.6];
lio1 = mask_pc_by_cube(lio, xyz_limits);
figure
pcshow(lio1);
xlabel('x');
ylabel('y');

dz = xyz_limits(3, 2) - xyz_limits(3, 1);
plot_xy(lio1, dz, 'section_xy.png');
end


function plot_xz(pc, dy, figname)
% PLOT_XZ  Scatter‐plot projection of a pointCloud onto the X–Z plane.
%
%   plot_xz(pc) displays the points (X,Z) with the same RGB color as in pc.
pts = pc.Location;                
X   = pts(:,1);
Z   = pts(:,3);

% Determine color from intensity
if isprop(pc, 'Intensity') && ~isempty(pc.Intensity)
    I = double(pc.Intensity);
    % Normalize to [0,1]
    Imin = min(I);
    Imax = max(I);
    if Imax > Imin
        C = (I - Imin) ./ (Imax - Imin);
    else
        C = zeros(size(I));
    end
else
    % Fallback to gray
    C = zeros(size(X));
end

% 2D scatter
figure;
scatter(X, Z, 2, C, '.');
axis tight;
grid on;
xlabel('X (m)'); ylabel('Z (m)');
% Add 5% margin on X axis
ax = gca;
xl = ax.XLim;                % current [xmin xmax]
dx = xl(2) - xl(1);          % total range
margin = 0.03 * dx;          % 5% margin
ax.XLim = [xl(1) - margin, xl(2) + margin];

title(sprintf('\\DeltaY = %.2f m', dy), 'Interpreter', 'tex');
export_fig(figname, '-png', '-transparent', '-r400');
end

function plot_yz(pc, deltax, figname)
% PLOT_YZ  Scatter‐plot projection of a pointCloud onto the Y–Z plane.
%
%   plot_yz(pc) displays the points (Y,Z) with the same RGB color as in pc.
pts = pc.Location;
Y   = pts(:,2);
Z   = pts(:,3);

% Determine color from intensity
if isprop(pc, 'Intensity') && ~isempty(pc.Intensity)
    I = double(pc.Intensity);
    % Normalize to [0,1]
    Imin = min(I);
    Imax = max(I);
    if Imax > Imin
        C = (I - Imin) ./ (Imax - Imin);
    else
        C = zeros(size(I));
    end
else
    % Fallback to gray
    C = zeros(size(X));
end

figure;
scatter(Y, Z, 2, C, '.');
axis tight;
grid on;
xlabel('Y (m)'); ylabel('Z (m)');

% Add 5% margin on X axis
ax = gca;
xl = ax.XLim;                % current [xmin xmax]
dx = xl(2) - xl(1);          % total range
margin = 0.03 * dx;          % 5% margin
ax.XLim = [xl(1) - margin, xl(2) + margin];

title(sprintf('\\DeltaX = %.2f m', deltax), 'Interpreter', 'tex');
export_fig(figname, '-png', '-transparent', '-r400');
end

function plot_xy(pc, dz, figname)
% PLOT_XY  Scatter‐plot projection of a pointCloud onto the X–Y plane.
%
%   plot_xy(pc) displays the points (X,Y) with the same RGB color as in pc.
pts = pc.Location;
X   = pts(:,1);
Y   = pts(:,2);

% Determine color from intensity
if isprop(pc, 'Intensity') && ~isempty(pc.Intensity)
    I = double(pc.Intensity);
    % Normalize to [0,1]
    Imin = min(I);
    Imax = max(I);
    if Imax > Imin
        C = (I - Imin) ./ (Imax - Imin);
    else
        C = zeros(size(I));
    end
else
    % Fallback to gray
    C = zeros(size(X));
end

figure;
scatter(X, Y, 2, C, '.');
axis tight;
grid on;
xlabel('X (m)'); ylabel('Y (m)');
% Add margin on X axis
ax = gca;
xl = ax.XLim;                % current [xmin xmax]
dx = xl(2) - xl(1);          % total range
margin = 0.03 * dx;          % 5% margin
ax.XLim = [xl(1) - margin, xl(2) + margin];

ax = gca;
yl = ax.YLim;                % current [xmin xmax]
dy = yl(2) - yl(1);          % total range
margin = 0.03 * dy;          % 5% margin
ax.YLim = [yl(1) - margin, yl(2) + margin];

title(sprintf('\\DeltaZ = %.2f m', dz), 'Interpreter', 'tex');
export_fig(figname, '-png', '-transparent', '-r400');
end
