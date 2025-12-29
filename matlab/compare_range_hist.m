function compare_range_hist(histo1, histo2, histo3)
%COMPARE_RANGE_HIST Compare 2 or 3 range-histogram CSVs and plot PDF/CDF overlays.
%
% Creates:
%   1) Probability (PDF-like) overlay (zero tails trimmed)
%   2) CDF overlay
% Saves both figures as PNGs next to histo1 by default.
%
% Usage:
%   compare_range_hist("/path/a.csv", "/path/b.csv");
%   compare_range_hist("/path/a.csv", "/path/b.csv", "/path/c.csv");
%   compare_range_hist();  % uses defaults below
%
% Expected CSV columns:
%   bin_left, bin_right, bin_center, count, probability, cumulative_probability

clearvars -except histo1 histo2 histo3; clc;

% ---- defaults ----
if nargin < 2 || isempty(histo1) || isempty(histo2)
    histo1 = "/home/jhuai/Downloads/mid360.csv";
    histo2 = "/home/jhuai/Downloads/airy.csv";
    histo3 = [];
end

paths = string({histo1, histo2});
if nargin >= 3 && ~isempty(histo3)
    paths(end+1) = string(histo3);
end

n = numel(paths);
assert(n == 2 || n == 3, "Need 2 or 3 csv paths.");

% Labels = basename without extension
labels = strings(1, n);
for i = 1:n
    [~, labels(i), ~] = fileparts(paths(i));
end

% ---- load tables ----
Ts = cell(1, n);
for i = 1:n
    Ts{i} = readtable(paths(i));
end

reqCols = ["bin_left","bin_right","bin_center","probability","cumulative_probability"];
for i = 1:n
    assert(all(ismember(reqCols, string(Ts{i}.Properties.VariableNames))), ...
        "CSV%d missing required columns. Found: %s", i, strjoin(Ts{i}.Properties.VariableNames, ", "));
end

% ---- percentiles ----
q = 0.90;
r90 = zeros(1, n);
for i = 1:n
    r90(i) = percentile_from_hist(Ts{i}, q);
    fprintf("[%s] r90 = %.3f m\n", labels(i), r90(i));
end

% ---- choose reference grid (use histo1) ----
x_ref = Ts{1}.bin_center;
p_ref = Ts{1}.probability;
c_ref = Ts{1}.cumulative_probability;

% Prepare arrays on reference grid
x = x_ref;
P = zeros(numel(x), n);
C = zeros(numel(x), n);

P(:,1) = p_ref;
C(:,1) = c_ref;

for i = 2:n
    xi = Ts{i}.bin_center;
    pi = Ts{i}.probability;
    ci = Ts{i}.cumulative_probability;

    sameBins = (numel(xi) == numel(x)) && all(abs(xi - x) < 1e-12);
    if sameBins
        P(:,i) = pi;
        C(:,i) = ci;
    else
        warning("Bin centers differ for '%s'; interpolating onto '%s' grid.", labels(i), labels(1));
        P(:,i) = interp1(xi, pi, x, "linear", 0);        % outside => 0 prob
        C(:,i) = interp1(xi, ci, x, "linear", "extrap"); % x-grid interpolation (not CDF->x)
    end
end

% ---- Trim zero tails for probability plot (where ALL curves are ~0) ----
epsProb = 0; % set to 1e-12 if you want tolerance
maskAnyNonZero = any(P > epsProb, 2);
if any(maskAnyNonZero)
    firstIdx = find(maskAnyNonZero, 1, "first");
    lastIdx  = find(maskAnyNonZero, 1, "last");
else
    firstIdx = 1; lastIdx = numel(x);
end

xt = x(firstIdx:lastIdx);
Pt = P(firstIdx:lastIdx, :);

% Use trimmed tail end to set x-limit (shared for both figures)
xLeft = 0;
xRight = x(lastIdx);
pad = 0.02 * (xRight - xLeft + eps);
xLim = [xLeft, xRight + pad];

% ---- Plot probability (trimmed) ----
fig1 = figure('Name','Range Histogram Probability (Trimmed)','Color','w');
hold on; grid on;
hProb = gobjects(1, n);
for i = 1:n
    hProb(i) = plot(xt, Pt(:,i), 'LineWidth', 1.5);
end
xlabel('Range (m)');
ylabel('Probability');
title('Range Distribution (Probability, trimmed zero tails)');
legend(hProb, labels, 'Location', 'best');
xlim(xLim);

% ---- Plot CDF (full) ----
fig2 = figure('Name','Range Histogram CDF','Color','w');
hold on; grid on;
hCdf = gobjects(1, n);
for i = 1:n
    hCdf(i) = plot(x, C(:,i), 'LineWidth', 1.5);
end
xlabel('Range (m)');
ylabel('Cumulative probability');

% Title with r90s
r90str = "";
for i = 1:n
    r90str = r90str + sprintf("%s=%.2fm", labels(i), r90(i));
    if i < n, r90str = r90str + ", "; end
end
title("Range Distribution (CDF)   r90: " + r90str);

legend(hCdf, labels, 'Location', 'best');
ylim([0 1]);
xlim(xLim);

% Mark r90 lines, hide from legend
for i = 1:n
    xl = xline(r90(i), '--', sprintf('%s r90=%.2f', labels(i), r90(i)));
    xl.HandleVisibility = 'off';
end

% ---- Save PNGs ----
outDir = fileparts(paths(1));
if outDir == "", outDir = "."; end

baseName = labels(1) + "_vs_" + labels(2);
if n == 3
    baseName = baseName + "_vs_" + labels(3);
end

png1 = fullfile(outDir, baseName + "_prob.png");
png2 = fullfile(outDir, baseName + "_cdf.png");

exportgraphics(fig1, png1, 'Resolution', 200);
exportgraphics(fig2, png2, 'Resolution', 200);

fprintf("Saved:\n  %s\n  %s\n", png1, png2);

end

% ---------------- local function ----------------
function rQ = percentile_from_hist(T, q)
%PERCENTILE_FROM_HIST Robust percentile from histogram CSV table.
% Handles non-unique CDF caused by zero-prob bins.

left = T.bin_left(:);
right = T.bin_right(:);
cdf = T.cumulative_probability(:);

q = max(0, min(1, q));

% Ensure monotonic non-decreasing
cdf = max(cdf, [0; cdf(1:end-1)]);

idx = find(cdf >= q, 1, "first");
if isempty(idx)
    rQ = right(end);
    return;
end
if idx == 1
    rQ = right(1);
    return;
end

c0 = cdf(idx-1);
c1 = cdf(idx);

% If flat segment, can't interpolate reliably
if c1 <= c0 + eps
    rQ = right(idx);
    return;
end

frac = (q - c0) / (c1 - c0);
frac = max(0, min(1, frac));
rQ = left(idx) + frac * (right(idx) - left(idx));
end
