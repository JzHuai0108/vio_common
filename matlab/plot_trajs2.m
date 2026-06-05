function plot_trajs2(folder, basename)
files = dir([folder, '/**/*', basename, '*.txt']);

close all;
fprintf('Found %d files\n', numel(files));
for i=1:length(files)
f = fullfile(files(i).folder, files(i).name);
[p, n, e] = fileparts(f);
[q, m, e] = fileparts(p);
[r, s, e] = fileparts(q);
[u, v, e] = fileparts(r);
ln = [v, '/', s, '/', m];
d = readmatrix(f);
if isempty(d)
    fprintf('No data found in %s\n', f);
    continue;
end

df = diff(d(:, 1));
fprintf('%d, %s, max df %.5f, min df %.5f, avg df %.5f\n', i, [ln, '/', n], max(df), min(df), mean(df));
figure
[mx, id] = max(df);
if mx > 50
    s = d(1:id, :);
    e = d(id+1:end, :);
    plot3(s(:, 2), s(:, 3), s(:, 4), 'r'); hold on;
    plot3(e(:, 2), e(:, 3), e(:, 4), 'b'); hold on;
else
    plot3(d(:, 2), d(:, 3), d(:, 4), 'k'); hold on;
end
plot3(d(1, 2), d(1, 3), d(1, 4), 'ro'); 
plot3(d(end, 2), d(end, 3), d(end, 4), 'bs'); grid on;

% axis equal;
xlabel('x');
ylabel('y');
zlabel('z');
title(ln, 'Interpreter','none');

if size(d, 2) >= 10
figure;
s = d(1, 1);
d(:, 1) = d(:, 1) - s;
plot(d(:, 1), d(:, 5), 'r'); hold on;
plot(d(:, 1), d(:, 6), 'g');
plot(d(:, 1), d(:, 7), 'b');
legend('qx', 'qy', 'qz');
xlabel('t');
title(ln, 'Interpreter','none');
end
end

