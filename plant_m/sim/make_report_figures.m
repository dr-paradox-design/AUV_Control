function make_report_figures(with_current)
% MAKE_REPORT_FIGURES  Simple figures for the report, written to reports/figures/.
% make_report_figures        still-water (pool) set: no ocean current
% make_report_figures(true)  also the current box, velocity triangle and current profile
%   block_diagram     system overview
%   thruster_layout   body axes and thruster layout, top view (ASSUMED placeholder geometry)
%   velocity_triangle water-relative velocity, current and ground velocity
%   fig4_current_profile   Ekman-type current profile (ASSUMED parameters)
%   surge_drag        thrust vs speed with the Wu (2018) coefficients
%   pitch_stability   open-loop pitch peak vs forward speed (Wu parameters)
if nargin < 1, with_current = false; end
here = fileparts(mfilename('fullpath')); root = fileparts(here);
addpath(fullfile(root, 'config')); setup_plant_paths();
out = fullfile(fileparts(root), 'reports', 'figures');
if ~exist(out, 'dir'), mkdir(out); end
set(0, 'DefaultAxesFontSize', 13, 'DefaultTextFontSize', 13, 'DefaultLineLineWidth', 1.8);
blue = [0.12 0.35 0.70]; red = [0.78 0.16 0.16]; green = [0.10 0.55 0.25]; grey = [0.35 0.35 0.35];

%% 1. block diagram
f = newfig(1300, 520); ax = axes('Position', [0 0 1 1]); hold on; axis off; xlim([0 26]); ylim([0 10.4]);
bx(ax, 0.6, 4.2, 4.0, 2.2, {'Controller'}, [0.95 0.95 0.95]);
bx(ax, 5.8, 4.2, 4.0, 2.2, {'Thrust allocation', '8 thrusters'}, [0.95 0.95 0.95]);
bx(ax, 11.0, 4.2, 4.4, 2.2, {'6-DOF plant', '(vehicle dynamics)'}, [0.83 0.90 0.98]);
bx(ax, 16.6, 4.2, 4.0, 2.2, {'Sensors'}, [0.95 0.95 0.95]);
bx(ax, 21.6, 4.2, 4.0, 2.2, {'State estimator'}, [0.86 0.95 0.86]);
if with_current, bx(ax, 11.0, 7.8, 4.4, 1.8, {'Ocean current  v_c'}, [0.99 0.93 0.82]); end
ar(ax, [4.6 5.8], [5.3 5.3]); text(5.2, 5.75, '\tau', 'HorizontalAlignment', 'center');
ar(ax, [9.8 11.0], [5.3 5.3]); text(10.4, 5.75, 'u', 'HorizontalAlignment', 'center');
ar(ax, [15.4 16.6], [5.3 5.3]); text(16.0, 5.75, '\eta, \nu', 'HorizontalAlignment', 'center');
ar(ax, [20.6 21.6], [5.3 5.3]);
if with_current, ar(ax, [13.2 13.2], [7.8 6.4]); text(13.5, 7.1, '\nu_r = \nu - \nu_c'); end
ar(ax, [23.6 23.6 2.6 2.6], [4.2 2.0 2.0 4.2]); text(13.1, 1.45, 'estimated state', 'HorizontalAlignment', 'center');
if ~with_current, ylim([0.8 7.6]); set(f, 'Position', [60 60 1300 340]); end   % crop the empty band
save_fig(f, out, 'block_diagram');

%% 2. body frame and thrusters in 3D
p = params_plant(); r = p.thr.r; e = p.thr.e;
% top view: forward (x_b) is up, starboard (y_b) is right; drawn in units of 5 cm
f = newfig(860, 800); ax = axes('Position', [0.03 0.03 0.94 0.90]); hold on; axis equal; axis off;
k = 20; th = linspace(0, 2 * pi, 200);
fill(k * 0.30 * sin(th), k * 0.26 * cos(th), [0.88 0.92 0.96], 'EdgeColor', [0.5 0.55 0.6], 'LineWidth', 1.4);
ar(ax, [0 0], [0 8.4], red, 2.6); text(0.4, 8.8, 'x_b (forward)', 'Color', red);
ar(ax, [0 9.0], [0 0], green, 2.6); text(9.3, 0, 'y_b (starboard)', 'Color', green);
plot(0, 0, 'o', 'MarkerSize', 15, 'Color', blue, 'LineWidth', 2); plot(0, 0, 'x', 'MarkerSize', 10, 'Color', blue, 'LineWidth', 2);
text(-0.6, -1.0, 'z_b (down, into the page)', 'Color', blue, 'HorizontalAlignment', 'right');
for i = 1:8
    py = k * r(2, i); px = k * r(1, i);
    if i <= 4
        ar(ax, [py, py + 2.4 * e(2, i)], [px, px + 2.4 * e(1, i)], grey, 2.4);
        plot(py, px, 'ko', 'MarkerFaceColor', 'k', 'MarkerSize', 8);
        text(py, px - 0.85, sprintf('T%d', i), 'HorizontalAlignment', 'center', 'FontWeight', 'bold');
    else
        plot(py, px, 'o', 'MarkerSize', 15, 'Color', grey, 'LineWidth', 2.2); plot(py, px, 'x', 'MarkerSize', 10, 'Color', grey, 'LineWidth', 2.2);
        text(py + 1.3 * sign(py), px, sprintf('T%d', i), 'HorizontalAlignment', 'center', 'FontWeight', 'bold');
    end
end
text(0, -7.2, {'arrow: horizontal thruster and its thrust direction', 'circle with cross: vertical thruster (thrust along z_b)'}, 'HorizontalAlignment', 'center', 'FontSize', 12);
xlim([-9.5 13]); ylim([-8.5 10]);
title('Thruster layout, top view (schematic, assumed geometry)');
save_fig(f, out, 'thruster_layout');

if with_current
%% 3. velocity triangle
f = newfig(760, 620); ax = axes('Position', [0.02 0.02 0.96 0.96]); hold on; axis equal; axis off;
psi = 25 * pi / 180; a = 6 * [sin(psi) cos(psi)]; c = 2.6 * [sin(100 * pi / 180) cos(100 * pi / 180)]; b = a + c;
plot([0 0], [0 6.6], '--', 'Color', [0.6 0.6 0.6], 'LineWidth', 1); text(0.1, 6.8, 'North');
plot([0 6.2], [0 0], '--', 'Color', [0.6 0.6 0.6], 'LineWidth', 1); text(6.3, 0, 'East');
ar(ax, [0 a(1)], [0 a(2)], blue, 2.6); ar(ax, [a(1) b(1)], [a(2) b(2)], red, 2.6); ar(ax, [0 b(1)], [0 b(2)], green, 2.6);
text(-0.3, 3.2, {'velocity through water', 'R \nu_r'}, 'Color', blue, 'HorizontalAlignment', 'right');
text(a(1) + c(1) / 2 + 0.1, a(2) + c(2) / 2 + 0.55, {'current  v_c'}, 'Color', red);
text(b(1) / 2 + 0.35, b(2) / 2 - 0.5, {'velocity over ground', 'v = R \nu_r + v_c'}, 'Color', green);
xlim([-4.6 8]); ylim([-0.8 7.2]);
save_fig(f, out, 'velocity_triangle');

%% 4. current profile
env = struct('preset', 'ekman', 'Winf', 0.05 + 0.02i, 'W0', 0.25 * exp(1i * 0.3), 'kd', 0.15, 'kv', 0.15);
z = linspace(0, 30, 301); V = zeros(3, numel(z));
for k = 1:numel(z), V(:, k) = current_profile(z(k), env); end
f = newfig(1150, 480);
subplot(1, 2, 1); hold on; grid on; box on;
plot(V(1, :), z, 'Color', blue); plot(V(2, :), z, 'Color', red); plot(hypot(V(1, :), V(2, :)), z, 'k--');
set(gca, 'YDir', 'reverse'); xlabel('current [m/s]'); ylabel('depth z [m]');
legend({'north', 'east', 'speed'}, 'Location', 'southeast'); title('Current against depth');
subplot(1, 2, 2); hold on; grid on; box on; axis equal;
plot(V(2, :), V(1, :), 'Color', blue); idx = [1 34 67 101 201];
plot(V(2, idx), V(1, idx), 'ko', 'MarkerFaceColor', 'k');
for k = idx, text(V(2, k) + 0.006, V(1, k), sprintf('%g m', z(k))); end
xlabel('east [m/s]'); ylabel('north [m/s]'); title('Direction turns with depth');
save_fig(f, out, 'current_profile');
end

%% NED and body frames (sketch, oblique view)
f = newfig(1200, 600); ax = axes('Position', [0 0 1 1]); hold on; axis off; xlim([0 24]); ylim([0 12]);
O = [6.5 8.2]; Bo = [15.5 6.6]; c20 = cos(20 * pi / 180); s20 = sin(20 * pi / 180);
th = linspace(0, 2 * pi, 200); ex = 3.6 * cos(th); ey = 1.2 * sin(th);
fill(Bo(1) + c20 * ex - s20 * ey, Bo(2) + s20 * ex + c20 * ey, [0.90 0.93 0.96], 'EdgeColor', [0.6 0.65 0.7], 'LineWidth', 1.2);
ar(ax, [O(1) Bo(1)], [O(2) Bo(2)], [0.55 0.1 0.1], 2.0);
text(10.2, 6.75, 'p  (position, in \{n\})', 'Color', [0.55 0.1 0.1], 'Rotation', -10);
ar(ax, [O(1) O(1) + 4.4], [O(2) O(2)], 'k', 2.4); text(O(1) + 4.6, O(2), 'x_n  (North)');
ar(ax, [O(1) O(1) - 2.5], [O(2) O(2) - 2.2], 'k', 2.4); text(O(1) - 2.8, O(2) - 2.5, 'y_n  (East)', 'HorizontalAlignment', 'right');
ar(ax, [O(1) O(1)], [O(2) O(2) - 4.4], 'k', 2.4); text(O(1) + 0.3, O(2) - 4.5, 'z_n  (Down)');
text(O(1) - 0.3, O(2) + 0.7, '\{n\}', 'HorizontalAlignment', 'right', 'FontSize', 16);
ar(ax, [Bo(1) Bo(1) + 4.6 * c20], [Bo(2) Bo(2) + 4.6 * s20], blue, 2.4); text(Bo(1) + 4.6 * c20 + 0.2, Bo(2) + 4.6 * s20 + 0.1, 'x_b  (forward)', 'Color', blue);
ar(ax, [Bo(1) Bo(1) - 2.5], [Bo(2) Bo(2) - 2.2], blue, 2.4); text(Bo(1) - 2.8, Bo(2) - 2.5, 'y_b  (starboard)', 'Color', blue, 'HorizontalAlignment', 'right');
ar(ax, [Bo(1) Bo(1) + 4.2 * s20], [Bo(2) Bo(2) - 4.2 * c20], blue, 2.4); text(Bo(1) + 4.2 * s20 + 0.3, Bo(2) - 4.2 * c20, 'z_b  (down)', 'Color', blue);
text(Bo(1) + 0.2, Bo(2) + 0.9, '\{b\}', 'Color', blue, 'FontSize', 16);
text(15.4, 10.6, 'attitude  R(q):  \{b\} \rightarrow \{n\}', 'HorizontalAlignment', 'center', 'FontSize', 15);
save_fig(f, out, 'frames_ned_body');

%% surge step response in still water (Wu parameters, thruster lag 0.15 s)
p = params_plant();
p.m = 11.5; p.rg = [0; 0; 0.02]; p.rb = [0; 0; 0]; p.Ig = 0.16 * eye(3); p.A = [5.5 12.7 14.57 0.12 0.12 0.12];
p.Dl = [4.03 6.22 5.18 0.07 0.07 0.07]; p.Dq = [18.18 21.66 36.99 1.55 1.55 1.55];
p = update_params(p); p.B = p.W;
Xs = [2 4 6]; cols = {green, blue, red};
f = newfig(900, 560); hold on; grid on; box on; hl = zeros(1, 3);
for i = 1:3
    X = Xs(i);
    o = simulate([0; 0; 5; 1; 0; 0; 0; zeros(6, 1)], @(t) [X * (1 - exp(-t / p.Tt)); 0; 0; 0; 0; 0], p, 0.002, 10);
    uss = (-p.Dl(1) + sqrt(p.Dl(1)^2 + 4 * p.Dq(1) * X)) / (2 * p.Dq(1));
    hl(i) = plot(o.t, o.X(:, 8), 'Color', cols{i});
    plot([0 10], [uss uss], ':', 'Color', cols{i}, 'LineWidth', 1.2);
    thmax = 0; for k = 1:50:numel(o.t), e2 = quat_to_euler(o.X(k, 4:7)'); thmax = max(thmax, abs(e2(2))); end
    fprintf('surge step %g N: final %.4f m/s, steady-state formula %.4f m/s, max pitch %.2f deg\n', X, o.X(end, 8), uss, thmax * 180 / pi);
end
xlabel('time [s]'); ylabel('surge speed u [m/s]'); xlim([0 10]); ylim([0 0.55]);
legend(hl, {'2 N', '4 N', '6 N'}, 'Location', 'southeast');
title('Surge step response in still water (dotted: steady state)');
save_fig(f, out, 'surge_step_response');

%% 5. surge drag: thrust needed against speed
Xu = 4.03; Xuu = 18.18; T = 4 * 40 * cos(pi / 4); u = linspace(0, 2.7, 300);
kimp = (T - Xu * 1.5) / 1.5^2;
f = newfig(900, 560); hold on; grid on; box on;
plot(u, Xu * u + Xuu * u.^2, 'Color', blue);
plot(u, Xuu * u.^2, '--', 'Color', blue);
plot(u, Xu * u + kimp * u.^2, 'Color', red);
plot([0 2.7], [T T], 'k:', 'LineWidth', 1.4);
u1 = (-Xu + sqrt(Xu^2 + 4 * Xuu * T)) / (2 * Xuu); u2 = sqrt(T / Xuu);
plot([u1 u2 1.5], [T T T], 'ko', 'MarkerFaceColor', 'k');
text(u1 - 0.05, T + 7, sprintf('%.2f', u1), 'HorizontalAlignment', 'right'); text(u2 + 0.03, T + 7, sprintf('%.2f', u2)); text(1.5, T + 7, '1.50', 'HorizontalAlignment', 'center');
text(0.05, T + 6, sprintf('available surge thrust %.0f N', T));
xlabel('forward speed [m/s]'); ylabel('thrust needed [N]'); ylim([0 150]); xlim([0 2.7]);
legend({'Wu: linear + quadratic drag', 'Wu: quadratic drag only', sprintf('drag implied by 1.5 m/s (X_{u|u|} = -%.1f)', kimp)}, 'Location', 'southeast');
title('Top speed depends strongly on the drag coefficient');
save_fig(f, out, 'surge_drag');

%% 6. open-loop pitch peak against forward speed
p = params_plant();
p.m = 11.5; p.rg = [0; 0; 0.02]; p.rb = [0; 0; 0]; p.Ig = 0.16 * eye(3); p.A = [5.5 12.7 14.57 0.12 0.12 0.12];
p.Dl = [4.03 6.22 5.18 0.07 0.07 0.07]; p.Dq = [18.18 21.66 36.99 1.55 1.55 1.55];
p = update_params(p); p.B = p.W;
Us = 0.30:0.05:1.00; pk = zeros(size(Us));
for i = 1:numel(Us)
    U = Us(i); tx = p.Dl(1) * U + p.Dq(1) * U^2;
    o = simulate([0; 0; 5; euler_to_quat(0, 2 * pi / 180, 0); U; 0; 0; 0; 0; 0], @(t) [tx; 0; 0; 0; 0; 0], p, 0.01, 90);
    th = zeros(size(o.t)); for k = 1:numel(th), ee = quat_to_euler(o.X(k, 4:7)'); th(k) = ee(2); end
    pk(i) = max(abs(th)) * 180 / pi;
end
Uc = sqrt(2 * p.W * 0.02 / (p.A(3) - p.A(1))); Us0 = sqrt(p.W * 0.02 / (p.A(3) - p.A(1)));
f = newfig(900, 560); hold on; grid on; box on;
plot(Us, pk, '-o', 'Color', blue, 'MarkerFaceColor', blue);
plot([Us0 Us0], [0 95], '--', 'Color', green); plot([Uc Uc], [0 95], '--', 'Color', red);
text(Us0 - 0.01, 70, sprintf('small-angle limit\n%.2f m/s', Us0), 'Color', green, 'HorizontalAlignment', 'right');
text(Uc + 0.01, 30, sprintf('Munk moment exceeds\nmaximum righting moment\n%.2f m/s', Uc), 'Color', red);
xlabel('forward speed [m/s]'); ylabel('peak pitch angle [deg]'); ylim([0 95]); xlim([0.3 1.0]);
title('Open-loop pitch grows with forward speed (2 deg initial pitch)');
save_fig(f, out, 'pitch_stability');
fid = fopen(fullfile(out, 'pitch_stability_data.txt'), 'w'); fprintf(fid, 'U [m/s]  peak pitch [deg]\n'); fprintf(fid, '%.2f  %.1f\n', [Us; pk]); fclose(fid);
fprintf('wrote figures to %s\n', out);
end


function f = newfig(w, h)
f = figure('Visible', 'off', 'Color', 'w', 'Position', [60 60 w h]);
try, theme(f, 'light'); catch, end
end

function save_fig(f, out, name)
print(f, '-dpng', '-r200', fullfile(out, [name '.png'])); close(f);
end

function bx(ax, x, y, w, h, txt, col)
rectangle('Parent', ax, 'Position', [x y w h], 'Curvature', 0.18, 'FaceColor', col, 'EdgeColor', [0.2 0.2 0.2], 'LineWidth', 1.6);
text(x + w / 2, y + h / 2, txt, 'Parent', ax, 'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle', 'FontSize', 15, 'Color', 'k');
end

function ar(ax, xs, ys, col, lw)
if nargin < 4, col = [0.15 0.15 0.15]; end
if nargin < 5, lw = 1.8; end
plot(ax, xs, ys, '-', 'Color', col, 'LineWidth', lw);
d = [xs(end) - xs(end-1), ys(end) - ys(end-1)]; d = d / norm(d); n = [-d(2) d(1)]; Lh = 0.42; Wh = 0.17;
patch(ax, xs(end) - [0, Lh * d(1) - Wh * n(1), Lh * d(1) + Wh * n(1)], ys(end) - [0, Lh * d(2) - Wh * n(2), Lh * d(2) + Wh * n(2)], col, 'EdgeColor', 'none');
end
