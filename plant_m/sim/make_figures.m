function make_figures()
% MAKE_FIGURES  Two figures from the plant's own test runs, written to results/:
%   fig_galilean.png      still-water run, run in a uniform current with and without the
%                         M_A nu_c_dot term, and the still run shifted by v_c t (report 5.3)
%   fig_suite_margins.png error / tolerance of every check and failure control
% Re-runs the invariance suite (about 20 s) and reads results/test_results.mat.
here = fileparts(mfilename('fullpath'));
root = fileparts(here);
addpath(fullfile(root, 'config')); setup_plant_paths();
outdir = fullfile(root, 'results');
if ~exist(outdir, 'dir'), mkdir(outdir); end
p = params_plant();

[~, info] = test_invariance(p);
o0 = info.o0; oc = info.oc; ob = info.ob; vc = info.vc;
shift = o0.X(:, 1:3) + o0.t * vc';

f = figure('Visible', 'off', 'Color', 'w', 'Position', [100 100 1300 480]);
light_theme(f);
subplot(1, 2, 1); hold on; box on; grid on;
plot(o0.X(:, 1), o0.X(:, 2), 'k-', 'LineWidth', 1.4);
plot(oc.X(:, 1), oc.X(:, 2), '-', 'Color', [0 0 0.7], 'LineWidth', 1.4);
plot(ob.X(:, 1), ob.X(:, 2), '--', 'Color', [0.8 0.1 0.1], 'LineWidth', 1.4);
plot(shift(:, 1), shift(:, 2), ':', 'Color', [0 0.6 0.2], 'LineWidth', 2.2);
a = o0.X(end, 1:2); b = shift(end, 1:2);
quiver(a(1), a(2), b(1) - a(1), b(2) - a(2), 0, 'Color', [0.6 0 0], 'LineWidth', 1.4, 'MaxHeadSize', 0.3);
axis equal; xlabel('North [m]'); ylabel('East [m]');
title('Turning manoeuvre, uniform current v_c = [0.3, -0.2] m/s, 30 s');
legend({'still water  p_0(t)', 'current, with M_A \nu_c dot  p_c(t)', 'current, M_A \nu_c dot dropped', ...
        'p_0(t) + v_c t', 'v_c t_f'}, 'Location', 'best');

subplot(1, 2, 2);
e1 = sqrt(sum((oc.X(:, 1:3) - shift).^2, 2));
e2 = sqrt(sum((ob.X(:, 1:3) - shift).^2, 2));
semilogy(oc.t, max(e1, 1e-16), '-', 'Color', [0 0 0.7], 'LineWidth', 1.4); hold on; box on; grid on;
semilogy(ob.t, max(e2, 1e-16), '--', 'Color', [0.8 0.1 0.1], 'LineWidth', 1.4);
xlabel('time [s]'); ylabel('| p_c(t) - p_0(t) - v_c t |  [m]');
title(sprintf('Invariance error: %.1e m with the term, %.1f m without', info.eG, info.eGbad));
legend({'with M_A \nu_c dot', 'M_A \nu_c dot dropped'}, 'Location', 'east');
print(f, '-dpng', '-r150', fullfile(outdir, 'fig_galilean.png')); close(f);

% margins
S = load(fullfile(outdir, 'test_results.mat')); r = S.r;
keep = ~strcmp({r.kind}, 'info') & [r.tol] > 0 & [r.err] > 0 | (~strcmp({r.kind}, 'info') & [r.tol] > 0 & [r.err] == 0);
r = r(keep);
suites = unique({r.suite}, 'stable');
f = figure('Visible', 'off', 'Color', 'w', 'Position', [100 100 1300 520]);
light_theme(f);
hold on; box on; grid on;
for i = 1:numel(r)
    x = find(strcmp(suites, r(i).suite));
    jit = (mod(i * 37, 11) - 5) / 30;
    y = max(abs(r(i).err) / r(i).tol, 1e-8);
    if strcmp(r(i).kind, 'check')
        h1 = plot(x + jit, y, 'o', 'MarkerFaceColor', [0.2 0.4 0.8], 'MarkerEdgeColor', 'k', 'MarkerSize', 7);
    else
        h2 = plot(x + jit, y, 's', 'MarkerFaceColor', [0.9 0.5 0.1], 'MarkerEdgeColor', 'k', 'MarkerSize', 7);
    end
end
set(gca, 'YScale', 'log', 'XTick', 1:numel(suites), 'XTickLabel', suites, 'XLim', [0.4 numel(suites) + 0.6], 'YLim', [1e-9 1e14]);
h3 = plot(get(gca, 'XLim'), [1 1], 'k--', 'LineWidth', 1.4);
ylabel('error / tolerance  (log scale)');
title('Every check must sit below 1; every failure control must sit above 1');
legend([h1 h2 h3], {'check (passes below 1)', 'failure control (passes above 1)', 'tolerance'}, 'Location', 'northwest');
print(f, '-dpng', '-r150', fullfile(outdir, 'fig_suite_margins.png')); close(f);
fprintf('wrote figures to %s\n', outdir);
end


function light_theme(f)
% Newer MATLAB follows the system theme, which can give dark axes on a white page.
try, theme(f, 'light'); catch, end
end
