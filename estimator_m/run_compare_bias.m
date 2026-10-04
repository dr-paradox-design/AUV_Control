function T = run_compare_bias(N)
% RUN_COMPARE_BIAS  Random-walk vs Gauss-Markov bias models (log 12.6).
% Truth bias model x filter bias model, N Monte Carlo runs each (paired seeds).
% The two truth models share the same driving noise density and differ only in decay.
% Writes results/compare_bias.txt. Uses parfor when the Parallel Computing Toolbox is present.
if nargin < 1, N = 20; end
setup_paths();
cfg = config_estimator();
tr = truth_trajectory(cfg);
combos = {'rw', 'rw'; 'rw', 'gm'; 'gm', 'rw'; 'gm', 'gm'};
dof = 16;
lo = chi2inv_core(0.025, N * dof) / N; hi = chi2inv_core(0.975, N * dof) / N;
lines = {sprintf('Bias-model comparison (MATLAB), N = %d runs per cell, 600 s scenario, all parameters ASSUMED', N), ...
         'Same seeds across filter models within a truth model (paired comparison).', '', ...
         sprintf('%5s %6s | %6s %6s | %6s %6s %6s %6s [m RMS] | %7s [deg/h] %7s [mg]', ...
                 'truth', 'filter', 'ANEES', 'in95%', 'h@300', 'h@360', 'h@500', 'h@599', 'bg_end', 'ba_end')};
for c = 1:size(combos, 1)
    tb = combos{c, 1}; fb = combos{c, 2};
    runs = cell(N, 1);
    parfor i = 1:N
        runs{i} = run_once(1000 + i, tb, fb, cfg, tr);
    end
    t = runs{1}.t;
    nees = cell2mat(cellfun(@(r) r.nees', runs, 'UniformOutput', false));
    anees = mean(nees, 1);
    inside = mean(anees >= lo & anees <= hi);
    E = zeros(16, numel(t), N);                      % 16 x T x N
    for i = 1:N, E(:, :, i) = runs{i}.err'; end
    hrms = zeros(1, 4); tt = [300 360 500 599];
    for j = 1:4
        [~, ii] = min(abs(t - tt(j)));
        hrms(j) = sqrt(mean(sum(squeeze(E(1:2, ii, :)).^2, 1)));
    end
    bg_end = sqrt(mean(reshape(E(13:15, end, :), [], 1).^2)) * 180 / pi * 3600;
    ba_end = sqrt(mean(reshape(E(10:12, end, :), [], 1).^2)) / 9.80665 * 1e3;
    lines{end + 1} = sprintf('%5s %6s | %6.2f %5.1f%% | %6.2f %6.2f %6.2f %6.2f        | %7.2f         %7.4f', ...
        tb, fb, mean(anees), 100 * inside, hrms, bg_end, ba_end); %#ok<AGROW>
    names = fieldnames(runs{1}.nis); s = '      mean NIS:';
    for k = 1:numel(names)
        v = cell2mat(cellfun(@(r) r.nis.(names{k}), runs', 'UniformOutput', false));
        s = sprintf('%s %s %.2f', s, names{k}, mean(v));
    end
    lines{end + 1} = s; %#ok<AGROW>
end
lines{end + 1} = '';
lines{end + 1} = sprintf('ANEES 95%% bounds for N=%d, dof 16: [%.2f, %.2f].', N, lo, hi);
lines{end + 1} = 'Timeline: surface 0-60 s (GNSS), dive 60-110 s, DVL dropout 300-360 s, ascent 500-550 s, surface 550-600 s.';
T = strjoin(lines, newline);
disp(T);
here = fileparts(mfilename('fullpath'));
if ~exist(fullfile(here, 'results'), 'dir'), mkdir(fullfile(here, 'results')); end
fid = fopen(fullfile(here, 'results', 'compare_bias.txt'), 'w'); fprintf(fid, '%s\n', T); fclose(fid);
end
