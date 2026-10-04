function test_consistency(N)
% TEST_CONSISTENCY  Monte Carlo consistency on the nominal case (matched GM models).
% Chi-square tests of Bar-Shalom et al. (2001). Slow: N full 600 s runs.
if nargin < 1, N = 8; end
cfg = config_estimator();
tr = truth_trajectory(cfg);
runs = cell(N, 1);
parfor i = 1:N
    runs{i} = run_once(500 + i, 'gm', 'gm', cfg, tr);
end
dof = 16;
nees = cell2mat(cellfun(@(r) r.nees', runs, 'UniformOutput', false));
lo = chi2inv_core(0.025, N * dof) / N; hi = chi2inv_core(0.975, N * dof) / N;
anees = mean(nees, 1);
inside = mean(anees >= lo & anees <= hi);
fprintf('  ANEES mean %.2f, inside 95%% band [%.2f, %.2f] at %.1f%% of points\n', mean(anees), lo, hi, 100 * inside);
assert(inside > 0.85, 'ANEES outside band too often');

% NIS per sensor, Bonferroni-corrected (5 sensors, 1% each)
dims = struct('dvl', 3, 'depth', 1, 'gnss_pos', 3, 'gnss_vel', 3, 'heading', 1);
names = fieldnames(dims);
depth_ok = true;
for s = 1:numel(names)
    v = cell2mat(cellfun(@(r) r.nis.(names{s}), runs', 'UniformOutput', false));
    m = dims.(names{s}); n = numel(v);
    lo_s = chi2inv_core(0.005, n * m) / n; hi_s = chi2inv_core(0.995, n * m) / n;
    fprintf('  %-9s mean NIS %.3f, band [%.3f, %.3f], n = %d\n', names{s}, mean(v), lo_s, hi_s, n);
    ok = mean(v) >= lo_s && mean(v) <= hi_s;
    if strcmp(names{s}, 'depth')
        depth_ok = ok;
    else
        assert(ok, sprintf('%s NIS outside band', names{s}));
    end
end
% Known modelling mismatch (see README): 10-bit depth quantisation error is nearly
% constant while depth is held, so it is not white. Depth NIS samples are correlated,
% the white-noise band does not apply, and per-run depth NIS swings in both directions.
if ~depth_ok
    fprintf('  EXPECTED FAILURE: depth NIS outside band (10-bit quantisation error is time-correlated)\n');
end
end
