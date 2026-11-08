function run_outage_benchmark(len_s)
% RUN_OUTAGE_BENCHMARK  Artificial DVL outages on the recorded log.
% Reads real_data/data/outage_windows_<len_s>.mat (from real_data/outage_benchmark.py windows <len_s>),
% removes the DVL samples inside every window, runs each ESKF variant, and saves 20 Hz positions to
% real_data/data/outage_<variant>_<len_s>.mat for outage_benchmark.py report <len_s>.
if nargin < 1, len_s = 20; end
tag = sprintf('%g', len_s);
root = fileparts(fileparts(mfilename('fullpath')));
data = fullfile(root, 'real_data', 'data');
w = load(fullfile(data, ['outage_windows_' tag '.mat']));
cfg = config_real_log();
robust.robust = struct('dvl', 11.34, 'depth', 6.63, 'heading', 6.63);
variants = {
  'eskf_ungated',    false, struct()
  'eskf_ungated_zv', true,  struct()
  'eskf_robust_zv',  true,  robust
};
for i = 1:size(variants, 1)
    [tr, imu, meas, x0, P0, raw] = load_real_log(fullfile(data, 'real_log.mat'), cfg, variants{i, 2}, w.windows);
    res = run_filter(tr, imu, meas, [], cfg, 'rw', x0, P0, 4, [], variants{i, 3});
    out.t_bag = raw.t0_bag + res.t;
    out.x = res.x(:, 1:6);
    save('-v7', fullfile(data, ['outage_' variants{i, 1} '_' tag '.mat']), '-struct', 'out');
    fprintf('%s: %d DVL samples kept, %d windows removed\n', variants{i, 1}, numel(meas.dvl), size(w.windows, 1));
end
end
