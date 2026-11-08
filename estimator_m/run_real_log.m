function run_real_log(variants)
% RUN_REAL_LOG  Run the ESKF on the recorded log and save results for real_data/compare_eskf.py.
% Needs real_data/data/real_log.mat from real_data/export_mat.py.
%   run_real_log                    % all variants
%   run_real_log({'eskf_gated_zv'}) % one variant
%
% All variants use DVL, depth and the VN-200 yaw as a heading aid; attitude comes from the raw gyro.
%   eskf_datasheet_zv  datasheet accelerometer noise (untuned), gated, zero-velocity aid
%   eskf_ungated_zv    tuned (config_real_log), no gating, zero-velocity aid
%   eskf_gated         tuned, 99% chi-square gating, no zero-velocity aid
%   eskf_gated_zv      tuned, gated, zero-velocity aid when the DVL is silent > 0.5 s
%   eskf_loosegate_zv  tuned, 99.99% gate, accept after 3 rejections in a row, zero-velocity aid
%   eskf_robust_zv     tuned, no gate; outliers down-weighted so their NIS is capped at the 99% value
root = fileparts(fileparts(mfilename('fullpath')));
data = fullfile(root, 'real_data', 'data');
all_variants = {'eskf_datasheet_zv', 'eskf_ungated_zv', 'eskf_gated', 'eskf_gated_zv', 'eskf_loosegate_zv', ...
                'eskf_robust_zv'};
if nargin < 1 || isempty(variants), variants = all_variants; end

tuned = config_real_log();
datasheet = tuned; datasheet.imu.accel_nd = config_estimator().imu.accel_nd;
gated.gate = struct('dvl', 11.34, 'depth', 6.63, 'heading', 6.63);   % chi2 99%: 3 dof, 1 dof
gated.max_consecutive_rejects = 10;
loose.gate = struct('dvl', 21.11, 'depth', 15.14, 'heading', 15.14); % chi2 99.99%
loose.max_consecutive_rejects = 3;
robust.robust = struct('dvl', 11.34, 'depth', 6.63, 'heading', 6.63);  % NIS caps at chi2 99%
log_every = 4;                                                           % 20 Hz output

for i = 1:numel(variants)
    v = variants{i};
    cfg = tuned; if strcmp(v, 'eskf_datasheet_zv'), cfg = datasheet; end
    use_zv = numel(v) > 3 && strcmp(v(end-2:end), '_zv');
    opts = gated;
    if strcmp(v, 'eskf_ungated_zv'), opts = struct(); end
    if strcmp(v, 'eskf_loosegate_zv'), opts = loose; end
    if strcmp(v, 'eskf_robust_zv'), opts = robust; end
    [tr, imu, meas, x0, P0, raw] = load_real_log(fullfile(data, 'real_log.mat'), cfg, use_zv);
    tic;
    res = run_filter(tr, imu, meas, [], cfg, 'rw', x0, P0, log_every, [], opts);
    el = toc;
    res.t_bag = raw.t0_bag + res.t;
    res.variant = v;
    res.dvl_k = [meas.dvl.k];
    save('-v7', fullfile(data, [v '.mat']), '-struct', 'res');
    fprintf('%s: %.0f s for %.0f s of data\n', v, el, tr.n * tr.dt);
    s = {'dvl', 'depth', 'heading', 'zero_vel'};
    for j = 1:numel(s)
        nv = res.nis.(s{j});
        if isempty(nv), continue; end
        fprintf('  %-8s n=%5d  rejected %5d  down-weighted %5d  median NIS %.3f\n', s{j}, numel(nv), ...
                res.rejected.(s{j}), res.downweighted.(s{j}), median(nv));
    end
    fprintf('  final accel bias [%.3f %.3f %.3f] m/s^2, gyro bias [%.3f %.3f %.3f] deg/s, depth bias %.3f m\n', ...
            res.x(end, 11:13), res.x(end, 14:16) * 180 / pi, res.x(end, 17));
end
end
