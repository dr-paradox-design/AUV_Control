function run_real_log(variants, run)
% run (optional): run name, data in real_data/data/<run>/ (from REAL_DATA_RUN=<run> python ...).
% Omit it for the 2026-11-05 log in real_data/data/.
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
%   eskf_noheading_zv  tuned, no gate, NO VN-200 yaw aid: yaw only from the gyros (observability test)
%   eskf_noheading_oc_zv  as eskf_noheading_zv, with the observability constraint (opts.oc)
%   eskf_oc_zv            as eskf_ungated_zv, with the observability constraint
%   eskf_oc_robusthdg_zv  as eskf_oc_zv, plus a robust heading update (NIS cap 6.63) against VN-200 yaw jumps
%   eskf_accel30_zv       as eskf_ungated_zv with accelerometer noise x30 (NIS-tuned on the 2026-11-11 run)
%   eskf_robust_x5_zv     robust update with accelerometer noise x5 (corrected 2026-11-05 DVL mounting)
%   eskf_hdg1hz_zv        as eskf_ungated_zv, VN-200 yaw used at 1 Hz (sigma 1.3 deg)
%   eskf_hdg0p2hz_zv      as eskf_ungated_zv, VN-200 yaw used at 0.2 Hz with sigma 3 deg (less trust in a
%                         time-correlated, jumpy magnetometer heading)
root = fileparts(fileparts(mfilename('fullpath')));
data = fullfile(root, 'real_data', 'data');
if nargin >= 2 && ~isempty(run), data = fullfile(data, run); end
all_variants = {'eskf_datasheet_zv', 'eskf_ungated_zv', 'eskf_gated', 'eskf_gated_zv', 'eskf_loosegate_zv', ...
                'eskf_robust_zv', 'eskf_noheading_zv', 'eskf_noheading_oc_zv', 'eskf_oc_zv', 'eskf_oc_robusthdg_zv', ...
                'eskf_hdg1hz_zv', 'eskf_hdg0p2hz_zv'};
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
    if strcmp(v, 'eskf_hdg1hz_zv'), cfg.heading.rate_hz = 1; end
    if strcmp(v, 'eskf_hdg0p2hz_zv'), cfg.heading.rate_hz = 0.2; cfg.heading.sigma = 3 * pi / 180; end
    if strcmp(v, 'eskf_accel30_zv'), cfg.imu.accel_nd = 30 * config_estimator().imu.accel_nd; end
    if strcmp(v, 'eskf_robust_x5_zv'), cfg.imu.accel_nd = 5 * config_estimator().imu.accel_nd; end
    use_zv = numel(v) > 3 && strcmp(v(end-2:end), '_zv');
    opts = gated;
    if strcmp(v, 'eskf_ungated_zv'), opts = struct(); end
    if strcmp(v, 'eskf_loosegate_zv'), opts = loose; end
    if any(strcmp(v, {'eskf_robust_zv', 'eskf_robust_x5_zv'})), opts = robust; end
    use = [];
    if strcmp(v, 'eskf_noheading_zv'), opts = struct(); use = {'dvl', 'depth', 'zero_vel'}; end
    if strcmp(v, 'eskf_noheading_oc_zv'), opts = struct('oc', true); use = {'dvl', 'depth', 'zero_vel'}; end
    if strcmp(v, 'eskf_oc_zv'), opts = struct('oc', true); end
    if strcmp(v, 'eskf_oc_robusthdg_zv'), opts = struct('oc', true); opts.robust.heading = 6.63; end
    if any(strcmp(v, {'eskf_hdg1hz_zv', 'eskf_hdg0p2hz_zv', 'eskf_accel30_zv'})), opts = struct(); end
    [tr, imu, meas, x0, P0, raw, cfg] = load_real_log(fullfile(data, 'real_log.mat'), cfg, use_zv);
    tic;
    gaps = zeros(0, 2); if isfield(raw, 'imu_gaps'), gaps = reshape(raw.imu_gaps, [], 2); end
    res = run_segments(tr, imu, meas, cfg, 'rw', x0, P0, log_every, use, opts, gaps, raw.vn_euler_grid);
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
