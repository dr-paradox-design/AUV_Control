function [tr, imu, meas, x0, P0, raw] = load_real_log(matfile, cfg, use_zero_vel, drop_dvl)
% LOAD_REAL_LOG  Filter inputs from real_data/export_mat.py output (body FRD, world NED).
% tr has t, dt, n only (no truth). x0 starts at the origin with the VN-200 attitude.
% drop_dvl (optional, m x 2, seconds from the log start): DVL samples inside these windows are removed
% before anything else, as artificial outages for benchmarking.
if nargin < 3, use_zero_vel = false; end
if nargin < 4, drop_dvl = zeros(0, 2); end
raw = load(matfile);
td = (double(raw.dvl_k(:)) - 1) * raw.dt;
keep = true(size(td));
for i = 1:size(drop_dvl, 1)
    keep(td >= drop_dvl(i, 1) & td < drop_dvl(i, 2)) = false;
end
raw.dvl_k = raw.dvl_k(keep);
raw.dvl_z = raw.dvl_z(keep, :);
tr.dt = raw.dt; tr.n = double(raw.n);
tr.t = (0:tr.n)' * tr.dt;

imu.dtheta = raw.dtheta;
imu.dvel = raw.dvel;

meas.dvl = to_struct(raw.dvl_k, raw.dvl_z);
meas.depth = to_struct(raw.depth_k, raw.depth_z);
hk = double(raw.heading_k(:)); hz = raw.heading_z(:);
src_rate = (numel(hk) - 1) / ((hk(end) - hk(1)) * tr.dt);   % 10 Hz in the export (IMU stamps are bursty)
keep = 1:max(1, round(src_rate / cfg.heading.rate_hz)):numel(hk);   % decimate to cfg.heading.rate_hz
meas.heading = to_struct(hk(keep), hz(keep));
meas.gnss_pos = struct('k', {}, 'z', {});
meas.gnss_vel = struct('k', {}, 'z', {});
meas.zero_vel = struct('k', {}, 'z', {});
if use_zero_vel
    t_dvl = (double(raw.dvl_k(:)) - 1) * tr.dt;
    step = round(cfg.zero_vel.period / tr.dt);
    ks = (1:step:tr.n + 1)';
    tk = (ks - 1) * tr.dt;
    idx = discretize_prev(t_dvl, tk);                 % last DVL sample at or before tk
    last = -Inf(size(tk)); last(idx > 0) = t_dvl(idx(idx > 0));
    ks = ks(tk - last > cfg.zero_vel.stale_after);
    meas.zero_vel = to_struct(ks, zeros(numel(ks), 3));
end

q0 = raw.q0(:)';
R0 = q2R(q0);
rl = R0 * cfg.depth.lever;
x0 = make_state([0 0 raw.depth0 - rl(3)], [0 0 0], q0, [0 0 0], [0 0 0], 0);
P0 = initial_covariance(cfg);
end

function s = to_struct(k, z)
k = double(k(:));
if isempty(k), s = struct('k', {}, 'z', {}); return; end    % e.g. a run without depth
if isvector(z), z = z(:); end
s = struct('k', num2cell(k), 'z', num2cell(z', 1)');
end

function idx = discretize_prev(t_sorted, tq)
% index of the last t_sorted <= tq (0 if none); t_sorted ascending
idx = zeros(size(tq));
j = 0;
for i = 1:numel(tq)
    while j < numel(t_sorted) && t_sorted(j + 1) <= tq(i), j = j + 1; end
    idx(i) = j;
end
end
