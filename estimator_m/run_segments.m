function res = run_segments(tr, imu, meas, cfg, filter_bias, x0, P0, log_every, use, opts, gaps, vn_euler)
% RUN_SEGMENTS  run_filter over the gap-free segments of a recording.
% gaps (m x 2, s from the start) are intervals with no IMU data; the filter does not propagate through
% them. Each segment restarts from the VN-200 attitude (vn_euler, rows of [roll pitch yaw] on the grid),
% zero velocity and P0, keeps the previous segment's position and bias estimates, and ends just before the
% next gap. Results are concatenated with times on the original grid. With no gaps this is run_filter.
if isempty(gaps)
    res = run_filter(tr, imu, meas, [], cfg, filter_bias, x0, P0, log_every, use, opts);
    res.segments = [0, tr.n * tr.dt];
    return
end
edges = [0; gaps(:, 2)]; ends = [gaps(:, 1); tr.n * tr.dt];     % segment s spans edges(s)..ends(s)
names = fieldnames(meas);
x = x0; out = [];
for s = 1:numel(ends)
    k1 = floor(edges(s) / tr.dt) + 2; if s == 1, k1 = 1; end    % first sample after the gap
    k2 = floor(ends(s) / tr.dt);                                  % last sample before the next gap
    if k2 - k1 < 10, continue; end
    seg.dt = tr.dt; seg.n = k2 - k1; seg.t = (0:seg.n)' * tr.dt;
    si.dtheta = imu.dtheta(k1:k2 - 1, :); si.dvel = imu.dvel(k1:k2 - 1, :);
    sm = struct();
    for i = 1:numel(names)
        m = meas.(names{i});
        if isempty(m), sm.(names{i}) = m; continue; end
        keep = [m.k] >= k1 & [m.k] <= k2;
        m = m(keep);
        for j = 1:numel(m), m(j).k = m(j).k - k1 + 1; end
        sm.(names{i}) = m;
    end
    if s > 1
        e = vn_euler(k1, :);
        x = make_state(x.p, [0 0 0], euler2q(e(1), e(2), e(3)), x.ba, x.bg, x.bd);
    end
    r = run_filter(seg, si, sm, [], cfg, filter_bias, x, P0, log_every, use, opts);
    r.t = r.t + (k1 - 1) * tr.dt;
    for f = fieldnames(r.innov_t)', r.innov_t.(f{1}) = r.innov_t.(f{1}) + (k1 - 1) * tr.dt; end
    last = r.x(end, :);
    x = make_state(last(1:3), last(4:6), last(7:10), last(11:13), last(14:16), last(17));
    out = [out, r]; %#ok<AGROW>
    segs(s, :) = [(k1 - 1) * tr.dt, k2 * tr.dt]; %#ok<AGROW>
end
res = out(1);
cat_rows = {'t', 'x', 'sd', 'yaw_sd', 'err', 'nees'};
for i = 2:numel(out)
    for c = cat_rows, res.(c{1}) = [res.(c{1}); out(i).(c{1})]; end
    for f = fieldnames(res.nis)'
        res.nis.(f{1}) = [res.nis.(f{1}), out(i).nis.(f{1})];
        res.accepted.(f{1}) = [res.accepted.(f{1}), out(i).accepted.(f{1})];
        res.rejected.(f{1}) = res.rejected.(f{1}) + out(i).rejected.(f{1});
        res.downweighted.(f{1}) = res.downweighted.(f{1}) + out(i).downweighted.(f{1});
        res.innov.(f{1}) = [res.innov.(f{1}); out(i).innov.(f{1})];
        res.innov_t.(f{1}) = [res.innov_t.(f{1}); out(i).innov_t.(f{1})];
    end
end
res.P_end = out(end).P_end;
res.segments = segs(any(segs, 2), :);
end
