function res = run_filter(tr, imu, meas, bd, cfg, filter_bias, x0, P0, log_every, use)
% RUN_FILTER  Run the ESKF over simulated (or recorded) data.
% Measurements at the same sample are fused in the order
% dvl, depth, gnss_pos, gnss_vel, heading (same as the Python reference).
if nargin < 9 || isempty(log_every), log_every = 10; end
names = {'dvl', 'depth', 'gnss_pos', 'gnss_vel', 'heading'};
if nargin < 10 || isempty(use), use = names; end

% schedule: rows [sample index, sensor index, measurement index]
sched = zeros(0, 3);
for s = 1:numel(names)
    if any(strcmp(use, names{s})) && ~isempty(meas.(names{s}))
        ks = [meas.(names{s}).k]';
        sched = [sched; ks, s * ones(size(ks)), (1:numel(ks))'];  %#ok<AGROW>
    end
end
sched = sortrows(sched, [1 2]);
first = ones(tr.n + 2, 1) * (size(sched, 1) + 1);
for r = size(sched, 1):-1:1, first(sched(r, 1)) = r; end

Q = eskf_Q(tr.dt, cfg, filter_bias);
x = x0; P = P0; w_ib = zeros(3, 1);
nis = struct(); for s = 1:numel(names), nis.(names{s}) = []; end
nlog = floor(tr.n / log_every) + 1;
res.t = zeros(nlog, 1); res.err = zeros(nlog, 16); res.sd = zeros(nlog, 16); res.nees = zeros(nlog, 1);
res.x = zeros(nlog, 17);
li = 0;

for k = 1:tr.n + 1                   % sample index (1-based)
    if k > 1
        [xn, w, a, R] = eskf_nominal(x, imu.dtheta(k-1, :), imu.dvel(k-1, :), tr.dt, cfg, filter_bias);
        F = eskf_F(R, w, a, tr.dt, cfg, filter_bias);
        P = F * P * F' + Q;
        x = xn; w_ib = w;
    end
    r = first(k);
    while r <= size(sched, 1) && sched(r, 1) == k
        name = names{sched(r, 2)};
        z = meas.(name)(sched(r, 3)).z;
        [x, P, nv] = eskf_update(x, P, name, z, w_ib, cfg);
        nis.(name)(end + 1) = nv;
        r = r + 1;
    end
    if mod(k - 1, log_every) == 0
        li = li + 1;
        xt = make_state(tr.p(k, :), tr.v(k, :), tr.q(k, :), imu.ba(k, :), imu.bg(k, :), bd(k));
        e = state_ominus(xt, x);
        res.t(li) = tr.t(k);
        res.err(li, :) = e';
        res.sd(li, :) = sqrt(diag(P))';
        res.nees(li) = e' * (P \ e);
        res.x(li, :) = [x.p', x.v', x.q, x.ba', x.bg', x.bd];
    end
end
res.nis = nis;
res.P_end = P;
end
