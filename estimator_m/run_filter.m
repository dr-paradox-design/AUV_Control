function res = run_filter(tr, imu, meas, bd, cfg, filter_bias, x0, P0, log_every, use, opts)
% RUN_FILTER  Run the ESKF over simulated (or recorded) data.
% Measurements at the same sample are fused in the order
% dvl, depth, gnss_pos, gnss_vel, heading, zero_vel (same as the Python reference for the first five).
% Truth-free mode: if tr has no field p (recorded data), err and nees are NaN.
% opts (optional): opts.gate.<sensor> = chi-square threshold on NIS (default Inf, no gating);
%                  opts.max_consecutive_rejects (default Inf): accept anyway after this many rejections
%                  in a row, so the filter cannot lock itself out;
%                  opts.robust.<sensor> = NIS cap: inflate R so the NIS of an outlier equals the cap
%                  (default Inf, off). See eskf_update.
if nargin < 9 || isempty(log_every), log_every = 10; end
names = {'dvl', 'depth', 'gnss_pos', 'gnss_vel', 'heading', 'zero_vel'};
if nargin < 10 || isempty(use), use = names; end
if nargin < 11 || isempty(opts), opts = struct(); end
if ~isfield(opts, 'max_consecutive_rejects'), opts.max_consecutive_rejects = Inf; end
has_truth = isfield(tr, 'p');

% schedule: rows [sample index, sensor index, measurement index]
sched = zeros(0, 3);
for s = 1:numel(names)
    if any(strcmp(use, names{s})) && isfield(meas, names{s}) && ~isempty(meas.(names{s}))
        ks = [meas.(names{s}).k]';
        sched = [sched; ks, s * ones(size(ks)), (1:numel(ks))'];  %#ok<AGROW>
    end
end
sched = sortrows(sched, [1 2]);
first = ones(tr.n + 2, 1) * (size(sched, 1) + 1);
for r = size(sched, 1):-1:1, first(sched(r, 1)) = r; end

gate = struct(); robust = struct(); consec = struct();
res.rejected = struct(); res.accepted = struct(); res.downweighted = struct();
for s = 1:numel(names)
    gate.(names{s}) = Inf; robust.(names{s}) = Inf;
    if isfield(opts, 'gate') && isfield(opts.gate, names{s}), gate.(names{s}) = opts.gate.(names{s}); end
    if isfield(opts, 'robust') && isfield(opts.robust, names{s}), robust.(names{s}) = opts.robust.(names{s}); end
    consec.(names{s}) = 0; res.rejected.(names{s}) = 0; res.downweighted.(names{s}) = 0;
end

Q = eskf_Q(tr.dt, cfg, filter_bias);
x = x0; P = P0; w_ib = zeros(3, 1);
nis = struct();
for s = 1:numel(names)
    if isfield(meas, names{s}) && any(strcmp(use, names{s}))
        nis.(names{s}) = zeros(1, numel(meas.(names{s})));
        res.accepted.(names{s}) = true(1, numel(meas.(names{s})));
    else
        nis.(names{s}) = [];
        res.accepted.(names{s}) = true(1, 0);
    end
end
nlog = floor(tr.n / log_every) + 1;
res.t = zeros(nlog, 1); res.err = NaN(nlog, 16); res.sd = zeros(nlog, 16); res.nees = NaN(nlog, 1);
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
        g = gate.(name);
        if consec.(name) >= opts.max_consecutive_rejects, g = Inf; end
        [x, P, nv, ok, lam] = eskf_update(x, P, name, z, w_ib, cfg, g, robust.(name));
        nis.(name)(sched(r, 3)) = nv;
        res.accepted.(name)(sched(r, 3)) = ok;
        if lam > 1, res.downweighted.(name) = res.downweighted.(name) + 1; end
        if ok
            consec.(name) = 0;
        else
            consec.(name) = consec.(name) + 1;
            res.rejected.(name) = res.rejected.(name) + 1;
        end
        r = r + 1;
    end
    if mod(k - 1, log_every) == 0
        li = li + 1;
        res.t(li) = tr.t(k);
        res.sd(li, :) = sqrt(diag(P))';
        res.x(li, :) = [x.p', x.v', x.q, x.ba', x.bg', x.bd];
        if has_truth
            xt = make_state(tr.p(k, :), tr.v(k, :), tr.q(k, :), imu.ba(k, :), imu.bg(k, :), bd(k));
            e = state_ominus(xt, x);
            res.err(li, :) = e';
            res.nees(li) = e' * (P \ e);
        end
    end
end
res.nis = nis;
res.P_end = P;
end
