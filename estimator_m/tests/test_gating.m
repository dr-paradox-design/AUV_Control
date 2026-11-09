function test_gating()
% TEST_GATING  Gate, robust mode and observability constraint in eskf_update; truth-free run_filter.
cfg = config_estimator();
cfg.zero_vel.sigma = 0.1;      % only set in config_real_log
x = make_state([1; 2; 3], [0.5; -0.2; 0.1], euler2q(0.1, -0.2, 1.0), zeros(3, 1), zeros(3, 1), 0);
P = initial_covariance(cfg);
w = [0.01; 0.02; -0.03];
z_ok = [0.6; 0.1; -0.05];

[x1, P1, n1, a1] = eskf_update(x, P, 'dvl', z_ok, w, cfg);
[x2, P2, n2, a2] = eskf_update(x, P, 'dvl', z_ok, w, cfg, Inf);
assert(a1 && a2 && n1 == n2 && isequal(x1, x2) && isequal(P1, P2), 'gate = Inf must equal no gate');

[x3, P3, n3, a3] = eskf_update(x, P, 'dvl', [50; 50; 50], w, cfg, 11.34);
assert(~a3 && n3 > 11.34, 'large innovation must be rejected');
assert(isequal(x3, x) && isequal(P3, P), 'a rejected update must leave x and P unchanged');
fprintf('  gate = Inf matches the ungated update; NIS %.0f rejected at 11.34 with x, P unchanged\n', n3);

% robust update: off by default; an outlier is used but with inflated noise so its NIS equals the cap
[x4, P4, n4, a4, l4] = eskf_update(x, P, 'dvl', z_ok, w, cfg, Inf, Inf);
assert(l4 == 1 && isequal(x4, x1) && isequal(P4, P1), 'robust off must equal the plain update');
zo = [5; -3; 2];
[x5, P5, n5, a5, l5] = eskf_update(x, P, 'dvl', zo, w, cfg, Inf, 11.34);
[h, H, Rm] = eskf_meas('dvl', x, w, cfg);
nis_eff = (zo - h)' * ((H * P * H' + l5 * Rm) \ (zo - h));
assert(a5 && l5 > 1 && abs(nis_eff - 11.34) < 1e-6 * 11.34, sprintf('robust NIS %.6f', nis_eff));
x6 = eskf_update(x, P, 'dvl', zo, w, cfg);
assert(norm(x5.v - x.v) < norm(x6.v - x.v), 'robust update must move the state less than the plain one');
assert(all(diag(P5) <= diag(P) + 1e-12), 'covariance must not grow in an update');
fprintf('  robust: NIS %.0f capped to 11.34 (lambda %.0f); velocity step %.3f vs %.3f m/s plain\n', ...
        n5, l5, norm(x5.v - x.v), norm(x6.v - x.v));

% observability constraint: with an isotropic prior P = s^2 I, an update leaves N' P N unchanged exactly
% when H N = 0. With oc, dvl / depth / zero_vel must not reduce it; without oc, zero_vel (v ~= 0) does.
xo = make_state([3; -2; 1], [0.4; 0.3; 0.05], euler2q(0.2, -0.1, 0.7), zeros(3, 1), zeros(3, 1), 0);
Pi = 0.04 * eye(16);
N = unobservable_directions(xo);
cases = {'dvl', [0.3; 0.2; 0.1]; 'depth', 1.2; 'zero_vel', zeros(3, 1)};
for i = 1:size(cases, 1)
    [~, Po] = eskf_update(xo, Pi, cases{i, 1}, cases{i, 2}, w, cfg, Inf, Inf, true);
    d = norm(N' * Po * N - N' * Pi * N);
    assert(d < 1e-12, sprintf('%s with oc reduced variance along N by %.3e', cases{i, 1}, d));
end
[~, Pz] = eskf_update(xo, Pi, 'zero_vel', zeros(3, 1), w, cfg);
dz = norm(N' * Pz * N - N' * Pi * N);
assert(dz > 1e-4, 'plain zero_vel should carry spurious information along N');
[~, Ph] = eskf_update(xo, Pi, 'heading', 0.5, w, cfg, Inf, Inf, true);
[~, Ph0] = eskf_update(xo, Pi, 'heading', 0.5, w, cfg);
assert(isequal(Ph, Ph0), 'oc must not change the heading update');
fprintf('  observability constraint: no variance change along N with oc (plain zero_vel: %.1e)\n', dz);

% truth-free run: no tr.p, so err and nees are NaN; state and covariance are still logged
tr.dt = 0.01; tr.n = 50; tr.t = (0:tr.n)' * tr.dt;
imu.dtheta = zeros(tr.n, 3);
imu.dvel = repmat(-cfg.g' * tr.dt, tr.n, 1);       % at rest, level: specific force = -g
meas.dvl = struct('k', {11, 31}, 'z', {[0; 0; 0], [100; 0; 0]});
opts.gate.dvl = 11.34;
res = run_filter(tr, imu, meas, [], cfg, 'rw', make_state([0 0 0], [0 0 0], [1 0 0 0], [0 0 0], [0 0 0], 0), ...
                 P, 10, {'dvl'}, opts);
assert(all(isnan(res.err(:))) && all(isnan(res.nees)), 'truth-free mode must not compute errors');
assert(isequal(res.accepted.dvl, [true false]) && res.rejected.dvl == 1, 'second DVL sample must be gated');
assert(all(isfinite(res.x(:))), 'state must stay finite');
fprintf('  truth-free run: err/NEES NaN, outlier DVL sample rejected\n');
end
