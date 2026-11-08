function test_gating()
% TEST_GATING  The optional NIS gate in eskf_update and the truth-free mode of run_filter.
cfg = config_estimator();
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
