function test_mechanisation()
% TEST_MECHANISATION  Conventions, truth consistency, noise-free mechanisation.
cfg = config_estimator();

% rotation conventions
phi = 0.3; th = -0.4; psi = 2.0;
R = q2R(euler2q(phi, th, psi));
assert(max(max(abs(R - euler2R(phi, th, psi)))) < 1e-12);
assert(max(max(abs(R' * R - eye(3)))) < 1e-12 && abs(det(R) - 1) < 1e-12);
[a, b, c] = R2euler(R); assert(max(abs([a b c] - [phi th psi])) < 1e-12);
rv = [0.1 -0.2 0.3]; assert(max(abs(qlog(qexp(rv)) - rv)) < 1e-12);
assert(isequal(vn2hamilton([1 2 3 4]), [4 1 2 3]));
fprintf('  rotation conventions OK\n');

% truth: depth integral matches the analytic depth
tr = truth_trajectory(cfg);
e = max(abs(tr.pz_check - tr.p(:, 3)));
assert(e < 1e-6, sprintf('depth integral %.2e', e));
fprintf('  truth depth integral vs analytic: %.2e m\n', e);

% truth: R_dot = R [w_nb]x (finite differences inside a turn)
i = round(160 / tr.dt) + 1; h = tr.dt;
Rm = q2R(tr.q(i - 1, :)); Rp = q2R(tr.q(i + 1, :)); R0 = q2R(tr.q(i, :));
Rdot = (Rp - Rm) / (2 * h);
e = max(max(abs(Rdot - R0 * skew3(tr.w_nb(i, :)'))));
assert(e < 1e-5, sprintf('body rates %.2e', e));      % central difference at dt = 0.01 s
fprintf('  truth body rates vs attitude derivative: %.2e\n', e);

% noise-free, bias-free increments through the filter mechanisation, 600 s
x = make_state(tr.p(1, :), tr.v(1, :), tr.q(1, :), zeros(3, 1), zeros(3, 1), 0);
for k = 1:tr.n
    x = eskf_nominal(x, tr.dtheta(k, :), tr.dvel(k, :), tr.dt, cfg, 'rw');
end
pe = norm(x.p - tr.p(end, :)');
ae = norm(qlog(qmul(qconj(x.q), tr.q(end, :))));
fprintf('  600 s noise-free: position error %.3e m, attitude error %.3e rad\n', pe, ae);
assert(pe < 0.05 && ae < 1e-6);
end
