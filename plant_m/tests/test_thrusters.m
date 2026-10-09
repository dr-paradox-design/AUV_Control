function r = test_thrusters(p)
% TEST_THRUSTERS  Allocation and lag. The thruster geometry itself is ASSUMED, so only the
% algebra is tested here, not whether B matches the real vehicle.
S = 'thrusters'; r = new_results();
rng(2);
B = thrust_matrix(p);
res = 0; nullpart = 0; red = 0;
for i = 1:10
    tau = randn(6, 1) * 5;
    u = thrust_alloc(tau, p);
    res = max(res, max(abs(B * u - tau)));
    Bp = B' / (B * B');
    nullpart = max(nullpart, norm((eye(8) - Bp * B) * u));
    u2 = thrust_alloc(tau, p, randn(8, 1));
    red = max(red, max(abs(B * u2 - tau)));
end
tau = randn(6, 1); u = thrust_alloc(tau, p); u2 = thrust_alloc(tau, p, randn(8, 1));
r(end+1) = mk(S, 'allocation: B u = tau (max residual)', 'check', res, 1e-12);
r(end+1) = mk(S, 'allocation: minimum-norm solution has no null-space component', 'check', nullpart, 1e-12);
r(end+1) = mk(S, 'allocation: adding a null-space term leaves tau unchanged', 'check', red, 1e-12);
r(end+1) = mk(S, 'allocation: minimum norm (norm(u) - norm(u + null part) < 0)', 'check', norm(u) - norm(u2), 0);

% first-order lag: u(t) = uc (1 - exp(-t/Tt))
uc = 3; dt = 0.002; n = 500; u = zeros(n + 1, 1);
f = @(x, t) (uc - x) / p.Tt;
for k = 1:n, u(k + 1) = rk4(f, u(k), (k - 1) * dt, dt); end
t = (0:n)' * dt;
r(end+1) = mk(S, 'thruster lag: step response (max error, N)', 'check', max(abs(u - uc * (1 - exp(-t / p.Tt)))), 1e-9);
r(end+1) = mk(S, 'control: step response compared with a 10% wrong time constant', 'control', max(abs(u - uc * (1 - exp(-t / (1.1 * p.Tt))))), 1e-9);
end
