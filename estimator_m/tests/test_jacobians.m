function test_jacobians()
% TEST_JACOBIANS  Analytic Jacobians vs finite differences (log 12.4, 12.7-12.10).
% The measurement Jacobians and the lever-arm / Earth-rate terms are hand-derived
% and not in Sola or MSS, so this is their primary check.
cfg = config_estimator();
x = test_state(cfg, 0);
w_ib = [0.1; -0.2; 0.3];
w_meas = w_ib + x.bg;          % gyro bias enters the lever-arm terms through w_nb
names = {'dvl', 'depth', 'gnss_pos', 'gnss_vel', 'heading'};
for i = 1:numel(names)
    [~, H] = eskf_meas(names{i}, x, w_ib, cfg);
    hf = @(xx) meas_h(names{i}, xx, w_meas - xx.bg, cfg);
    Hn = num_jac(hf, x, 1e-6);
    if strcmp(names{i}, 'heading'), Hn = wrap_pi(Hn); end
    err = max(abs(H(:) - Hn(:)));
    assert(err < 1e-7 + 1e-5 * max(abs(Hn(:))), sprintf('%s Jacobian mismatch %.3e', names{i}, err));
    fprintf('  %-9s H max error %.2e\n', names{i}, err);
end

% F is first order (Sola 269): mismatch is O(dt^2), dominated by the position
% row's 0.5 R [a]x dt^2 term, and must shrink ~4x when dt halves.
models = {'rw', 'gm'};
for i = 1:2
    x = test_state(cfg, 1);
    wt = [0.05; -0.1; 0.2]; ft = [0.3; -0.2; -9.6];
    dt = 0.01;
    e1 = F_mismatch(x, wt, ft, 2 * dt, cfg, models{i});
    e2 = F_mismatch(x, wt, ft, dt, cfg, models{i});
    assert(e2 < 1.1 * 0.5 * norm(ft) * dt^2, sprintf('F mismatch %.3e', e2));
    assert(e1 / e2 > 3.5 && e1 / e2 < 4.5, sprintf('F ratio %.2f', e1 / e2));
    fprintf('  F (%s): mismatch %.2e at dt=0.01, ratio %.2f when dt halves\n', models{i}, e2, e1 / e2);
end
end


function x = test_state(cfg, seed)
rng(seed);
x = make_state(5 * randn(3, 1), randn(3, 1), euler2q(0.3, -0.4, 2.0), ...
               0.01 * randn(3, 1), 0.01 * randn(3, 1), 0.05);
end


function h = meas_h(name, x, w_ib, cfg)
[h, ~] = eskf_meas(name, x, w_ib, cfg);
end


function J = num_jac(fun, x, eps)
f0 = fun(x);
J = zeros(numel(f0), 16);
for i = 1:16
    d = zeros(16, 1); d(i) = eps;
    J(:, i) = (fun(state_oplus(x, d)) - fun(state_oplus(x, -d))) / (2 * eps);
end
end


function e = F_mismatch(x, wt, ft, dt, cfg, model)
dth = (wt + x.bg)' * dt; dv = (ft + x.ba)' * dt;
[xn, w, a, R] = eskf_nominal(x, dth, dv, dt, cfg, model);
F = eskf_F(R, w, a, dt, cfg, model);
prop = @(xx) state_ominus(eskf_nominal(xx, dth, dv, dt, cfg, model), xn);
Fn = num_jac(prop, x, 1e-7);
e = max(abs(F(:) - Fn(:)));
end
