function res = run_once(seed, truth_bias, filter_bias, cfg, tr, log_every)
% RUN_ONCE  Simulate sensors with seed, initialise from P0, run the filter.
if nargin < 4 || isempty(cfg), cfg = config_estimator(); end
if nargin < 5 || isempty(tr), tr = truth_trajectory(cfg); end
if nargin < 6, log_every = 10; end
rng(seed);
[imu, meas, bd] = sim_sensors(tr, cfg, truth_bias);
P0 = initial_covariance(cfg);
xt0 = make_state(tr.p(1, :), tr.v(1, :), tr.q(1, :), imu.ba(1, :), imu.bg(1, :), bd(1));
x0 = state_oplus(xt0, chol(P0, 'lower') * randn(16, 1));
res = run_filter(tr, imu, meas, bd, cfg, filter_bias, x0, P0, log_every);
end
