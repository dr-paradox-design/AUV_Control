function test_cross_check_python()
% TEST_CROSS_CHECK_PYTHON  Same inputs as the frozen Python reference
% (estimator/export_reference.py) must give the same truth and filter output.
here = fileparts(mfilename('fullpath'));
ref = load(fullfile(here, 'reference_python.mat'));
cfg = config_estimator();
cfg.sc.duration = double(ref.duration);
le = double(ref.log_every);
tr = truth_trajectory(cfg);

% truth
idx = 1:le:tr.n + 1; idn = 1:le:tr.n;
et = max([max(max(abs(tr.p(idx, :) - ref.truth_p))), max(max(abs(tr.v(idx, :) - ref.truth_v))), ...
          max(max(abs(tr.q(idx, :) - ref.truth_q))), max(max(abs(tr.dtheta(idn, :) - ref.dtheta_true))), ...
          max(max(abs(tr.dvel(idn, :) - ref.dvel_true)))]);
fprintf('  truth vs Python: max difference %.2e\n', et);
assert(et < 1e-10, 'truth trajectory differs from Python');

% filter on identical inputs
imu.dtheta = ref.imu_dtheta; imu.dvel = ref.imu_dvel;
imu.ba = zeros(tr.n + 1, 3); imu.bg = zeros(tr.n + 1, 3);   % only used for the error log
names = {'dvl', 'depth', 'gnss_pos', 'gnss_vel', 'heading'};
for s = 1:numel(names)
    K = ref.(['k_' names{s}]); Z = ref.(['z_' names{s}]);
    m = struct('k', {}, 'z', {});
    for j = 1:numel(K), m(j).k = K(j); m(j).z = Z(j, :)'; end
    meas.(names{s}) = m;
end
x0v = ref.x0;
x0 = make_state(x0v(1:3), x0v(4:6), x0v(7:10), x0v(11:13), x0v(14:16), x0v(17));
res = run_filter(tr, imu, meas, zeros(tr.n + 1, 1), cfg, char(ref.bias_model), x0, ref.P0, le);
ex = max(max(abs(res.x - ref.x_log)));
eP = max(max(abs(res.P_end - ref.P_end))) / max(max(abs(ref.P_end)));
fprintf('  filter states vs Python over %d s: max difference %.2e\n', round(cfg.sc.duration), ex);
fprintf('  final covariance vs Python: max relative difference %.2e\n', eP);
assert(ex < 1e-9, 'filter states differ from Python');
assert(eP < 1e-9, 'filter covariance differs from Python');
end
