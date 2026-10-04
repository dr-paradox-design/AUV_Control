function [imu, meas, bd] = sim_sensors(tr, cfg, bias_model)
% SIM_SENSORS  IMU increments with biases and noise, plus aiding measurements.
% Uses the global random stream; call rng(seed) first.
% meas.<sensor> is a struct array with fields k (1-based sample index) and z (column).
dt = tr.dt; n = tr.n;
I = cfg.imu;
imu.bg = sim_bias(n, dt, I.gyro_bias_sigma, I.bias_tau, bias_model);
imu.ba = sim_bias(n, dt, I.accel_bias_sigma, I.bias_tau, bias_model);
imu.dtheta = tr.dtheta + imu.bg(1:n, :) * dt + I.gyro_nd * sqrt(dt) * randn(n, 3);
imu.dvel = tr.dvel + imu.ba(1:n, :) * dt + I.accel_nd * sqrt(dt) * randn(n, 3);

bd = zeros(n + 1, 1);
bd(1) = cfg.depth.bias_sigma0 * randn;
bd(2:end) = bd(1) + cumsum(cfg.depth.bias_rw * sqrt(dt) * randn(n, 1));
step = cfg.depth.full_scale / cfg.depth.adc_counts;

e = @(rate) round(cfg.imu.rate_hz / rate);
meas = struct('dvl', struct('k', {}, 'z', {}), 'depth', struct('k', {}, 'z', {}), ...
              'gnss_pos', struct('k', {}, 'z', {}), 'gnss_vel', struct('k', {}, 'z', {}), ...
              'heading', struct('k', {}, 'z', {}));
for k = 0:n
    i = k + 1;
    t = tr.t(i); q = tr.q(i, :); R = q2R(q); v = tr.v(i, :)'; p = tr.p(i, :)'; w = tr.w_nb(i, :)';
    if mod(k, e(cfg.dvl.rate_hz)) == 0
        alt = cfg.dvl.seabed_depth - p(3);
        if p(3) > cfg.dvl.min_depth && ~(t >= cfg.dvl.dropout(1) && t < cfg.dvl.dropout(2)) ...
                && alt < cfg.dvl.max_altitude
            z = cfg.dvl.R_bd' * (R' * v + cross(w, cfg.dvl.lever)) + cfg.dvl.sigma * randn(3, 1);
            meas.dvl(end + 1) = struct('k', i, 'z', z);
        end
    end
    if mod(k, e(cfg.depth.rate_hz)) == 0
        rl = R * cfg.depth.lever;
        d = p(3) + rl(3) + bd(i) + cfg.depth.sigma * randn;
        d = min(max(round(d / step) * step, 0), cfg.depth.full_scale);
        meas.depth(end + 1) = struct('k', i, 'z', d);
    end
    if mod(k, e(cfg.gnss.rate_hz)) == 0
        lg = cfg.gnss.lever;
        pa = p + R * lg;
        if pa(3) < 0                                   % antenna above water
            meas.gnss_pos(end + 1) = struct('k', i, 'z', pa + cfg.gnss.sigma_pos .* randn(3, 1));
            meas.gnss_vel(end + 1) = struct('k', i, 'z', v + R * cross(w, lg) + cfg.gnss.sigma_vel * randn(3, 1));
        end
    end
    if mod(k, e(cfg.heading.rate_hz)) == 0
        meas.heading(end + 1) = struct('k', i, 'z', tr.euler(i, 3) + cfg.heading.sigma * randn);
    end
end
end


function b = sim_bias(n, dt, sigma, tau, model)
% 'gm': first-order Gauss-Markov. 'rw': random walk with the same driving noise
% density (2 sigma^2 / tau), so the two differ only in decay.
b = zeros(n + 1, 3);
b(1, :) = sigma * randn(1, 3);
switch model
    case 'gm', phi = exp(-dt / tau); s = sigma * sqrt(1 - phi^2);
    case 'rw', phi = 1;              s = sqrt(2 * sigma^2 / tau * dt);
    otherwise, error('unknown bias model %s', model);
end
w = s * randn(n, 3);
for k = 1:n
    b(k + 1, :) = phi * b(k, :) + w(k, :);
end
end
