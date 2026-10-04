function [xn, w, a, R] = eskf_nominal(x, dtheta, dvel, dt, cfg, bias_model)
% ESKF_NOMINAL  Nominal mechanisation over one IMU interval (log 12.3, 12.8).
w = dtheta(:) / dt - x.bg;                  % w_ib estimate (body)
a = dvel(:) / dt - x.ba;                    % specific force estimate (body)
R = q2R(x.q);
acc_n = R * a + cfg.g - 2 * cross(cfg.w_ie, x.v);
xn = x;
xn.p = x.p + x.v * dt + 0.5 * acc_n * dt^2;
xn.v = x.v + acc_n * dt;
xn.q = qnormalize(qmul(qmul(qexp(-cfg.w_ie' * dt), x.q), qexp(w' * dt)));
if strcmp(bias_model, 'gm')
    phi = exp(-dt / cfg.imu.bias_tau);
    xn.ba = phi * x.ba;
    xn.bg = phi * x.bg;
end
end
