function Q = eskf_Q(dt, cfg, bias_model)
% ESKF_Q  Discrete process noise from datasheet densities (Sola eqs. 261-264).
I = cfg.imu; I3 = eye(3);
Q = zeros(16);
Q(4:6, 4:6) = I.accel_nd^2 * dt * I3;
Q(7:9, 7:9) = I.gyro_nd^2 * dt * I3;
sig = [I.accel_bias_sigma, I.gyro_bias_sigma];
blk = {10:12, 13:15};
for j = 1:2
    if strcmp(bias_model, 'gm')
        phi = exp(-dt / I.bias_tau);
        Q(blk{j}, blk{j}) = sig(j)^2 * (1 - phi^2) * I3;
    else
        Q(blk{j}, blk{j}) = 2 * sig(j)^2 / I.bias_tau * dt * I3;
    end
end
Q(16, 16) = cfg.depth.bias_rw^2 * dt;
end
