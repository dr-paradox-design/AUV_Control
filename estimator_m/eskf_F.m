function F = eskf_F(R, w, a, dt, cfg, bias_model)
% ESKF_F  Error-state transition (Sola eq. 269, plus Coriolis and bias decay).
I3 = eye(3);
F = eye(16);
F(1:3, 4:6) = I3 * dt;
F(4:6, 4:6) = I3 - 2 * skew3(cfg.w_ie) * dt;
F(4:6, 7:9) = -R * skew3(a) * dt;
F(4:6, 10:12) = -R * dt;
F(7:9, 7:9) = q2R(qexp(w' * dt))';
F(7:9, 13:15) = -I3 * dt;
if strcmp(bias_model, 'gm')
    F(10:15, 10:15) = exp(-dt / cfg.imu.bias_tau) * eye(6);
end
end
