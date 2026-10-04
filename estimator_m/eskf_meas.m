function [h, H, Rm] = eskf_meas(name, x, w_ib, cfg)
% ESKF_MEAS  Measurement prediction h, Jacobian H (1x16 or 3x16) and noise Rm.
% Lever-arm terms use w_nb = w_ib - R' w_ie (log 12.4, refined in 12.10).
R = q2R(x.q);
I3 = eye(3);
wie_b = R' * cfg.w_ie;
w_nb = w_ib - wie_b;
switch name
    case 'dvl'
        l = cfg.dvl.lever; Rbd = cfg.dvl.R_bd;
        h = Rbd' * (R' * x.v + cross(w_nb, l));
        H = zeros(3, 16);
        H(:, 4:6) = Rbd' * R';
        H(:, 7:9) = Rbd' * (skew3(R' * x.v) + skew3(l) * skew3(wie_b));
        H(:, 13:15) = Rbd' * skew3(l);
        Rm = cfg.dvl.sigma^2 * I3;
    case 'depth'
        l = cfg.depth.lever;
        rl = R * l;
        h = x.p(3) + rl(3) + x.bd;
        H = zeros(1, 16);
        H(3) = 1;
        RS = R * skew3(l);
        H(7:9) = -RS(3, :);
        H(16) = 1;
        Rm = cfg.depth.sigma^2 + (cfg.depth.full_scale / cfg.depth.adc_counts)^2 / 12;
    case 'gnss_pos'
        l = cfg.gnss.lever;
        h = x.p + R * l;
        H = zeros(3, 16);
        H(:, 1:3) = I3;
        H(:, 7:9) = -R * skew3(l);
        Rm = diag(cfg.gnss.sigma_pos.^2);
    case 'gnss_vel'
        l = cfg.gnss.lever;
        h = x.v + R * cross(w_nb, l);
        H = zeros(3, 16);
        H(:, 4:6) = I3;
        H(:, 7:9) = -R * skew3(cross(w_nb, l)) + R * skew3(l) * skew3(wie_b);
        H(:, 13:15) = R * skew3(l);
        Rm = cfg.gnss.sigma_vel^2 * I3;
    case 'heading'
        % yaw (Fossen zyx); exact small-angle Jacobian (log 12.10):
        % d psi = [tan(theta) cos(psi), tan(theta) sin(psi), 1] R dth
        psi = atan2(R(2,1), R(1,1));
        theta = -asin(max(-1, min(1, R(3,1))));
        h = psi;
        H = zeros(1, 16);
        H(7:9) = [tan(theta) * cos(psi), tan(theta) * sin(psi), 1] * R;
        Rm = cfg.heading.sigma^2;
    otherwise
        error('unknown measurement %s', name);
end
end
