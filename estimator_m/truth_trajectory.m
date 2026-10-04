function tr = truth_trajectory(cfg, m)
% TRUTH_TRAJECTORY  Designed truth trajectory with noise-free IMU increments.
%
% The trajectory is analytic (speed, heading, depth, roll, pitch with closed-form
% derivatives). Gyro increments are exact from the truth attitude and Earth rate;
% velocity increments are Simpson integrals of the true specific force.
% Rows are time samples at the IMU rate.
if nargin < 2, m = 4; end                         % Simpson sub-intervals (even)
dt = 1 / cfg.imu.rate_hz;
n = round(cfg.sc.duration / dt);
tr.dt = dt; tr.n = n;
tr.t = (0:n)' * dt;
tr.w_ie = cfg.w_ie; tr.g = cfg.g;

% fine grid for Simpson integration
tf = (0:n*m)' * (dt / m);
kf = truth_kinematics(tf, cfg);
f_n = kf.a - cfg.g' + 2 * cross(repmat(cfg.w_ie', numel(tf), 1), kf.v, 2);
wts = ones(1, m + 1); wts(2:2:m) = 4; wts(3:2:m-1) = 2; wts = wts * (dt / m) / 3;
idx = (0:n-1)' * m + (1:m+1);                     % n-by-(m+1) indices into the fine grid
int_f = zeros(n, 3); int_v = zeros(n, 3);
for c = 1:3
    fc = f_n(:, c); vc = kf.v(:, c);
    int_f(:, c) = fc(idx) * wts';
    int_v(:, c) = vc(idx) * wts';
end

k = truth_kinematics(tr.t, cfg);
tr.v = k.v;
tr.w_nb = k.w_nb;
tr.euler = [k.phi, k.theta, k.psi];
tr.q = euler2q(k.phi, k.theta, k.psi);
tr.p = [zeros(1, 3); cumsum(int_v, 1)];
tr.pz_check = tr.p(:, 3);
tr.p(:, 3) = k.depth;                            % analytic depth

% noise-free increments over [t_k, t_k+1]
dq_e = qexp(cfg.w_ie' * dt);
tr.dtheta = qlog(qmul(qmul(qconj(tr.q(1:n, :)), repmat(dq_e, n, 1)), tr.q(2:n+1, :)));
tr.dvel = zeros(n, 3);
for i = 1:n
    tr.dvel(i, :) = (euler2R(k.phi(i), k.theta(i), k.psi(i))' * int_f(i, :)')';
end
end


function k = truth_kinematics(t, cfg)
P = truth_profiles(t, cfg.sc);
c = cos(P.psi); s = sin(P.psi);
k.v = [P.U .* c, P.U .* s, P.dd];
k.a = [P.dU .* c - P.U .* P.dpsi .* s, P.dU .* s + P.U .* P.dpsi .* c, P.ddd];
phi = P.phi; th = P.theta;
k.w_nb = [P.dphi - P.dpsi .* sin(th), ...
          P.dtheta .* cos(phi) + P.dpsi .* cos(th) .* sin(phi), ...
          -P.dtheta .* sin(phi) + P.dpsi .* cos(th) .* cos(phi)];   % Fossen zyx body rates
k.phi = phi; k.theta = th; k.psi = P.psi; k.depth = P.d;
end


function P = truth_profiles(t, sc)
[s, ds] = smoothstep((t - 10) / 30);
P.U = sc.surge * s; P.dU = sc.surge * ds / 30;
P.psi = zeros(size(t)); P.dpsi = P.psi; L = sc.turn_len;
for i = 1:numel(sc.turn_times)
    [s, ds] = turn((t - sc.turn_times(i)) / L);
    P.psi = P.psi + sc.turn_signs(i) * pi * s;
    P.dpsi = P.dpsi + sc.turn_signs(i) * pi * ds / L;
end
D = sc.dive_depth;
[s1, ds1, dds1] = smoothstep((t - sc.dive_start) / sc.dive_len);
[s2, ds2, dds2] = smoothstep((t - sc.ascent_start) / sc.ascent_len);
P.d = D * (s1 - s2);
P.dd = D * (ds1 / sc.dive_len - ds2 / sc.ascent_len);
P.ddd = D * (dds1 / sc.dive_len^2 - dds2 / sc.ascent_len^2);
P.phi = 0.02 * sin(0.3 * t);   P.dphi = 0.02 * 0.3 * cos(0.3 * t);
P.theta = 0.03 * sin(0.21 * t); P.dtheta = 0.03 * 0.21 * cos(0.21 * t);
end


function [s, ds, dds] = smoothstep(x)
% Quintic 0->1 on [0, 1], zero 1st and 2nd derivatives at both ends.
x = min(max(x, 0), 1);
s = x.^3 .* (10 - 15 * x + 6 * x.^2);
ds = 30 * x.^2 .* (1 - x).^2;
dds = 60 * x .* (1 - x) .* (1 - 2 * x);
end


function [s, ds] = turn(x)
% 0->1 on [0, 1] with derivative 1 - cos(2 pi x).
inside = x > 0 & x < 1;
xc = min(max(x, 0), 1);
s = xc - sin(2 * pi * xc) / (2 * pi);
ds = (1 - cos(2 * pi * xc)) .* inside;
end
