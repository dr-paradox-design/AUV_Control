function r = test_cross_check(p)
% TEST_CROSS_CHECK  The quaternion plant against the Euler-angle plant over a 20 s manoeuvre
% that stays away from gimbal lock. The two share body_dynamics.m; they differ in the
% rotation matrix, the kinematic matrix and the restoring force, which is what this checks.
S = 'cross-check'; r = new_results();
pe = p;
pe.B = pe.W + 2;                       % net positive buoyancy (Wu: B 114.8 N vs W 112.8 N) so the (W - B) terms act
pe.env = struct('preset', 'ekman', 'Winf', 0.05 + 0.02i, 'W0', 0.25 * exp(1i * 0.3), 'kd', 0.15, 'kv', 0.15);
tau = @(t) [6; 1.5 * sin(0.6 * t); 2 * sin(0.4 * t); 0.8 * sin(0.9 * t); 0.6 * sin(0.5 * t); 1.2 * sin(0.7 * t) + 0.3];
dt = 0.002; T = 20;
Th0 = [0.1; 0.2; 0.3]; nu0 = [0.2; 0.05; 0; 0; 0; 0]; pos0 = [0; 0; 4];
oq = simulate([pos0; euler_to_quat(Th0(1), Th0(2), Th0(3)); nu0], tau, pe, dt, T);
oe = simulate_euler([pos0; Th0; nu0], tau, pe, dt, T);
[eP, eV, eA, thmax] = compare(oq, oe);
r(end+1) = mk(S, 'position, quaternion vs Euler plant (max error, m)', 'check', eP, 1e-8);
r(end+1) = mk(S, 'body velocity, quaternion vs Euler plant (max error)', 'check', eV, 1e-8);
r(end+1) = mk(S, 'attitude, quaternion vs Euler plant (max error, rad)', 'check', eA, 1e-8);
r(end+1) = mk(S, 'manoeuvre stays away from gimbal lock (max |theta| - 60 deg, rad < 0)', 'check', thmax - pi / 3, 0);

Tbad = @(Th) Tzyx(Th) .* [1 -1 1; 1 1 1; 1 1 1];                  % one sign flipped in T_Theta
oe1 = simulate_euler([pos0; Th0; nu0], tau, pe, dt, T, Tbad, []);
[~, ~, eA1] = compare(oq, oe1);
r(end+1) = mk(S, 'control: Euler plant with one sign flipped in T_Theta (attitude error)', 'control', eA1, 1e-8);
gbad = @(Th, pp) -restoring_euler(Th, pp);
oe2 = simulate_euler([pos0; Th0; nu0], tau, pe, dt, T, [], gbad);
[eP2, ~, ~] = compare(oq, oe2);
r(end+1) = mk(S, 'control: Euler plant with the restoring force sign flipped (position error)', 'control', eP2, 1e-8);
end


function [eP, eV, eA, thmax] = compare(oq, oe)
eP = max(max(abs(oq.X(:, 1:3) - oe.X(:, 1:3))));
eV = max(max(abs(oq.X(:, 8:13) - oe.X(:, 7:12))));
n = size(oq.X, 1); d = zeros(n, 3);
for k = 1:n
    d(k, :) = (quat_to_euler(oq.X(k, 4:7)') - oe.X(k, 4:6)')';
end
eA = max(max(abs(atan2(sin(d), cos(d)))));
thmax = max(abs(oe.X(:, 5)));
end
