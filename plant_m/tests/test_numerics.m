function r = test_numerics(p)
% TEST_NUMERICS  RK4 against a tight-tolerance ode45 reference: observed order from step
% halving (report 5.4), and the quaternion norm after renormalisation each step.
%
% History, kept on purpose: the first version of this test used the quadratic damping of the
% plant and failed its pre-set window (observed orders 4.58 and 2.29). A diagnosis showed the
% ode45 reference was fine (settings agree to 1e-12) and that the damping term |nu| nu has a
% kink wherever a velocity component changes sign, which this manoeuvre does. The order check
% therefore runs on a smooth right-hand side (linear damping), with the window UNCHANGED, and
% the quadratic-damping orders are recorded below as an 'info' row, not hidden.
S = 'numerics'; r = new_results();
env = struct('preset', 'ekman', 'Winf', 0.05 + 0.02i, 'W0', 0.25 * exp(1i * 0.3), 'kd', 0.15, 'kv', 0.15);
tau = @(t) [6; 1.5 * sin(0.6 * t); 2 * sin(0.4 * t); 0.8 * sin(0.9 * t); 0.6 * sin(0.5 * t); 1.2 * sin(0.7 * t) + 0.3];
x0 = [0; 0; 4; euler_to_quat(0.1, 0.2, 0.3); 0.2; 0.05; 0; 0; 0; 0];
T = 5; hs = [0.05 0.025 0.0125];

% smooth right-hand side: linear damping only
ps = p; ps.env = env; ps.Dl = [5 10 15 1 1 1]; ps.Dq = zeros(1, 6);
[e, xref] = errors_vs_ode45(x0, tau, ps, T, hs);
p1 = log2(e(1) / e(2)); p2 = log2(e(2) / e(3));
fprintf('  smooth RHS, RK4 errors vs ode45: %.3e %.3e %.3e   observed order %.3f, %.3f\n', e, p1, p2);
r(end+1) = mk(S, 'RK4 observed order, smooth RHS, h = 0.05 -> 0.025 (|order - 4|)', 'check', abs(p1 - 4), 0.5);
r(end+1) = mk(S, 'RK4 observed order, smooth RHS, h = 0.025 -> 0.0125 (|order - 4|)', 'check', abs(p2 - 4), 0.5);

% control: the same harness on forward Euler must NOT look fourth order
f = @(x, t) rhs_env(x, tau(t), ps);
ee = zeros(1, 2); hh = [0.01 0.005];
for i = 1:2
    n = round(T / hh(i)); x = x0;
    for k = 1:n, x = x + hh(i) * f(x, (k - 1) * hh(i)); end
    ee(i) = max(abs(x - xref));
end
pe1 = log2(ee(1) / ee(2));
fprintf('  forward Euler errors: %.3e %.3e   observed order %.3f\n', ee, pe1);
r(end+1) = mk(S, 'control: forward Euler through the same order check', 'control', abs(pe1 - 4), 0.5);

% recorded observation: the plant's own quadratic damping, same manoeuvre
pq = p; pq.env = env;
[eq] = errors_vs_ode45(x0, tau, pq, T, [hs, hs(3) / 2, hs(3) / 4]);
oq = log2(eq(1:end-1) ./ eq(2:end));
fprintf('  quadratic damping, observed orders: %s\n', sprintf('%.2f ', oq));
r(end+1) = mk(S, 'info: quadratic damping, min observed order over 4 halvings (|nu| nu kink at sign changes)', 'info', min(oq), NaN);
r(end+1) = mk(S, 'info: quadratic damping, max observed order over 4 halvings', 'info', max(oq), NaN);

% quaternion norm after renormalisation at the end of every step
o = simulate(x0, tau, pq, 0.002, 30);
nq = sqrt(sum(o.X(:, 4:7).^2, 2));
r(end+1) = mk(S, 'quaternion norm stays 1 over 30 s (max | |q| - 1 |)', 'check', max(abs(nq - 1)), 1e-14);
end


function [e, xref] = errors_vs_ode45(x0, tau, p, T, hs)
f = @(x, t) rhs_env(x, tau(t), p);
[~, Y] = ode45(@(t, x) f(x, t), [0 T], x0, odeset('RelTol', 1e-13, 'AbsTol', 1e-15));
xref = Y(end, :)';
e = zeros(1, numel(hs));
for i = 1:numel(hs)
    o = simulate(x0, tau, p, hs(i), T);
    e(i) = max(abs(o.X(end, :)' - xref));
end
end


function xd = rhs_env(x, tau, p)
[vc, dvc] = current_profile(x(3), p.env);
xd = plant_rhs(x, tau, vc, dvc, p);
end
