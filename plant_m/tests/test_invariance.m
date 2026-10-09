function [r, info] = test_invariance(p)
% TEST_INVARIANCE  Physics must not depend on arbitrary choices of reference.
%  (a) rotating the initial heading rotates the whole trajectory by the same angle;
%  (b) hydrodynamic forces depend only on nu_r, so a run in a uniform current equals the
%      still-water run carried along by the current: p_c(t) = p_0(t) + v_c t  (report 5.3);
%  (c) the analytic nu_c_dot equals a finite difference of nu_c(t) = R' v_c(z) along a
%      trajectory through a depth-varying current (report 5.5 and 3.22).
S = 'invariance'; r = new_results(); info = struct();
dt = 0.002; T = 30; tau = manoeuvre();

% (a) yaw rotation, still water, restoring and damping on
alpha = 0.7;
nu0 = [0.3; 0; 0; 0; 0; 0];
oA = simulate([0; 0; 0; 1; 0; 0; 0; nu0], tau, p, dt, T);
oB = simulate([0; 0; 0; cos(alpha/2); 0; 0; sin(alpha/2); nu0], tau, p, dt, T);
Rz = @(a) [cos(a) -sin(a) 0; sin(a) cos(a) 0; 0 0 1];
eP = max(max(abs(oB.X(:, 1:3) - (Rz(alpha) * oA.X(:, 1:3)')')));
eV = max(max(abs(oB.X(:, 8:13) - oA.X(:, 8:13))));
eWrong = max(max(abs(oB.X(:, 1:3) - (Rz(alpha + 0.1) * oA.X(:, 1:3)')')));
r(end+1) = mk(S, 'yaw rotation: positions equal the rotated still trajectory (max error, m)', 'check', eP, 1e-9);
r(end+1) = mk(S, 'yaw rotation: body-frame velocities identical', 'check', eV, 1e-9);
r(end+1) = mk(S, 'control: compared with a trajectory rotated by the wrong angle', 'control', eWrong, 1e-9);

% (b) Galilean invariance with a manoeuvre that turns
vc = [0.3; -0.2; 0];
q0 = euler_to_quat(0, 0, 0.4); R0 = quat_R(q0);
nur0 = [0.3; 0; 0; 0; 0; 0.05];
x_still = [0; 0; 0; q0; nur0];
x_curr = [0; 0; 0; q0; nur0 + [R0' * vc; 0; 0; 0]];
p0 = p; p0.env = struct('preset', 'still');
pc = p; pc.env = struct('preset', 'uniform', 'vn', vc(1), 've', vc(2));
o0 = simulate(x_still, tau, p0, dt, T);
oc = simulate(x_curr, tau, pc, dt, T);
eG = max(sqrt(sum((oc.X(:, 1:3) - o0.X(:, 1:3) - o0.t * vc').^2, 2)));
pbad = pc; pbad.include_nuc_dot = false;                       % the omission found in the report
ob = simulate(x_curr, tau, pbad, dt, T);
eGbad = max(sqrt(sum((ob.X(:, 1:3) - o0.X(:, 1:3) - o0.t * vc').^2, 2)));
r(end+1) = mk(S, 'Galilean invariance in a uniform current (max position error, m)', 'check', eG, 1e-9);
r(end+1) = mk(S, 'control: same check with the M_A nu_c_dot term dropped', 'control', eGbad, 1e-9);
info.o0 = o0; info.oc = oc; info.ob = ob; info.vc = vc; info.eG = eG; info.eGbad = eGbad;

% (c) nu_c_dot against a finite difference, through an Ekman profile
pe = p;
pe.env = struct('preset', 'ekman', 'Winf', 0.05 + 0.02i, 'W0', 0.25 * exp(1i * 0.3), 'kd', 0.15, 'kv', 0.15);
tau2 = @(t) [4; 0; 3 * sin(0.5 * t); 0; 0.3 * sin(0.4 * t); 1.2 * sin(0.6 * t)];
o = simulate([0; 0; 3; euler_to_quat(0, 0, 0.3); zeros(6, 1)], tau2, pe, 0.002, 15);
n = size(o.X, 1); nuc = zeros(n, 3); an = zeros(n, 3); an_off = zeros(n, 3);
po = pe; po.include_nuc_dot = false;
for k = 1:n
    x = o.X(k, :)'; R = quat_R(x(4:7));
    [vcn, dv] = current_profile(x(3), pe.env);
    nuc(k, :) = (R' * vcn)';
    [~, aux] = plant_rhs(x, tau2(o.t(k)), vcn, dv, pe);  an(k, :) = aux.nuc_dot(1:3)';
    [~, aux] = plant_rhs(x, tau2(o.t(k)), vcn, dv, po);  an_off(k, :) = aux.nuc_dot(1:3)';
end
fd = (nuc(3:end, :) - nuc(1:end-2, :)) / (2 * 0.002);
r(end+1) = mk(S, 'nu_c_dot: analytic vs finite difference through an Ekman profile (max error, m/s^2)', 'check', max(max(abs(an(2:end-1, :) - fd))), 1e-5);
r(end+1) = mk(S, 'control: nu_c_dot switched off', 'control', max(max(abs(an_off(2:end-1, :) - fd))), 1e-5);
info.zmin = min(o.X(:, 3)); info.zmax = max(o.X(:, 3));
assert(info.zmin > 0, 'vehicle left the water in the nu_c_dot trajectory; the test is not valid');
end
