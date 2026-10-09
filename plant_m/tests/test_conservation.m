function r = test_conservation(p)
% TEST_CONSERVATION  A free body in still water with C built from the total inertia must
% conserve kinetic energy, linear momentum and angular momentum (in {n}); with damping the
% energy must never increase (passivity).
S = 'conservation'; r = new_results();
pc = neutral_params(p); dt = 0.002; T = 20;
q0 = euler_to_quat(0.3, -0.2, 0.5); pos0 = [1; 2; 3];
nu0 = [0.5; -0.3; 0.2; 0.4; -0.2; 0.5];
x0 = [pos0; q0; nu0];
M11 = diag(pc.m + pc.A(1:3)); M22 = pc.Ig + diag(pc.A(4:6));
Mt = blkdiag(M11, M22);

o = simulate(x0, @(t) zeros(6, 1), pc, dt, T);
[E, P, H] = invariants(o.X, Mt, M11, M22);
eE = max(abs(E - E(1))) / E(1);
eP = max(sqrt(sum((P - P(1, :)).^2, 2))) / norm(P(1, :));
eH = max(sqrt(sum((H - H(1, :)).^2, 2))) / norm(H(1, :));
r(end+1) = mk(S, 'free body: kinetic energy conserved (relative drift over 20 s)', 'check', eE, 1e-8);
r(end+1) = mk(S, 'free body: linear momentum conserved (relative drift)', 'check', eP, 1e-8);
r(end+1) = mk(S, 'free body: angular momentum conserved (relative drift)', 'check', eH, 1e-8);

pd = pc; pd.Dq = p.Dq;                                  % damping switched back on
o2 = simulate(x0, @(t) zeros(6, 1), pd, dt, T);
E2 = invariants(o2.X, Mt, M11, M22);
r(end+1) = mk(S, 'damped body: energy never increases (max step increase)', 'check', max(diff(E2)), 1e-12);
r(end+1) = mk(S, 'damped body: energy strictly decreases overall (E_end/E_0 - 1 < 0)', 'check', E2(end) / E2(1) - 1, 0);
r(end+1) = mk(S, 'control: energy-conservation check applied to the damped run', 'control', max(abs(E2 - E2(1))) / E2(1), 1e-8);
end


function [E, P, H] = invariants(X, Mt, M11, M22)
n = size(X, 1);
E = zeros(n, 1); P = zeros(n, 3); H = zeros(n, 3);
for k = 1:n
    R = quat_R(X(k, 4:7)'); nu = X(k, 8:13)';
    E(k) = 0.5 * nu' * Mt * nu;
    Pn = R * (M11 * nu(1:3));
    P(k, :) = Pn';
    H(k, :) = (R * (M22 * nu(4:6)) + cross(X(k, 1:3)', Pn))';
end
end
