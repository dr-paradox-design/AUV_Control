function r = test_kinematics(p) %#ok<INUSD>
% TEST_KINEMATICS  Report eqs. 2.3, 2.4, 3.1-3.4 checked numerically: R(q) against the
% Euler-angle matrix, R_dot = R S(omega), q'T(q) = 0, det T_Theta = 1/cos(theta).
S = 'kinematics'; r = new_results();
rng(3);
eR = 0; eRd = 0; eT = 0; eN = 0; eD = 0; eOrth = 0;
h = 1e-6;
for i = 1:20
    Th = [0.9 * (rand - 0.5) * pi, 0.8 * (rand - 0.5) * pi, 2 * (rand - 0.5) * pi];
    q = euler_to_quat(Th(1), Th(2), Th(3)); R = quat_R(q); w = randn(3, 1);
    eR = max(eR, max(max(abs(R - Rzyx(Th(1), Th(2), Th(3))))));
    eOrth = max(eOrth, max(max(abs(R' * R - eye(3)))) + abs(det(R) - 1));
    qd = 0.5 * quat_T(q) * w;
    Rd = (quat_R(q + h * qd) - quat_R(q - h * qd)) / (2 * h);
    eRd = max(eRd, max(max(abs(Rd - R * skewm(w)))));
    eN = max(eN, abs(q' * quat_T(q) * w));
    eD = max(eD, abs(det(Tzyx(Th)) - 1 / cos(Th(2))));
end
r(end+1) = mk(S, 'R(q) equals the Euler-angle matrix (3.3 vs 2.4)', 'check', eR, 1e-12);
r(end+1) = mk(S, 'R orthonormal with det 1', 'check', eOrth, 1e-12);
r(end+1) = mk(S, 'R_dot = R S(omega) with q_dot = 1/2 T(q) omega (2.3, 3.4)', 'check', eRd, 1e-8);
r(end+1) = mk(S, 'q'' T(q) omega = 0, so the norm is preserved (3.4)', 'check', eN, 1e-14);
r(end+1) = mk(S, 'det T_Theta = 1/cos(theta) (3.1)', 'check', eD, 1e-12);
Th = [0.3 -0.4 1.2]; q = euler_to_quat(Th(1), Th(2), Th(3)); w = [0.2; -0.1; 0.3];
qd = 0.5 * quat_T(q) * w;
Rdbad = (quat_R(q + h * qd) - quat_R(q - h * qd)) / (2 * h);
r(end+1) = mk(S, 'control: R_dot compared with S(omega) R instead of R S(omega)', 'control', max(max(abs(Rdbad - skewm(w) * quat_R(q)))), 1e-8);
r(end+1) = mk(S, 'control: R(q) compared with its transpose', 'control', max(max(abs(quat_R(q)' - Rzyx(Th(1), Th(2), Th(3))))), 1e-12);
end
