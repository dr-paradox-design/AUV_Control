function r = test_analytic(p)
% TEST_ANALYTIC  With one damping term active the 6-DOF code must reproduce single-axis
% closed-form solutions, and every other state must stay at its initial value.
S = 'analytic'; r = new_results();
pn = neutral_params(p); dt = 0.002;
x0 = @(q0, nu0) [zeros(3, 1); q0(:); nu0(:)];
qid = [1; 0; 0; 0];
mu = pn.m + pn.A(1); mw = pn.m + pn.A(3);

% (a) linear surge drag: u(t) = u0 exp(-Xu t / mu)
pa = pn; pa.Dl(1) = 3; u0 = 1.5;
o = simulate(x0(qid, [u0; 0; 0; 0; 0; 0]), @(t) zeros(6, 1), pa, dt, 10);
r(end+1) = mk(S, 'linear surge drag: u(t)', 'check', max(abs(o.X(:, 8) - u0 * exp(-pa.Dl(1) * o.t / mu))), 1e-9);
r(end+1) = mk(S, 'linear surge drag: all other states unchanged', 'check', off_axis(o.X, [1 8]), 1e-12);
r(end+1) = mk(S, 'control: linear surge drag with added mass forgotten', 'control', max(abs(o.X(:, 8) - u0 * exp(-pa.Dl(1) * o.t / pn.m))), 1e-9);

% (b) thrust with quadratic drag: u(t) = U tanh(t sqrt(T k) / mu)
pb = pn; pb.Dq(1) = 18.18; Tn = 40; k = pb.Dq(1);
o = simulate(x0(qid, zeros(6, 1)), @(t) [Tn; 0; 0; 0; 0; 0], pb, dt, 15);
U = sqrt(Tn / k); a = sqrt(Tn * k) / mu;
r(end+1) = mk(S, 'thrust and quadratic drag: u(t)', 'check', max(abs(o.X(:, 8) - U * tanh(a * o.t))), 1e-9);
r(end+1) = mk(S, 'thrust and quadratic drag: all other states unchanged', 'check', off_axis(o.X, [1 8]), 1e-12);
r(end+1) = mk(S, 'control: u(t) with added mass forgotten', 'control', max(abs(o.X(:, 8) - U * tanh(sqrt(Tn * k) / pn.m * o.t))), 1e-9);

% (c) terminal velocity U = sqrt(T / k)
r(end+1) = mk(S, 'terminal velocity sqrt(T/k)', 'check', abs(o.X(end, 8) - U), 1e-9);
r(end+1) = mk(S, 'control: terminal velocity with k doubled', 'control', abs(o.X(end, 8) - sqrt(Tn / (2 * k))), 1e-9);

% (d) linear heave drag: w(t) = w0 exp(-Zw t / mw)
pd = pn; pd.Dl(3) = 5; w0 = 0.8;
o = simulate(x0(qid, [0; 0; w0; 0; 0; 0]), @(t) zeros(6, 1), pd, dt, 10);
r(end+1) = mk(S, 'linear heave drag: w(t)', 'check', max(abs(o.X(:, 10) - w0 * exp(-pd.Dl(3) * o.t / mw))), 1e-9);
r(end+1) = mk(S, 'linear heave drag: all other states unchanged', 'check', off_axis(o.X, [3 10]), 1e-12);
r(end+1) = mk(S, 'control: w(t) with added mass forgotten', 'control', max(abs(o.X(:, 10) - w0 * exp(-pd.Dl(3) * o.t / pn.m))), 1e-9);

% (e) undamped roll about a stable metacentric height: T = 2 pi sqrt((Ix + |Kpdot|) / (W BG))
pe = pn; BG = 2.26 / pn.W; pe.rb = [0; 0; -BG];          % buoyancy above the CG, rg = 0 keeps M diagonal
pe = update_params(pe);
phi0 = 0.03;
o = simulate(x0(euler_to_quat(phi0, 0, 0), zeros(6, 1)), @(t) zeros(6, 1), pe, dt, 10);
phi = 2 * atan2(o.X(:, 5), o.X(:, 4));
idx = find(phi(1:end-1) < 0 & phi(2:end) >= 0);
tc = o.t(idx) - phi(idx) .* (o.t(idx + 1) - o.t(idx)) ./ (phi(idx + 1) - phi(idx));
Tsim = tc(2) - tc(1);
Tan = 2 * pi * sqrt((pe.Ig(1) + pe.A(4)) / (pe.W * BG));
Twrong = 2 * pi * sqrt(pe.Ig(1) / (pe.W * BG));
r(end+1) = mk(S, 'undamped roll: period, relative error', 'check', abs(Tsim / Tan - 1), 1e-3);
r(end+1) = mk(S, 'undamped roll: all other states unchanged', 'check', off_axis(o.X, [4 5 11]), 1e-12);
r(end+1) = mk(S, 'control: roll period with added inertia forgotten', 'control', abs(Twrong / Tan - 1), 1e-3);
end


function e = off_axis(X, keep)
% Largest deviation of the states that should not move: zero for all columns except keep,
% and 1 for the quaternion scalar part (column 4) unless it is in keep.
Y = X; Y(:, 4) = Y(:, 4) - 1;
Y(:, keep) = 0;
e = max(abs(Y(:)));
end
