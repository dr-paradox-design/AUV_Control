function r = test_matrices(p)
% TEST_MATRICES  Structure of the model matrices, plus an independent Newton-Euler check of
% (3.8)-(3.9) and a check of C_A against the entries printed in the report (3.11).
S = 'matrices'; r = new_results();
rng(1);
nsamp = 20;
sym_err = 0; skewRB = 0; skewA = 0; energyRB = 0; energyA = 0; ne_err = 0; ca_err = 0;
for i = 1:nsamp
    nu = randn(6, 1); nur = randn(6, 1); nud = randn(6, 1);
    [MRB, CRB] = rigid_body(nu, p);
    [MA, CA] = added_mass(nur, p);
    sym_err = max([sym_err, max(max(abs(MRB - MRB'))), max(max(abs(MA - MA')))]);
    skewRB = max(skewRB, max(max(abs(CRB + CRB'))));
    skewA = max(skewA, max(max(abs(CA + CA'))));
    energyRB = max(energyRB, abs(nu' * CRB * nu));
    energyA = max(energyA, abs(nur' * CA * nur));
    ne_err = max(ne_err, max(abs(MRB * nud + CRB * nu - newton_euler(nu, nud, p))));
    ca_err = max(ca_err, max(max(abs(CA - ca_printed(nur, p)))));
end
M0 = rigid_body(zeros(6, 1), p); MA0 = added_mass(zeros(6, 1), p);
r(end+1) = mk(S, 'M_RB and M_A symmetric', 'check', sym_err, 1e-12);
r(end+1) = mk(S, 'M_RB, M_A and M = M_RB + M_A positive definite (-min eig < 0)', 'check', ...
              -min([eig(M0); eig(MA0); eig(M0 + MA0)]), 0);
r(end+1) = mk(S, 'C_RB skew-symmetric: max |C + C''|', 'check', skewRB, 1e-12);
r(end+1) = mk(S, 'C_A skew-symmetric: max |C + C''|', 'check', skewA, 1e-12);
r(end+1) = mk(S, 'nu'' C_RB nu = 0', 'check', energyRB, 1e-12);
r(end+1) = mk(S, 'nu_r'' C_A nu_r = 0', 'check', energyA, 1e-12);
r(end+1) = mk(S, 'M_RB nu_dot + C_RB nu equals independent Newton-Euler force and moment', 'check', ne_err, 1e-10);
r(end+1) = mk(S, 'C_A equals the matrix printed in report (3.11)', 'check', ca_err, 1e-14);

% rank and null space of the thruster matrix
B = thrust_matrix(p);
r(end+1) = mk(S, 'thruster matrix: rank 6 and null space of dimension 2', 'check', ...
              abs(rank(B) - 6) + abs(size(B, 2) - rank(B) - 2), 0.5);

% failure controls: the same checks given deliberately wrong matrices
nu = [0.7; -0.4; 0.5; 0.3; -0.6; 0.2]; nud = [0.1; 0.2; -0.3; 0.4; 0.1; -0.2];
[MRB, CRB] = rigid_body(nu, p);
CRBbad = CRB; CRBbad(1:3, 4:6) = -CRBbad(1:3, 4:6);                    % one block sign-flipped
r(end+1) = mk(S, 'control: C_RB with one block flipped, skew check', 'control', max(max(abs(CRBbad + CRBbad'))), 1e-12);
CRBbad = CRB; CRBbad(4:6, 4:6) = -p.m * skewm(skewm(nu(1:3)) * p.rg) - skewm(p.Io * nu(4:6));  % sign of m S(S(v) rg)
r(end+1) = mk(S, 'control: C_RB with the m S(S(v) rg) sign flipped, Newton-Euler check', 'control', ...
              max(abs(MRB * nud + CRBbad * nu - newton_euler(nu, nud, p))), 1e-10);
[~, CA] = added_mass(nu, p); CAbad = CA; CAbad(4:6, 1:3) = -CAbad(4:6, 1:3);
r(end+1) = mk(S, 'control: C_A with one block flipped, printed-matrix check', 'control', max(max(abs(CAbad - ca_printed(nu, p)))), 1e-14);
end


function f = newton_euler(nu, nud, p)
% Force and moment about the body origin from the rigid-body laws, written without M or C.
v = nu(1:3); w = nu(4:6); vd = nud(1:3); wd = nud(4:6); rg = p.rg;
a_g = vd + cross(wd, rg) + cross(w, v) + cross(w, cross(w, rg));       % CG acceleration, body frame
F = p.m * a_g;
Mo = p.Ig * wd + cross(w, p.Ig * w) + cross(rg, F);
f = [F; Mo];
end


function C = ca_printed(nur, p)
% C_A transcribed entry by entry from report (3.11), in terms of the derivatives X_udot etc.
Xu = -p.A(1); Yv = -p.A(2); Zw = -p.A(3); Kp = -p.A(4); Mq = -p.A(5); Nr = -p.A(6);
u = nur(1); v = nur(2); w = nur(3); pp = nur(4); q = nur(5); rr = nur(6);
C = [ 0,        0,        0,        0,       -Zw*w,     Yv*v;
      0,        0,        0,        Zw*w,     0,       -Xu*u;
      0,        0,        0,       -Yv*v,     Xu*u,     0;
      0,       -Zw*w,     Yv*v,     0,       -Nr*rr,    Mq*q;
      Zw*w,     0,       -Xu*u,     Nr*rr,    0,       -Kp*pp;
     -Yv*v,     Xu*u,     0,       -Mq*q,     Kp*pp,    0];
end
