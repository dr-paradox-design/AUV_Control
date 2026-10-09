function [MRB, CRB] = rigid_body(nu, p)
% RIGID_BODY  Rigid-body inertia and Coriolis-centripetal matrices about the body origin
% (report eqs. 3.8, 3.9, as printed). CRB is skew-symmetric, so nu' CRB nu = 0.
v = nu(1:3); w = nu(4:6); m = p.m; rg = p.rg;
MRB = [m * eye(3),  -m * skewm(rg);
       m * skewm(rg), p.Io];
X = -m * skewm(v) - m * skewm(skewm(w) * rg);
CRB = [zeros(3), X;
       X,        m * skewm(skewm(v) * rg) - skewm(p.Io * w)];
end
