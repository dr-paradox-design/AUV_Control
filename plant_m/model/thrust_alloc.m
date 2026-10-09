function u = thrust_alloc(tau, p, z)
% THRUST_ALLOC  Minimum-norm allocation u = B' (B B')^-1 tau (report 3.18 with z = 0).
% Optional z in R^8 adds a null-space component (I - B+ B) z, which does not change tau.
B = thrust_matrix(p);
Bp = B' / (B * B');
u = Bp * tau(:);
if nargin > 2 && ~isempty(z)
    u = u + (eye(size(B, 2)) - Bp * B) * z(:);
end
end
