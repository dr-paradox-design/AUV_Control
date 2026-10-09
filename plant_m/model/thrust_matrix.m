function B = thrust_matrix(p)
% THRUST_MATRIX  Thruster configuration matrix B in R^{6x8}, column i = [e_i; r_i x e_i]
% (report eq. 3.17). tau = B u.
n = size(p.thr.r, 2);
B = zeros(6, n);
for i = 1:n
    B(:, i) = [p.thr.e(:, i); cross(p.thr.r(:, i), p.thr.e(:, i))];
end
end
