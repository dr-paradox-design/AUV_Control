function out = simulate_euler(x0, tau_fun, p, dt, T, Tfun, gfun)
% SIMULATE_EULER  Fixed-step RK4 of the Euler-angle plant (12 states), same environment
% handling as simulate.m. Tfun and gfun are optional overrides for failure controls.
if nargin < 6, Tfun = []; end
if nargin < 7, gfun = []; end
n = round(T / dt);
t = (0:n)' * dt;
X = zeros(n + 1, 12);
X(1, :) = x0(:)';
f = @(x, tt) rhs_env(x, tau_fun(tt), p, Tfun, gfun);
for k = 1:n
    X(k + 1, :) = rk4(f, X(k, :)', t(k), dt)';
end
out.t = t; out.X = X;
end


function xd = rhs_env(x, tau, p, Tfun, gfun)
[vc, dvc] = current_profile(x(3), p.env);
xd = euler_plant_rhs(x, tau, vc, dvc, p, Tfun, gfun);
end
