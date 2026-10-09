function out = simulate(x0, tau_fun, p, dt, T)
% SIMULATE  Fixed-step RK4 of the quaternion plant (13 states). tau_fun is a handle t -> 6x1
% generalised force in the body frame. The current and its gradient are evaluated from
% p.env at the depth of every Runge-Kutta stage and passed to the plant. The quaternion is
% renormalised after each complete step and not at the intermediate stages (report 3.1.4).
n = round(T / dt);
t = (0:n)' * dt;
X = zeros(n + 1, 13);
X(1, :) = x0(:)';
f = @(x, tt) rhs_env(x, tau_fun(tt), p);
for k = 1:n
    xn = rk4(f, X(k, :)', t(k), dt);
    xn(4:7) = xn(4:7) / norm(xn(4:7));
    X(k + 1, :) = xn';
end
out.t = t; out.X = X;
end


function xd = rhs_env(x, tau, p)
[vc, dvc] = current_profile(x(3), p.env);
xd = plant_rhs(x, tau, vc, dvc, p);
end
