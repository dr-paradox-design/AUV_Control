function xn = rk4(f, x, t, dt)
% RK4  One classical fourth-order Runge-Kutta step of x_dot = f(x, t).
k1 = f(x, t);
k2 = f(x + 0.5 * dt * k1, t + 0.5 * dt);
k3 = f(x + 0.5 * dt * k2, t + 0.5 * dt);
k4 = f(x + dt * k3, t + dt);
xn = x + dt / 6 * (k1 + 2 * k2 + 2 * k3 + k4);
end
