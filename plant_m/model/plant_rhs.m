function [xd, aux] = plant_rhs(x, tau, vcn, dvcdz, p)
% PLANT_RHS  Quaternion plant, x = [pos(3); q(4); nu(6)] (13 states), report eqs. 3.5, 3.16.
%   pos_dot = R nu_1,   q_dot = 1/2 T(q) omega,   nu_dot from body_dynamics.
% The current v_c^n and its depth gradient are arguments: the plant never calls the
% current module (report 3.6).
q = x(4:7); nu = x(8:13);
R = quat_R(q);
[nudot, aux] = body_dynamics(nu, R, restoring(q, p), tau, vcn, dvcdz, p);
xd = [R * nu(1:3); 0.5 * quat_T(q) * nu(4:6); nudot];
end
