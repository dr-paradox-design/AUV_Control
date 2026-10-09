function [xd, aux] = euler_plant_rhs(x, tau, vcn, dvcdz, p, Tfun, gfun)
% EULER_PLANT_RHS  Euler-angle plant, x = [pos(3); Theta(3); nu(6)] (12 states). Used only
% as a cross-check of the quaternion plant; it has its own rotation matrix (Rzyx), its own
% kinematic matrix (Tzyx) and its own restoring force (restoring_euler). Tfun and gfun can
% be replaced to build deliberately wrong variants for the failure controls.
if nargin < 6 || isempty(Tfun), Tfun = @Tzyx; end
if nargin < 7 || isempty(gfun), gfun = @restoring_euler; end
Th = x(4:6); nu = x(7:12);
R = Rzyx(Th(1), Th(2), Th(3));
[nudot, aux] = body_dynamics(nu, R, gfun(Th, p), tau, vcn, dvcdz, p);
xd = [R * nu(1:3); Tfun(Th) * nu(4:6); nudot];
end
