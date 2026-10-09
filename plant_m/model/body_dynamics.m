function [nudot, aux] = body_dynamics(nu, R, g, tau, vcn, dvcdz, p)
% BODY_DYNAMICS  Acceleration of the 6-DOF model in the body frame (report 3.14-3.16).
% Shared by the quaternion plant and the Euler-angle plant: they differ only in the
% kinematics and in how g(eta) is formed.
%
%   nu_c   = [R' v_c^n; 0],           nu_r = nu - nu_c
%   nu_c_dot = [-w x (R' v_c^n) + R' (dv_c/dz) zdot; 0]     (report 5.5 and 3.22, dv_c/dt = 0)
%   nu_dot = M^-1 [ tau - C_RB(nu) nu - C_A(nu_r) nu_r - D(nu_r) nu_r - g + M_A nu_c_dot ]
% With p.include_nuc_dot = false the last term is dropped: the "slowly varying current"
% simplification whose omission the report found by validation.
v = nu(1:3); w = nu(4:6);
nuc = [R' * vcn; zeros(3, 1)];
nur = nu - nuc;
zdot = R(3, :) * v;                                   % NED vertical velocity of the origin
if p.include_nuc_dot
    nuc_dot = [-cross(w, R' * vcn) + R' * (dvcdz * zdot); zeros(3, 1)];
else
    nuc_dot = zeros(6, 1);
end
[MRB, CRB] = rigid_body(nu, p);
[MA, CA]   = added_mass(nur, p);
D = damping(nur, p);
nudot = (MRB + MA) \ (tau(:) - CRB * nu - CA * nur - D * nur - g + MA * nuc_dot);
aux.nuc = nuc; aux.nur = nur; aux.nuc_dot = nuc_dot;
end
