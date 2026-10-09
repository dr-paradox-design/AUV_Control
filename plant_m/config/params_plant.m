function p = params_plant()
% PARAMS_PLANT  Parameters of the 6-DOF plant. Every value is ASSUMED unless its comment
% says otherwise. None of them is a measurement of the manta-ray vehicle.
%
% Source tags used below:
%   [log]     value from the estimator log (user value, unconfirmed)
%   [report]  value stated in AUV_Midterm_Report.pdf
%   [recall]  BlueROV2 literature value recalled from memory, UNVERIFIED; the mid-term
%             report confirms only Xu|u| = -18.18
%   [assumed] chosen here for the tests
% Units SI. Body frame FRD. Damping and added-mass entries are stored as positive magnitudes.

p.g   = 9.80665;                       % [m/s^2]
p.rho = 1000;                          % [kg/m^3] fresh water, [assumed]

p.m = 12.0;                            % [kg] [log]
p.W = p.m * p.g;                       % [N] weight
p.B = p.W;                             % [N] neutrally buoyant, [assumed]

p.Ig = diag([0.16 0.16 0.16]);         % [kg m^2] inertia about the CG, [recall]
BG   = 2.26 / p.W;                     % [m] chosen so that W*BG = 2.26 N m, the maximum righting
                                       %     moment quoted in the report (section 5.5.2) for m = 12 kg
p.rg = [0; 0; BG];                     % [m] CG below the origin (z down), [report-derived]
p.rb = [0; 0; 0];                      % [m] buoyancy at the body origin, [assumed]
p.Io = p.Ig - p.m * skewm(p.rg)^2;     % parallel-axis theorem (report 3.8)

p.A  = [5.5 12.7 14.57 0.12 0.12 0.12];   % added mass |Xudot Yvdot Zwdot Kpdot Mqdot Nrdot|, [recall]
p.Dl = zeros(1, 6);                       % linear damping (report model has none), [assumed]
p.Dq = [18.18 21.66 36.99 1.55 1.55 1.55];% quadratic damping |Xu|u| ...|; Xu|u| = 18.18 [report], rest [recall]

p.Tt = 0.15;                           % [s] thruster lag [report]
p.thr = thruster_geometry();           % 8 thrusters, geometry [assumed], see thruster_geometry.m

p.include_nuc_dot = true;              % false reproduces the omission the report found (M_A * nu_c_dot)

p.env = struct('preset', 'still');     % current preset: still | uniform | shear | ekman
end
