function pn = neutral_params(p)
% NEUTRAL_PARAMS  Parameters for the closed-form and conservation cases: neutral buoyancy,
% rg = rb = 0, no damping, still water. Individual tests switch on the one term they need.
pn = p;
pn.rg = zeros(3, 1); pn.rb = zeros(3, 1);
pn.Dl = zeros(1, 6); pn.Dq = zeros(1, 6);
pn.env = struct('preset', 'still');
pn.include_nuc_dot = true;
pn = update_params(pn);
pn.B = pn.W;
end
