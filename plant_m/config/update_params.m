function p = update_params(p)
% UPDATE_PARAMS  Recompute the derived entries after editing m, rg, rb or Ig.
p.W  = p.m * p.g;
p.Io = p.Ig - p.m * skewm(p.rg)^2;
end
