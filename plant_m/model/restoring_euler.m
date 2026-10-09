function g = restoring_euler(Th, p)
% RESTORING_EULER  g(eta) in the explicit Euler-angle form, written out component by
% component (not through R), so it is an independent derivation of restoring.m.
phi = Th(1); th = Th(2);
sf = sin(phi); cf = cos(phi); st = sin(th); ct = cos(th);
W = p.W; B = p.B;
xg = p.rg(1); yg = p.rg(2); zg = p.rg(3);
xb = p.rb(1); yb = p.rb(2); zb = p.rb(3);
g = [ (W - B) * st;
     -(W - B) * ct * sf;
     -(W - B) * ct * cf;
     -(yg * W - yb * B) * ct * cf + (zg * W - zb * B) * ct * sf;
      (zg * W - zb * B) * st + (xg * W - xb * B) * ct * cf;
     -(xg * W - xb * B) * ct * sf - (yg * W - yb * B) * st];
end
