function T = Tzyx(Th)
% TZYX  Theta_dot = T(Theta) omega (report eq. 3.1). Singular at theta = +-90 deg.
sf = sin(Th(1)); cf = cos(Th(1)); tt = tan(Th(2)); ct = cos(Th(2));
T = [1, sf * tt, cf * tt;
     0, cf,     -sf;
     0, sf / ct, cf / ct];
end
