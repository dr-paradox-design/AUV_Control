function R = euler2R(phi, theta, psi)
% EULER2R  Fossen Rzyx = Rz(psi) Ry(theta) Rx(phi) (scalars).
cf = cos(phi); sf = sin(phi); ct = cos(theta); st = sin(theta); cp = cos(psi); sp = sin(psi);
R = [cp*ct, -sp*cf + cp*st*sf,  sp*sf + cp*cf*st;
     sp*ct,  cp*cf + sf*st*sp, -cp*sf + st*sp*cf;
     -st,    ct*sf,             ct*cf];
end
