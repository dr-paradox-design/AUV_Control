function R = Rzyx(phi, theta, psi)
% RZYX  Rz(psi) Ry(theta) Rx(phi), body -> NED (report eq. 2.4). Written out entry by entry
% so that it does not share code with quat_R.
cf = cos(phi); sf = sin(phi); ct = cos(theta); st = sin(theta); cp = cos(psi); sp = sin(psi);
R = [cp*ct, -sp*cf + cp*st*sf,  sp*sf + cp*cf*st;
     sp*ct,  cp*cf + sf*st*sp, -cp*sf + st*sp*cf;
     -st,    ct*sf,             ct*cf];
end
