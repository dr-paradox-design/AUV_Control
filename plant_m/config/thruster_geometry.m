function thr = thruster_geometry()
% THRUSTER_GEOMETRY  Positions r (3x8) and unit thrust directions e (3x8), body frame FRD.
% ASSUMED: an X-pattern of four horizontal thrusters at 45 degrees and four vertical ones,
% with BlueROV2-Heavy-like positions recalled from memory (UNVERIFIED). It is NOT the real
% vectoring pattern of the vehicle; it only has the right structure (rank 6, null space of
% dimension 2). Replace with the measured geometry before using B for anything but tests.
c = cos(pi / 4); s = sin(pi / 4);
% horizontal: front-right, front-left, rear-right, rear-left
rh = [ 0.156  0.156 -0.156 -0.156;
       0.111 -0.111  0.111 -0.111;
       0      0      0      0   ];
eh = [ c  c  c  c;
      -s  s  s -s;
       0  0  0  0];
% vertical (thrust along +z, down)
rv = [ 0.120  0.120 -0.120 -0.120;
       0.218 -0.218  0.218 -0.218;
       0      0      0      0   ];
ev = [0 0 0 0; 0 0 0 0; 1 1 1 1];
thr.r = [rh, rv];
thr.e = [eh, ev];
end
