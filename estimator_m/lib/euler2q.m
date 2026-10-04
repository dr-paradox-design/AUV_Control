function q = euler2q(phi, theta, psi)
% EULER2Q  q = qz(psi) (x) qy(theta) (x) qx(phi), so q2R(q) = Rzyx. Column inputs allowed.
z = zeros(size(phi));
qx = [cos(phi/2), sin(phi/2), z, z];
qy = [cos(theta/2), z, sin(theta/2), z];
qz = [cos(psi/2), z, z, sin(psi/2)];
q = qmul(qmul(qz, qy), qx);
end
