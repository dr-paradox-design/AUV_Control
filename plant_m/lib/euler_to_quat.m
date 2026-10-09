function q = euler_to_quat(phi, theta, psi)
% EULER_TO_QUAT  Unit quaternion [eta; eps] (Hamilton, scalar first) of Rz(psi) Ry(theta) Rx(phi).
cr = cos(phi/2); sr = sin(phi/2); cp = cos(theta/2); sp = sin(theta/2); cy = cos(psi/2); sy = sin(psi/2);
q = [cr*cp*cy + sr*sp*sy;
     sr*cp*cy - cr*sp*sy;
     cr*sp*cy + sr*cp*sy;
     cr*cp*sy - sr*sp*cy];
end
