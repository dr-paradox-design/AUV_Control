function R = quat_R(q)
% QUAT_R  Rotation matrix {b} -> {n} of a unit quaternion q = [eta; eps] (Hamilton,
% scalar first), R = I + 2 eta S(eps) + 2 S(eps)^2 (report eq. 3.3).
eta = q(1); e = q(2:4);
Se = skewm(e);
R = eye(3) + 2 * eta * Se + 2 * Se * Se;
end
