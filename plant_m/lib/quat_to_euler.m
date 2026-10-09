function Th = quat_to_euler(q)
% QUAT_TO_EULER  [phi; theta; psi] of a unit quaternion, via R(q) (zyx sequence).
R = quat_R(q);
Th = [atan2(R(3, 2), R(3, 3)); -asin(max(-1, min(1, R(3, 1)))); atan2(R(2, 1), R(1, 1))];
end
