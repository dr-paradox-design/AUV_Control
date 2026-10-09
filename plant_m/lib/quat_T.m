function T = quat_T(q)
% QUAT_T  T(q) with q_dot = 1/2 T(q) omega (report eq. 3.4); 4x3, no division.
eta = q(1); e = q(2:4);
T = [-e(:)'; eta * eye(3) + skewm(e)];
end
