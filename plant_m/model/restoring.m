function g = restoring(q, p)
% RESTORING  g(eta) from the quaternion (report eq. 3.13):
%   fg = R' [0 0 W],  fb = -R' [0 0 B],  g = -[fg + fb; rg x fg + rb x fb].
R  = quat_R(q);
fg = R' * [0; 0; p.W];
fb = -R' * [0; 0; p.B];
g  = -[fg + fb; cross(p.rg, fg) + cross(p.rb, fb)];
end
