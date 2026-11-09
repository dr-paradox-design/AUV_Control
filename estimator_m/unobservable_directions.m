function N = unobservable_directions(x)
% UNOBSERVABLE_DIRECTIONS  Error-state directions (16 x 3) that DVL, depth and zero-velocity
% measurements cannot see:
% a rotation of the whole solution about NED down (yaw), and north / east translation.
% Earth rate makes yaw weakly observable in theory (gyrocompassing); a MEMS IMU cannot use that.
ez = [0; 0; 1];
yaw = [cross(ez, x.p); cross(ez, x.v); q2R(x.q)' * ez; zeros(7, 1)];
N = [yaw, [1; zeros(15, 1)], [0; 1; zeros(14, 1)]];
end
