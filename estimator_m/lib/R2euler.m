function [phi, theta, psi] = R2euler(R)
phi = atan2(R(3,2), R(3,3));
theta = -asin(max(-1, min(1, R(3,1))));
psi = atan2(R(2,1), R(1,1));
end
