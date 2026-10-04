function S = skew3(a)
% SKEW3  [a]x such that skew3(a)*b = cross(a, b).
S = [0, -a(3), a(2); a(3), 0, -a(1); -a(2), a(1), 0];
end
