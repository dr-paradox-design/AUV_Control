function S = skewm(a)
% SKEWM  S(a) with S(a) b = a x b (report eq. 2.3).
S = [0, -a(3), a(2); a(3), 0, -a(1); -a(2), a(1), 0];
end
