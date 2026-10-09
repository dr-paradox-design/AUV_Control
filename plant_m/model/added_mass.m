function [MA, CA] = added_mass(nur, p)
% ADDED_MASS  Diagonal added mass (report 3.10) and its skew-symmetric Coriolis matrix
% built from the water-relative velocity (report 3.11):
%   CA = [0, -S(MA11 v_r); -S(MA11 v_r), -S(MA22 w_r)].
MA = diag(p.A);
a = MA(1:3, 1:3) * nur(1:3);
b = MA(4:6, 4:6) * nur(4:6);
CA = [zeros(3), -skewm(a);
      -skewm(a), -skewm(b)];
end
