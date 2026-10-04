function q = qexp(rv)
% QEXP  Quaternion of rotation vector(s) rv (rows, angle*axis).
ang = sqrt(sum(rv.^2, 2));
k = 0.5 - ang.^2 / 48;                 % series of sin(a/2)/a near zero
big = ang > 1e-8;
k(big) = sin(ang(big) / 2) ./ ang(big);
q = [cos(ang / 2), k .* rv];
end
