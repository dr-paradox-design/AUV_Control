function rv = qlog(q)
% QLOG  Rotation vector(s) of quaternion(s) q (rows), shortest rotation.
neg = q(:,1) < 0;
q(neg,:) = -q(neg,:);
w = q(:,1); v = q(:,2:4);
s = sqrt(sum(v.^2, 2));
ang = 2 * atan2(s, w);
k = 2 ./ max(w, 1e-12);
big = s > 1e-8;
k(big) = ang(big) ./ s(big);
rv = k .* v;
end
