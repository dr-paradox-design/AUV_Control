function dx = state_ominus(x, ref)
% STATE_OMINUS  dx such that ref (+) dx = x.
dx = zeros(16, 1);
dx(1:3) = x.p - ref.p;
dx(4:6) = x.v - ref.v;
dx(7:9) = qlog(qmul(qconj(ref.q), x.q))';
dx(10:12) = x.ba - ref.ba;
dx(13:15) = x.bg - ref.bg;
dx(16) = x.bd - ref.bd;
end
