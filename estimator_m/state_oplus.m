function x = state_oplus(x, dx)
% STATE_OPLUS  x (+) dx (Sola eq. 282). State fields are columns; q is a row [w x y z].
% Error state (16): [dp(1:3) dv(4:6) dth(7:9) da_b(10:12) dw_b(13:15) db_d(16)].
x.p = x.p + dx(1:3);
x.v = x.v + dx(4:6);
x.q = qnormalize(qmul(x.q, qexp(dx(7:9)')));
x.ba = x.ba + dx(10:12);
x.bg = x.bg + dx(13:15);
x.bd = x.bd + dx(16);
end
