function [x, P, nis] = eskf_update(x, P, name, z, w_ib, cfg)
% ESKF_UPDATE  Joseph-form update, injection, reset G = I. Returns NIS.
[h, H, Rm] = eskf_meas(name, x, w_ib, cfg);
r = z(:) - h;
if strcmp(name, 'heading'), r = wrap_pi(r); end
S = H * P * H' + Rm;
K = (S \ (H * P))';
dx = K * r;
IKH = eye(16) - K * H;
P = IKH * P * IKH' + K * Rm * K';
P = 0.5 * (P + P');
x = state_oplus(x, dx);
nis = r' * (S \ r);
end
