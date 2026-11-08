function [x, P, nis, accepted] = eskf_update(x, P, name, z, w_ib, cfg, gate)
% ESKF_UPDATE  Joseph-form update, injection, reset G = I. Returns NIS.
% Optional gate: skip the update (accepted = false) when NIS > gate. Default Inf (no gating).
if nargin < 7 || isempty(gate), gate = Inf; end
[h, H, Rm] = eskf_meas(name, x, w_ib, cfg);
r = z(:) - h;
if strcmp(name, 'heading'), r = wrap_pi(r); end
S = H * P * H' + Rm;
nis = r' * (S \ r);
accepted = nis <= gate;
if ~accepted, return; end
K = (S \ (H * P))';
dx = K * r;
IKH = eye(16) - K * H;
P = IKH * P * IKH' + K * Rm * K';
P = 0.5 * (P + P');
x = state_oplus(x, dx);
end
