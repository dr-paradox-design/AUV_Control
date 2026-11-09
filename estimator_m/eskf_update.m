function [x, P, nis, accepted, lambda] = eskf_update(x, P, name, z, w_ib, cfg, gate, robust_k2, oc)
% ESKF_UPDATE  Joseph-form update, injection, reset G = I. Returns NIS (before any down-weighting).
% Optional gate: skip the update (accepted = false) when NIS > gate. Default Inf (no gating).
% Optional robust_k2: when NIS > robust_k2, inflate the measurement noise by lambda > 1 so that the
% NIS becomes robust_k2, then update. Outliers keep at most the pull of a borderline measurement and
% no data is discarded. lambda = 1 when not inflated. Default Inf (off).
% Optional oc (observability constraint, after Huang et al. 2010): for dvl, depth and zero_vel, project
% H so that H N = 0 for N = [rotation about NED down, north and east translation]. These measurements
% then cannot reduce uncertainty in directions they do not observe. Default false.
if nargin < 7 || isempty(gate), gate = Inf; end
if nargin < 8 || isempty(robust_k2), robust_k2 = Inf; end
if nargin < 9 || isempty(oc), oc = false; end
[h, H, Rm] = eskf_meas(name, x, w_ib, cfg);
if oc && any(strcmp(name, {'dvl', 'depth', 'zero_vel'}))
    N = unobservable_directions(x);
    H = H - (H * N) / (N' * N) * N';
end
r = z(:) - h;
if strcmp(name, 'heading'), r = wrap_pi(r); end
HPH = H * P * H';
S = HPH + Rm;
nis = r' * (S \ r);
lambda = 1;
accepted = nis <= gate;
if ~accepted, return; end
if nis > robust_k2
    lambda = inflation(r, HPH, Rm, robust_k2);
    Rm = lambda * Rm;
    S = HPH + Rm;
end
K = (S \ (H * P))';
dx = K * r;
IKH = eye(16) - K * H;
P = IKH * P * IKH' + K * Rm * K';
P = 0.5 * (P + P');
x = state_oplus(x, dx);
end


function lam = inflation(r, HPH, Rm, k2)
% Smallest lambda with r' (HPH + lambda Rm)^-1 r = k2; NIS falls monotonically in lambda.
lo = 1; hi = 2;
while r' * ((HPH + hi * Rm) \ r) > k2, lo = hi; hi = 2 * hi; end
for i = 1:40
    mid = sqrt(lo * hi);
    if r' * ((HPH + mid * Rm) \ r) > k2, lo = mid; else, hi = mid; end
end
lam = hi;
end
