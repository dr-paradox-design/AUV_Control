function r = test_current(p)
% TEST_CURRENT  The current module: closed-form gradient identity (report 3.21) against
% finite differences, presets, far field and surface values.
S = 'current'; r = new_results();
env = struct('preset', 'ekman', 'Winf', 0.05 + 0.02i, 'W0', 0.25 * exp(1i * 0.3), 'kd', 0.15, 'kv', 0.15);
h = 1e-5; zs = [0.5 2 7 15];
eg = 0; ebad = 0; evert = 0;
for z = zs
    [~, g] = current_profile(z, env);
    vp = current_profile(z + h, env); vm = current_profile(z - h, env);
    fd = (vp - vm) / (2 * h);
    eg = max(eg, max(abs(g - fd)));
    evert = max(evert, abs(vp(3)));
    W = env.Winf + env.W0 * exp((-env.kd + 1i * env.kv) * z);
    gb = (-env.kd) * (W - env.Winf);                                   % gradient with the i kv factor left out
    ebad = max(ebad, max(abs([real(gb); imag(gb); 0] - fd)));
end
r(end+1) = mk(S, 'Ekman gradient identity vs finite difference (max error, m/s per m)', 'check', eg, 1e-8);
r(end+1) = mk(S, 'control: gradient without the i k_v factor', 'control', ebad, 1e-8);
r(end+1) = mk(S, 'vertical current component is zero', 'check', evert, 0.5e-15);

[v0, ~] = current_profile(0, env);
r(end+1) = mk(S, 'Ekman surface value equals Winf + W0', 'check', max(abs(v0(1:2) - [real(env.Winf + env.W0); imag(env.Winf + env.W0)])), 1e-14);
[vf, ~] = current_profile(100, env);
r(end+1) = mk(S, 'Ekman far field equals Winf (max difference, m/s)', 'check', max(abs(vf(1:2) - [real(env.Winf); imag(env.Winf)])), 1e-6);
[va, ga] = current_profile(-1, env);
r(end+1) = mk(S, 'above the surface: surface value held, zero gradient', 'check', max(abs([va(1:2) - v0(1:2); ga])), 1e-14);

[vs, gs] = current_profile(5, struct('preset', 'still'));
r(end+1) = mk(S, 'preset still: zero current and gradient', 'check', max(abs([vs; gs])), 0.5e-15);
[vu, gu] = current_profile(5, struct('preset', 'uniform', 'vn', 0.3, 've', -0.2));
r(end+1) = mk(S, 'preset uniform: constant current, zero gradient', 'check', max(abs([vu - [0.3; -0.2; 0]; gu])), 1e-15);
sh = struct('preset', 'shear', 'vn', 0.1, 've', 0.0, 'gn', -0.02, 'ge', 0.01);
[~, gsh] = current_profile(4, sh);
fd = (current_profile(4 + h, sh) - current_profile(4 - h, sh)) / (2 * h);
r(end+1) = mk(S, 'preset shear: gradient vs finite difference', 'check', max(abs(gsh - fd)), 1e-9);
end
