function [vc, dvcdz] = current_profile(z, env)
% CURRENT_PROFILE  Ocean current v_c^n in NED and its depth gradient dv_c/dz at depth z
% (z down, so z > 0 is under water). Presets: still, uniform, shear, ekman.
%   ekman: W(z) = Winf + W0 exp(-kd z) exp(i kv z), W = vN + i vE (report 3.20),
%          dW/dz = (-kd + i kv)(W - Winf)                          (report 3.21)
% Above the surface (z < 0) the profile is held at its surface value with zero gradient.
% The vertical component is zero in every preset.
zc = max(z, 0);
inside = z >= 0;
switch env.preset
    case 'still'
        vc = [0; 0; 0]; dvcdz = [0; 0; 0];
    case 'uniform'
        vc = [env.vn; env.ve; 0]; dvcdz = [0; 0; 0];
    case 'shear'
        vc = [env.vn + env.gn * zc; env.ve + env.ge * zc; 0];
        dvcdz = inside * [env.gn; env.ge; 0];
    case 'ekman'
        e = exp((-env.kd + 1i * env.kv) * zc);
        W  = env.Winf + env.W0 * e;
        dW = (-env.kd + 1i * env.kv) * (W - env.Winf);
        vc = [real(W); imag(W); 0];
        dvcdz = inside * [real(dW); imag(dW); 0];
    otherwise
        error('unknown current preset %s', env.preset);
end
end
