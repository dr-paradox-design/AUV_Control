function check_report_claims()
% CHECK_REPORT_CLAIMS  Recompute the numbers in AUV_Midterm_Report.pdf sections 5.5.1 and
% 5.5.2 from the parameters of Wu (2018) Tables 5.1-5.3, which were read directly in
% the thesis PDF (this session). Writes results/report_claims.txt.
here = fileparts(mfilename('fullpath')); root = fileparts(here);
addpath(fullfile(root, 'config')); setup_plant_paths();
L = {};
% Wu 2018 Tables 5.1-5.3 (values as printed; magnitudes)
W = 112.8; BG = 0.02; Xud = 5.5; Zwd = 14.57;
Xu = 4.03; Xuu = 18.18;
T = 4 * 40 * cos(pi / 4);                                  % four horizontal T200 at 45 deg, ~40 N each (Wu p.49)
L{end+1} = sprintf('net surge thrust 4*40*cos45 = %.1f N (report: 113 N)', T);
uq = sqrt(T / Xuu);
ul = (-Xu + sqrt(Xu^2 + 4 * Xuu * T)) / (2 * Xuu);
L{end+1} = sprintf('top speed, quadratic drag only (eq. 3.12 as printed): %.3f m/s', uq);
L{end+1} = sprintf('top speed, with Wu linear Xu = -4.03: %.3f m/s (report: 2.39)', ul);
k = (T - Xu * 1.5) / 1.5^2;
L{end+1} = sprintf('Xu|u| implied by 1.5 m/s, with linear term: %.1f (report: -47.6), ratio to 18.18: %.2f (report: 2.6)', k, k / Xuu);
L{end+1} = sprintf('Xu|u| implied by 1.5 m/s, quadratic only: %.1f', T / 1.5^2);
Mrm = W * BG;
L{end+1} = sprintf('maximum righting moment W*BG = %.3f N m (report: 2.26)', Mrm);
Uc = sqrt(2 * Mrm / (Zwd - Xud));
L{end+1} = sprintf('Munk crossover, max Munk moment 1/2 (Zwd-Xud) U^2 = W*BG: U = %.3f m/s (report: 0.71)', Uc);
L{end+1} = sprintf('small-angle instability threshold, (Zwd-Xud) U^2 = W*BG: U = %.3f m/s', sqrt(Mrm / (Zwd - Xud)));

% open-loop pitch behaviour by simulation with the Wu parameters (neutral and Wu's B = 114.8 N)
for Bv = [112.8 114.8]
    p = params_plant();
    p.m = 11.5; p.rg = [0; 0; 0.02]; p.rb = [0; 0; 0]; p.Ig = 0.16 * eye(3); p.A = [5.5 12.7 14.57 0.12 0.12 0.12];
    p.Dl = [4.03 6.22 5.18 0.07 0.07 0.07]; p.Dq = [18.18 21.66 36.99 1.55 1.55 1.55];
    p = update_params(p); p.B = Bv;
    L{end+1} = sprintf('pitch scan, B = %.1f N, 2 deg initial pitch, surge thrust holds speed U, 60 s: max |theta| in degrees', Bv);
    for U = [0.3 0.4 0.5 0.6 0.71 0.8 0.9 1.0]
        tx = p.Dl(1) * U + p.Dq(1) * U^2;
        o = simulate([0; 0; 5; euler_to_quat(0, 2 * pi / 180, 0); U; 0; 0; 0; 0; 0], @(t) [tx; 0; 0; 0; 0; 0], p, 0.005, 60);
        th = zeros(size(o.t)); for kk = 1:numel(th), e = quat_to_euler(o.X(kk, 4:7)'); th(kk) = e(2); end
        L{end+1} = sprintf('   U = %.2f m/s: max |theta| = %6.1f deg, final |theta| = %6.1f deg', U, max(abs(th)) * 180 / pi, abs(th(end)) * 180 / pi); %#ok<AGROW>
    end
end
outdir = fullfile(root, 'results'); if ~exist(outdir, 'dir'), mkdir(outdir); end
fid = fopen(fullfile(outdir, 'report_claims.txt'), 'w'); fprintf(fid, '%s\n', L{:}); fclose(fid);
fprintf('%s\n', L{:});
end
