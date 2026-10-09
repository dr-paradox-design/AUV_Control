function [r, info] = run_all_tests()
% RUN_ALL_TESTS  Run every suite, print one line per check and write
% results/test_results.txt. Tolerances are fixed in the test files before any run.
% A 'check' passes when its error is below the tolerance; a 'control' (a deliberately wrong
% input given to the same check) passes only when the check rejects it.
here = fileparts(mfilename('fullpath'));
addpath(fileparts(here));
addpath(fullfile(fileparts(here), 'config'));
setup_plant_paths();
p = params_plant();

suites = {'test_matrices', 'test_thrusters', 'test_current', 'test_analytic', ...
          'test_conservation', 'test_invariance', 'test_cross_check', 'test_numerics'};
r = new_results(); info = struct();
for i = 1:numel(suites)
    t0 = tic;
    fprintf('%s\n', suites{i});
    try
        if strcmp(suites{i}, 'test_invariance')
            [ri, info.invariance] = feval(suites{i}, p);
        else
            ri = feval(suites{i}, p);
        end
    catch err
        ri = mk(suites{i}, ['suite raised an error: ' err.message], 'check', Inf, 0);
    end
    for j = 1:numel(ri)
        fprintf('  %-4s %-95s err %.3e  tol %.1e\n', pf(ri(j).pass, ri(j).kind), ri(j).name, ri(j).err, ri(j).tol);
    end
    r = [r, ri]; %#ok<AGROW>
    fprintf('  (%.1f s)\n', toc(t0));
end

isc = strcmp({r.kind}, 'check'); isk = strcmp({r.kind}, 'control'); ok = [r.pass];
fprintf('\nchecks passed: %d of %d;  failure controls that failed as intended: %d of %d\n', ...
        sum(ok & isc), sum(isc), sum(ok & isk), sum(isk));

outdir = fullfile(fileparts(here), 'results');
if ~exist(outdir, 'dir'), mkdir(outdir); end
fid = fopen(fullfile(outdir, 'test_results.txt'), 'w');
fprintf(fid, 'plant_m test results (MATLAB %s)\n', version);
fprintf(fid, 'checks passed: %d of %d; failure controls that failed as intended: %d of %d\n\n', ...
        sum(ok & isc), sum(isc), sum(ok & isk), sum(isk));
for j = 1:numel(r)
    fprintf(fid, '%-4s | %-13s | %-7s | %s | err %.3e | tol %.1e\n', pf(r(j).pass, r(j).kind), r(j).suite, r(j).kind, r(j).name, r(j).err, r(j).tol);
end
fclose(fid);
save(fullfile(outdir, 'test_results.mat'), 'r');
end


function s = pf(b, kind)
if strcmp(kind, 'info'), s = 'INFO';
elseif b, s = 'PASS';
else, s = 'FAIL';
end
end
