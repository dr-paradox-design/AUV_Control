function run_all_tests(include_slow)
% RUN_ALL_TESTS  Run the estimator tests in MATLAB or GNU Octave.
%   run_all_tests        % fast tests (~1 min in Octave, seconds in MATLAB)
%   run_all_tests(true)  % also the Monte Carlo consistency test (slow)
if nargin < 1, include_slow = false; end
here = fileparts(mfilename('fullpath'));
addpath(fileparts(here)); setup_paths();
tests = {'test_jacobians', 'test_mechanisation', 'test_cross_check_python'};
if include_slow, tests{end + 1} = 'test_consistency'; end
nfail = 0;
for i = 1:numel(tests)
    fprintf('%s\n', tests{i});
    try
        feval(tests{i});
        fprintf('  PASS\n');
    catch err
        nfail = nfail + 1;
        fprintf('  FAIL: %s\n', err.message);
    end
end
fprintf('\n%d of %d tests passed\n', numel(tests) - nfail, numel(tests));
end
