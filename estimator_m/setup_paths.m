function setup_paths()
% SETUP_PATHS  Add the estimator folders to the path.
here = fileparts(mfilename('fullpath'));
addpath(here, fullfile(here, 'lib'), fullfile(here, 'tests'));
end
