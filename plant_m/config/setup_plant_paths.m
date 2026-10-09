function setup_plant_paths()
% SETUP_PLANT_PATHS  Add all plant_m folders to the MATLAB path. Needs no toolbox.
here = fileparts(fileparts(mfilename('fullpath')));      % plant_m
addpath(here, fullfile(here, 'config'), fullfile(here, 'lib'), fullfile(here, 'model'), ...
        fullfile(here, 'sim'), fullfile(here, 'tests'));
end
