function rng(seed)
% RNG  Stand-in for Octave < 7, which has no rng. Only rng(seed) with a numeric seed.
% Put on the path by setup_paths / setup_plant_paths only when rng does not exist.
% Random draws differ from MATLAB's rng anyway (different generators).
if ~(isnumeric(seed) && isscalar(seed))
    error('compat rng: only rng(seed) with a numeric seed is supported');
end
rand('state', seed);
randn('state', seed);
end
