function s = mk(suite, name, kind, err, tol)
% MK  One test result. kind 'check': passes when err < tol. kind 'control': a deliberately
% wrong input given to the same check; it passes only when the check REJECTS it (err > tol).
if strcmp(kind, 'check')
    pass = err < tol;
elseif strcmp(kind, 'control')
    pass = err > tol;
elseif strcmp(kind, 'info')
    pass = true;                      % a recorded observation, not a pass/fail check
else
    error('unknown kind %s', kind);
end
s = struct('suite', suite, 'name', name, 'kind', kind, 'err', err, 'tol', tol, 'pass', logical(pass));
end
