function D = damping(nur, p)
% DAMPING  Diagonal damping matrix, linear plus quadratic in the water-relative velocity
% (report 3.12 has the quadratic part only; the linear part defaults to zero). Positive
% semi-definite, so nu_r' D nu_r >= 0.
D = diag(p.Dl(:) + p.Dq(:) .* abs(nur(:)));
end
