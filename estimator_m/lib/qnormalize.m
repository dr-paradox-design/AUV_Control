function q = qnormalize(q)
q = q ./ sqrt(sum(q.^2, 2));
end
