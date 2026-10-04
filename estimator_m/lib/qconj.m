function r = qconj(q)
% QCONJ  Quaternion conjugate (rows [w x y z]).
r = [q(:,1), -q(:,2:4)];
end
