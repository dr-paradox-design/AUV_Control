function q = vn2hamilton(q_vn)
% VN2HAMILTON  Reorder a VN-200 scalar-last quaternion [x y z w] to [w x y z].
% Only the element order is handled. Whether the VN-200 quaternion is Hamilton or
% JPL, and body->NED or NED->body, is NOT yet confirmed from UM2000.
q = [q_vn(:,4), q_vn(:,1:3)];
end
