"""Rotation utilities.

Convention: Hamilton quaternion, scalar first, q = [w, x, y, z].
q represents body -> NED, so v_ned = R(q) v_body.
Euler angles follow Fossen's zyx convention: R = Rz(psi) Ry(theta) Rx(phi).
"""
import numpy as np


def skew(a):
    """[a]x such that skew(a) @ b = cross(a, b)."""
    return np.array([[0.0, -a[2], a[1]],
                     [a[2], 0.0, -a[0]],
                     [-a[1], a[0], 0.0]])


def quat_mul(p, q):
    """Hamilton product p (x) q. Works on (4,) or (N, 4) arrays."""
    pw, px, py, pz = np.moveaxis(np.asarray(p), -1, 0)
    qw, qx, qy, qz = np.moveaxis(np.asarray(q), -1, 0)
    return np.stack([pw * qw - px * qx - py * qy - pz * qz,
                     pw * qx + px * qw + py * qz - pz * qy,
                     pw * qy - px * qz + py * qw + pz * qx,
                     pw * qz + px * qy - py * qx + pz * qw], axis=-1)


def quat_conj(q):
    q = np.asarray(q)
    return q * np.array([1.0, -1.0, -1.0, -1.0])


def quat_exp(rv):
    """Quaternion of the rotation vector rv (angle * axis). (3,) or (N, 3)."""
    rv = np.asarray(rv, dtype=float)
    ang = np.linalg.norm(rv, axis=-1, keepdims=True)
    half = 0.5 * ang
    # sin(a/2)/a with a series fallback near zero
    k = np.where(ang > 1e-8, np.sin(half) / np.where(ang > 1e-8, ang, 1.0), 0.5 - ang ** 2 / 48.0)
    return np.concatenate([np.cos(half), k * rv], axis=-1)


def quat_log(q):
    """Rotation vector of quaternion q (shortest rotation). (4,) or (N, 4)."""
    q = np.asarray(q, dtype=float)
    q = np.where(q[..., :1] < 0, -q, q)
    w = q[..., :1]
    v = q[..., 1:]
    s = np.linalg.norm(v, axis=-1, keepdims=True)
    ang = 2.0 * np.arctan2(s, w)
    k = np.where(s > 1e-8, ang / np.where(s > 1e-8, s, 1.0), 2.0 / np.maximum(w, 1e-12))
    return k * v


def quat_normalize(q):
    return q / np.linalg.norm(q, axis=-1, keepdims=True)


def quat_to_R(q):
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def euler_to_R(phi, theta, psi):
    """Fossen Rzyx for scalars or arrays; returns (..., 3, 3)."""
    cf, sf = np.cos(phi), np.sin(phi)
    ct, st = np.cos(theta), np.sin(theta)
    cp, sp = np.cos(psi), np.sin(psi)
    R = np.stack([
        np.stack([cp * ct, -sp * cf + cp * st * sf, sp * sf + cp * cf * st], -1),
        np.stack([sp * ct, cp * cf + sf * st * sp, -cp * sf + st * sp * cf], -1),
        np.stack([-st, ct * sf, ct * cf], -1)], -2)
    return R


def euler_to_quat(phi, theta, psi):
    """q = qz(psi) (x) qy(theta) (x) qx(phi), so R(q) = Rz Ry Rx. Scalars or arrays."""
    phi, theta, psi = np.broadcast_arrays(phi, theta, psi)
    z = np.zeros_like(phi, dtype=float)
    qx = np.stack([np.cos(phi / 2), np.sin(phi / 2), z, z], -1)
    qy = np.stack([np.cos(theta / 2), z, np.sin(theta / 2), z], -1)
    qz = np.stack([np.cos(psi / 2), z, z, np.sin(psi / 2)], -1)
    return quat_mul(quat_mul(qz, qy), qx)


def R_to_euler(R):
    phi = np.arctan2(R[2, 1], R[2, 2])
    theta = -np.arcsin(np.clip(R[2, 0], -1.0, 1.0))
    psi = np.arctan2(R[1, 0], R[0, 0])
    return phi, theta, psi


def wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


def vn_quat_to_hamilton(q_vn):
    """Reorder a VN-200 scalar-last quaternion [x, y, z, w] to scalar-first [w, x, y, z].

    Only the element order is handled here. Whether the VN-200 quaternion is
    Hamilton or JPL, and body->NED or NED->body, is NOT yet confirmed from the
    manual (UM2000); check before using real data.
    """
    q_vn = np.asarray(q_vn)
    return np.concatenate([q_vn[..., 3:4], q_vn[..., :3]], axis=-1)
