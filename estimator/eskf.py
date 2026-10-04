"""Error-state Kalman filter (log section 12).

Nominal state: p, v (NED), q (body->NED, Hamilton, scalar first), a_b, w_b (body),
b_d (depth-sensor bias).
Error state (16): [dp(0:3), dv(3:6), dth(6:9), da_b(9:12), dw_b(12:15), db_d(15)].
Local attitude error: q_true = q (x) exp(dth).
"""
import numpy as np

from . import config as C
from .rotations import quat_exp, quat_mul, quat_normalize, quat_to_R, skew, wrap, quat_log, quat_conj

I3 = np.eye(3)
NX = 16


class State:
    __slots__ = ("p", "v", "q", "ba", "bg", "bd")

    def __init__(self, p, v, q, ba, bg, bd):
        self.p, self.v, self.q = np.array(p, float), np.array(v, float), np.array(q, float)
        self.ba, self.bg, self.bd = np.array(ba, float), np.array(bg, float), float(bd)

    def copy(self):
        return State(self.p, self.v, self.q, self.ba, self.bg, self.bd)

    def oplus(self, dx):
        """x (+) dx  (Sola eq. 282)."""
        return State(self.p + dx[0:3], self.v + dx[3:6],
                     quat_normalize(quat_mul(self.q, quat_exp(dx[6:9]))),
                     self.ba + dx[9:12], self.bg + dx[12:15], self.bd + dx[15])

    def ominus(self, ref):
        """dx such that ref (+) dx = self."""
        dx = np.zeros(NX)
        dx[0:3] = self.p - ref.p
        dx[3:6] = self.v - ref.v
        dx[6:9] = quat_log(quat_mul(quat_conj(ref.q), self.q))
        dx[9:12] = self.ba - ref.ba
        dx[12:15] = self.bg - ref.bg
        dx[15] = self.bd - ref.bd
        return dx


def propagate_nominal(x, dtheta, dvel, dt, w_ie, g, bias_model):
    """Nominal mechanisation over one IMU interval (log 12.3 and 12.8)."""
    w = dtheta / dt - x.bg           # w_ib estimate
    a = dvel / dt - x.ba             # specific force estimate (body)
    R = quat_to_R(x.q)
    acc_n = R @ a + g - 2 * np.cross(w_ie, x.v)
    xn = x.copy()
    xn.p = x.p + x.v * dt + 0.5 * acc_n * dt ** 2
    xn.v = x.v + acc_n * dt
    xn.q = quat_normalize(quat_mul(quat_mul(quat_exp(-w_ie * dt), x.q), quat_exp(w * dt)))
    if bias_model == "gm":
        phi = np.exp(-dt / C.IMU["bias_tau"])
        xn.ba = phi * x.ba
        xn.bg = phi * x.bg
    return xn, w, a, R


def error_transition(R, w, a, dt, w_ie, bias_model):
    """F_x (Sola eq. 269, plus Coriolis term and bias decay)."""
    F = np.eye(NX)
    F[0:3, 3:6] = I3 * dt
    F[3:6, 3:6] = I3 - 2 * skew(w_ie) * dt
    F[3:6, 6:9] = -R @ skew(a) * dt
    F[3:6, 9:12] = -R * dt
    F[6:9, 6:9] = quat_to_R(quat_exp(w * dt)).T
    F[6:9, 12:15] = -I3 * dt
    if bias_model == "gm":
        phi = np.exp(-dt / C.IMU["bias_tau"])
        F[9:15, 9:15] = phi * np.eye(6)
    return F


def process_noise(dt, bias_model):
    I = C.IMU
    Q = np.zeros((NX, NX))
    Q[3:6, 3:6] = I["accel_nd"] ** 2 * dt * I3
    Q[6:9, 6:9] = I["gyro_nd"] ** 2 * dt * I3
    for sl, sig in ((slice(9, 12), I["accel_bias_sigma"]), (slice(12, 15), I["gyro_bias_sigma"])):
        if bias_model == "gm":
            phi = np.exp(-dt / I["bias_tau"])
            Q[sl, sl] = sig ** 2 * (1 - phi ** 2) * I3
        else:
            Q[sl, sl] = 2 * sig ** 2 / I["bias_tau"] * dt * I3
    Q[15, 15] = C.DEPTH["bias_rw"] ** 2 * dt
    return Q


# ----------------------------------------------------------------------------- measurements
# Each returns (h, H) for the current nominal state. w_ib is the latest gyro rate
# estimate; the lever-arm terms use w_nb = w_ib - R^T w_ie (log 12.4, refined).
def h_dvl(x, w_ib, w_ie):
    R = quat_to_R(x.q)
    l, Rbd = C.DVL["lever"], C.DVL["R_bd"]
    wie_b = R.T @ w_ie
    w_nb = w_ib - wie_b
    h = Rbd.T @ (R.T @ x.v + np.cross(w_nb, l))
    H = np.zeros((3, NX))
    H[:, 3:6] = Rbd.T @ R.T
    H[:, 6:9] = Rbd.T @ (skew(R.T @ x.v) + skew(l) @ skew(wie_b))
    H[:, 12:15] = Rbd.T @ skew(l)
    return h, H


def h_depth(x, w_ib=None, w_ie=None):
    R = quat_to_R(x.q)
    l = C.DEPTH["lever"]
    h = np.array([x.p[2] + (R @ l)[2] + x.bd])
    H = np.zeros((1, NX))
    H[0, 2] = 1.0
    H[0, 6:9] = -(R @ skew(l))[2]
    H[0, 15] = 1.0
    return h, H


def h_gnss_pos(x, w_ib=None, w_ie=None):
    R = quat_to_R(x.q)
    l = C.GNSS["lever"]
    H = np.zeros((3, NX))
    H[:, 0:3] = I3
    H[:, 6:9] = -R @ skew(l)
    return x.p + R @ l, H


def h_gnss_vel(x, w_ib, w_ie):
    R = quat_to_R(x.q)
    l = C.GNSS["lever"]
    wie_b = R.T @ w_ie
    w_nb = w_ib - wie_b
    h = x.v + R @ np.cross(w_nb, l)
    H = np.zeros((3, NX))
    H[:, 3:6] = I3
    H[:, 6:9] = -R @ skew(np.cross(w_nb, l)) + R @ skew(l) @ skew(wie_b)
    H[:, 12:15] = R @ skew(l)
    return h, H


def h_heading(x, w_ib=None, w_ie=None):
    """Yaw psi (Fossen zyx). Exact small-angle Jacobian:
    d psi = [tan(theta) cos(psi), tan(theta) sin(psi), 1] R dth."""
    R = quat_to_R(x.q)
    psi = np.arctan2(R[1, 0], R[0, 0])
    theta = -np.arcsin(np.clip(R[2, 0], -1, 1))
    row = np.array([np.tan(theta) * np.cos(psi), np.tan(theta) * np.sin(psi), 1.0]) @ R
    H = np.zeros((1, NX))
    H[0, 6:9] = row
    return np.array([psi]), H


MEAS = {
    "dvl": (h_dvl, lambda: C.DVL["sigma"] ** 2 * I3),
    "depth": (h_depth, lambda: np.array([[C.DEPTH["sigma"] ** 2
                                          + (C.DEPTH["full_scale"] / C.DEPTH["adc_counts"]) ** 2 / 12]])),
    "gnss_pos": (h_gnss_pos, lambda: np.diag(C.GNSS["sigma_pos"] ** 2)),
    "gnss_vel": (h_gnss_vel, lambda: C.GNSS["sigma_vel"] ** 2 * I3),
    "heading": (h_heading, lambda: np.array([[C.HEADING["sigma"] ** 2]])),
}


class ESKF:
    def __init__(self, x0, P0, bias_model="rw"):
        self.x, self.P = x0.copy(), P0.copy()
        self.bias_model = bias_model
        self.w_ie, self.g = C.earth_vectors()
        self.w_ib = np.zeros(3)
        self._Q = {}

    def predict(self, dtheta, dvel, dt):
        xn, w, a, R = propagate_nominal(self.x, dtheta, dvel, dt, self.w_ie, self.g, self.bias_model)
        F = error_transition(R, w, a, dt, self.w_ie, self.bias_model)
        if dt not in self._Q:
            self._Q[dt] = process_noise(dt, self.bias_model)
        self.P = F @ self.P @ F.T + self._Q[dt]
        self.x, self.w_ib = xn, w

    def update(self, name, z):
        """Returns the normalised innovation squared (NIS)."""
        hfun, Rfun = MEAS[name]
        h, H = hfun(self.x, self.w_ib, self.w_ie)
        Rm = Rfun()
        r = z - h
        if name == "heading":
            r = wrap(r)
        S = H @ self.P @ H.T + Rm
        K = np.linalg.solve(S, H @ self.P).T
        dx = K @ r
        IKH = np.eye(NX) - K @ H
        self.P = IKH @ self.P @ IKH.T + K @ Rm @ K.T     # Joseph form
        self.P = 0.5 * (self.P + self.P.T)
        self.x = self.x.oplus(dx)                         # injection, reset G = I
        return float(r @ np.linalg.solve(S, r))
