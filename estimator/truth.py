"""Designed truth trajectory and simulated sensors (no plant model).

The trajectory is defined analytically (speed, heading, depth, roll, pitch with
closed-form derivatives), so IMU increments can be generated consistently:
  - gyro increment:  exact, from the truth attitude and the known Earth rate,
  - velocity increment: Simpson integral of the true specific force.
"""
import numpy as np

from . import config as C
from .rotations import (euler_to_R, euler_to_quat, quat_conj, quat_exp, quat_log,
                        quat_mul, skew)


# ----------------------------------------------------------------------------- profiles
def _smoothstep(x):
    """Quintic 0->1 on [0, 1] with zero 1st and 2nd derivatives at both ends."""
    x = np.clip(x, 0.0, 1.0)
    s = x ** 3 * (10 - 15 * x + 6 * x ** 2)
    ds = 30 * x ** 2 * (1 - x) ** 2
    dds = 60 * x * (1 - x) * (1 - 2 * x)
    return s, ds, dds


def _turn(x):
    """0->1 on [0, 1] with derivative 1 - cos(2 pi x)."""
    inside = (x > 0) & (x < 1)
    xc = np.clip(x, 0.0, 1.0)
    s = xc - np.sin(2 * np.pi * xc) / (2 * np.pi)
    ds = np.where(inside, 1 - np.cos(2 * np.pi * xc), 0.0)
    dds = np.where(inside, 2 * np.pi * np.sin(2 * np.pi * xc), 0.0)
    return s, ds, dds


def profiles(t, sc=C.SCENARIO):
    """Return dict of analytic profiles and their time derivatives at times t."""
    t = np.asarray(t, dtype=float)
    # surge speed: ramps up between 10 s and 40 s
    s, ds, dds = _smoothstep((t - 10.0) / 30.0)
    U, dU = sc["surge"] * s, sc["surge"] * ds / 30.0
    # heading: 180 deg turns
    psi = np.zeros_like(t); dpsi = np.zeros_like(t); ddpsi = np.zeros_like(t)
    L = sc["turn_len"]
    for t0, sgn in zip(sc["turn_times"], sc["turn_signs"]):
        s, ds, dds = _turn((t - t0) / L)
        psi += sgn * np.pi * s
        dpsi += sgn * np.pi * ds / L
        ddpsi += sgn * np.pi * dds / L ** 2
    # depth: dive and ascent
    D = sc["dive_depth"]
    s1, ds1, dds1 = _smoothstep((t - sc["dive_start"]) / sc["dive_len"])
    s2, ds2, dds2 = _smoothstep((t - sc["ascent_start"]) / sc["ascent_len"])
    d = D * (s1 - s2)
    dd = D * (ds1 / sc["dive_len"] - ds2 / sc["ascent_len"])
    ddd = D * (dds1 / sc["dive_len"] ** 2 - dds2 / sc["ascent_len"] ** 2)
    # small roll / pitch oscillations (gives the filter attitude excitation)
    phi, dphi = 0.02 * np.sin(0.3 * t), 0.02 * 0.3 * np.cos(0.3 * t)
    theta, dtheta = 0.03 * np.sin(0.21 * t), 0.03 * 0.21 * np.cos(0.21 * t)
    return dict(U=U, dU=dU, psi=psi, dpsi=dpsi, ddpsi=ddpsi, d=d, dd=dd, ddd=ddd,
                phi=phi, dphi=dphi, theta=theta, dtheta=dtheta)


def kinematics(t):
    """Truth NED velocity, acceleration, Euler angles and body rates w_nb at times t."""
    P = profiles(t)
    c, s = np.cos(P["psi"]), np.sin(P["psi"])
    v = np.stack([P["U"] * c, P["U"] * s, P["dd"]], -1)
    a = np.stack([P["dU"] * c - P["U"] * P["dpsi"] * s,
                  P["dU"] * s + P["U"] * P["dpsi"] * c,
                  P["ddd"]], -1)
    phi, th, psi = P["phi"], P["theta"], P["psi"]
    dphi, dth, dpsi = P["dphi"], P["dtheta"], P["dpsi"]
    # body rates from Euler rates (Fossen zyx)
    w = np.stack([dphi - dpsi * np.sin(th),
                  dth * np.cos(phi) + dpsi * np.cos(th) * np.sin(phi),
                  -dth * np.sin(phi) + dpsi * np.cos(th) * np.cos(phi)], -1)
    return dict(v=v, a=a, phi=phi, theta=th, psi=psi, w_nb=w, depth=P["d"])


# ----------------------------------------------------------------------------- truth
class Truth:
    """Truth trajectory sampled at the IMU rate, with noise-free IMU increments."""

    def __init__(self, dt=1.0 / C.IMU["rate_hz"], duration=C.SCENARIO["duration"], m=4):
        self.dt = dt
        n = int(round(duration / dt))
        self.t = np.arange(n + 1) * dt
        self.w_ie, self.g = C.earth_vectors()

        # fine grid for Simpson integration (m sub-intervals per IMU step, m even)
        tf = np.arange(n * m + 1) * (dt / m)
        kf = kinematics(tf)
        Rf = euler_to_R(kf["phi"], kf["theta"], kf["psi"])
        # specific force in NED: f_n = a - g + 2 w_ie x v
        f_n = kf["a"] - self.g + 2 * np.cross(self.w_ie, kf["v"])
        wts = np.ones(m + 1); wts[1:-1:2] = 4; wts[2:-1:2] = 2
        wts *= (dt / m) / 3.0
        idx = np.arange(n)[:, None] * m + np.arange(m + 1)[None, :]
        int_f = np.einsum("kj,kjc->kc", wts[None, :] * np.ones((n, 1)), f_n[idx])
        int_v = np.einsum("kj,kjc->kc", wts[None, :] * np.ones((n, 1)), kf["v"][idx])

        k = kinematics(self.t)
        self.v = k["v"]
        self.w_nb = k["w_nb"]
        self.R = euler_to_R(k["phi"], k["theta"], k["psi"])
        self.q = euler_to_quat(k["phi"], k["theta"], k["psi"])
        self.euler = np.stack([k["phi"], k["theta"], k["psi"]], -1)
        self.p = np.zeros((n + 1, 3))
        self.p[1:] = np.cumsum(int_v, axis=0)
        self.p[:, 2] = k["depth"]          # analytic depth (matches the integral)
        self._pz_check = np.concatenate([[0.0], np.cumsum(int_v[:, 2])])

        # noise-free increments over [t_k, t_k+1]
        dq_earth = quat_exp(self.w_ie * dt)
        self.dtheta = quat_log(quat_mul(quat_mul(quat_conj(self.q[:-1]), dq_earth), self.q[1:]))
        self.dvel = np.einsum("kji,kj->ki", self.R[:-1], int_f)   # R_k^T * integral of f_n
        self.n = n


# ----------------------------------------------------------------------------- sensors
def simulate_biases(n, dt, sigma, tau, model, rng, dim=3):
    """Bias sequence. 'gm': first-order Gauss-Markov. 'rw': random walk with the
    same driving noise density (2 sigma^2 / tau), so the two differ only in decay."""
    b = np.zeros((n + 1, dim))
    b[0] = rng.normal(0.0, sigma, dim)
    if model == "gm":
        phi = np.exp(-dt / tau)
        s = sigma * np.sqrt(1 - phi ** 2)
    elif model == "rw":
        phi = 1.0
        s = np.sqrt(2 * sigma ** 2 / tau * dt)
    else:
        raise ValueError(model)
    w = rng.normal(0.0, s, (n, dim))
    for k in range(n):
        b[k + 1] = phi * b[k] + w[k]
    return b


def simulate_imu(tr, bias_model, rng):
    dt, n = tr.dt, tr.n
    I = C.IMU
    bg = simulate_biases(n, dt, I["gyro_bias_sigma"], I["bias_tau"], bias_model, rng)
    ba = simulate_biases(n, dt, I["accel_bias_sigma"], I["bias_tau"], bias_model, rng)
    dth = tr.dtheta + bg[:-1] * dt + rng.normal(0, I["gyro_nd"] * np.sqrt(dt), (n, 3))
    dv = tr.dvel + ba[:-1] * dt + rng.normal(0, I["accel_nd"] * np.sqrt(dt), (n, 3))
    return dict(dtheta=dth, dvel=dv, bg=bg, ba=ba)


def _every(rate_hz, imu_rate=C.IMU["rate_hz"]):
    return int(round(imu_rate / rate_hz))


def simulate_aiding(tr, rng):
    """Aiding measurements at IMU indices. Returns dict sensor -> list of (k, z)."""
    out = {"dvl": [], "depth": [], "gnss_pos": [], "gnss_vel": [], "heading": []}
    w_ie = tr.w_ie
    bd = np.zeros(tr.n + 1)
    bd[0] = rng.normal(0, C.DEPTH["bias_sigma0"])
    bd[1:] = bd[0] + np.cumsum(rng.normal(0, C.DEPTH["bias_rw"] * np.sqrt(tr.dt), tr.n))
    step = C.DEPTH["full_scale"] / C.DEPTH["adc_counts"]
    for k in range(tr.n + 1):
        t, R, v, p, w = tr.t[k], tr.R[k], tr.v[k], tr.p[k], tr.w_nb[k]
        if k % _every(C.DVL["rate_hz"]) == 0:
            d0, d1 = C.DVL["dropout"]
            alt = C.DVL["seabed_depth"] - p[2]
            if p[2] > C.DVL["min_depth"] and not (d0 <= t < d1) and alt < C.DVL["max_altitude"]:
                l = C.DVL["lever"]
                z = C.DVL["R_bd"].T @ (R.T @ v + np.cross(w, l))
                out["dvl"].append((k, z + rng.normal(0, C.DVL["sigma"], 3)))
        if k % _every(C.DEPTH["rate_hz"]) == 0:
            d = p[2] + (R @ C.DEPTH["lever"])[2] + bd[k] + rng.normal(0, C.DEPTH["sigma"])
            d = np.clip(np.round(d / step) * step, 0.0, C.DEPTH["full_scale"])
            out["depth"].append((k, np.array([d])))
        if k % _every(C.GNSS["rate_hz"]) == 0:
            lg = C.GNSS["lever"]
            if (p + R @ lg)[2] < 0.0:   # antenna above water
                zp = p + R @ lg + rng.normal(0, C.GNSS["sigma_pos"])
                zv = v + R @ np.cross(w, lg) + rng.normal(0, C.GNSS["sigma_vel"], 3)
                out["gnss_pos"].append((k, zp))
                out["gnss_vel"].append((k, zv))
        if k % _every(C.HEADING["rate_hz"]) == 0:
            psi = tr.euler[k, 2] + rng.normal(0, C.HEADING["sigma"])
            out["heading"].append((k, np.array([psi])))
    return out, bd
