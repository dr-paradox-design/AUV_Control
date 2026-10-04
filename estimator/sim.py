"""Monte Carlo runner for the ESKF on the designed trajectory."""
import numpy as np

from . import config as C
from .eskf import ESKF, NX, State
from .truth import Truth, simulate_aiding, simulate_imu

_TRUTH = None


def get_truth():
    global _TRUTH
    if _TRUTH is None:
        _TRUTH = Truth()
    return _TRUTH


def initial_covariance():
    I = C.INIT
    d = np.zeros(NX)
    d[0:3] = I["pos"]
    d[3:6] = I["vel"]
    d[6:9] = [I["roll_pitch"], I["roll_pitch"], I["yaw"]]
    d[9:12] = C.IMU["accel_bias_sigma"]
    d[12:15] = C.IMU["gyro_bias_sigma"]
    d[15] = C.DEPTH["bias_sigma0"]
    return np.diag(d ** 2)


def run_once(seed, truth_bias="gm", filter_bias="gm", use=None, log_every=10):
    """One run. Returns errors, covariance diagonals, NEES and NIS.

    use: set of sensor names to fuse (default: all).
    """
    tr = get_truth()
    rng = np.random.default_rng(seed)
    imu = simulate_imu(tr, truth_bias, rng)
    meas, bd = simulate_aiding(tr, rng)
    use = set(meas) if use is None else set(use)

    by_k = {}
    for name, lst in meas.items():
        if name in use:
            for k, z in lst:
                by_k.setdefault(k, []).append((name, z))

    def true_state(k):
        return State(tr.p[k], tr.v[k], tr.q[k], imu["ba"][k], imu["bg"][k], bd[k])

    P0 = initial_covariance()
    x0 = true_state(0).oplus(rng.multivariate_normal(np.zeros(NX), P0))
    f = ESKF(x0, P0, filter_bias)

    nis = {name: [] for name in meas}
    rec_t, rec_e, rec_sd, rec_nees = [], [], [], []

    def record(k):
        e = true_state(k).ominus(f.x)
        rec_t.append(tr.t[k])
        rec_e.append(e)
        rec_sd.append(np.sqrt(np.diag(f.P)))
        rec_nees.append(float(e @ np.linalg.solve(f.P, e)))

    for name, z in by_k.get(0, []):
        nis[name].append(f.update(name, z))
    record(0)
    for k in range(tr.n):
        f.predict(imu["dtheta"][k], imu["dvel"][k], tr.dt)
        for name, z in by_k.get(k + 1, []):
            nis[name].append(f.update(name, z))
        if (k + 1) % log_every == 0:
            record(k + 1)
    return dict(t=np.array(rec_t), err=np.array(rec_e), sd=np.array(rec_sd),
                nees=np.array(rec_nees), nis={k: np.array(v) for k, v in nis.items()})
