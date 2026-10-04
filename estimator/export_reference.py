"""Export one Python run (inputs and outputs) for the MATLAB cross-check.

Writes estimator_m/tests/reference_python.mat. The MATLAB test feeds the same
IMU data, measurements and initial state to its own filter and compares.
Usage: python -m estimator.export_reference
"""
import numpy as np
from scipy.io import savemat

from estimator import config as C
from estimator.eskf import ESKF, NX, State
from estimator.sim import initial_covariance
from estimator.truth import Truth, simulate_aiding, simulate_imu

DURATION = 120.0     # covers surface GNSS, dive start, all sensors
LOG_EVERY = 10


def main(seed=42, bias_model="gm", out="estimator_m/tests/reference_python.mat"):
    tr = Truth(duration=DURATION)
    rng = np.random.default_rng(seed)
    imu = simulate_imu(tr, bias_model, rng)
    meas, bd = simulate_aiding(tr, rng)
    P0 = initial_covariance()
    x0 = State(tr.p[0], tr.v[0], tr.q[0], imu["ba"][0], imu["bg"][0], bd[0]).oplus(
        rng.multivariate_normal(np.zeros(NX), P0))

    order = ["dvl", "depth", "gnss_pos", "gnss_vel", "heading"]
    by_k = {}
    for name in order:
        for k, z in meas[name]:
            by_k.setdefault(k, []).append((name, z))

    f = ESKF(x0, P0, bias_model)
    xs = []

    def log():
        x = f.x
        xs.append(np.concatenate([x.p, x.v, x.q, x.ba, x.bg, [x.bd]]))

    for name, z in by_k.get(0, []):
        f.update(name, z)
    log()
    for k in range(tr.n):
        f.predict(imu["dtheta"][k], imu["dvel"][k], tr.dt)
        for name, z in by_k.get(k + 1, []):
            f.update(name, z)
        if (k + 1) % LOG_EVERY == 0:
            log()

    ref = dict(duration=DURATION, log_every=LOG_EVERY, bias_model=bias_model,
               # truth subsampled every LOG_EVERY samples to keep the file small
               truth_p=tr.p[::LOG_EVERY], truth_v=tr.v[::LOG_EVERY], truth_q=tr.q[::LOG_EVERY],
               dtheta_true=tr.dtheta[::LOG_EVERY], dvel_true=tr.dvel[::LOG_EVERY],
               imu_dtheta=imu["dtheta"], imu_dvel=imu["dvel"], P0=P0,
               x0=np.concatenate([x0.p, x0.v, x0.q, x0.ba, x0.bg, [x0.bd]]),
               x_log=np.array(xs), P_end=f.P)
    for name in order:   # sample index made 1-based for MATLAB
        ref[f"k_{name}"] = np.array([k + 1 for k, _ in meas[name]], float)
        ref[f"z_{name}"] = np.array([np.atleast_1d(z) for _, z in meas[name]])
    savemat(out, ref, do_compression=True)
    print(f"wrote {out}: {tr.n} IMU steps, {len(xs)} logged states")


if __name__ == "__main__":
    main()
