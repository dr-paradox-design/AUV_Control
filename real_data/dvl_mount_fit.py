"""Estimate the DVL mounting yaw from the data itself (no ground truth needed).

    [REAL_DATA_RUN=<run>] python dvl_mount_fit.py

Body-frame kinematics: d(v_b)/dt + w x v_b = f_b + R^T g, with v_b = Rz(psi) v_dvl (+ lever-arm term).
The IMU gives the right-hand side; the DVL (in its own frame) gives v_dvl. The yaw psi that makes the two
agree best is the DVL mounting about body z. Both sides are low-passed (1 s) to tame differentiation noise.
Reported next to the rotation the run's /tf_static claims.
"""
import numpy as np
import pandas as pd
from scipy.spatial.transform import Rotation as Rot

from paths import EXTRACTED, RESULTS, RUN, static_tf

G = np.array([0.0, 0.0, -9.81])            # gravity in ENU (z up)
rd = lambda n: pd.read_parquet(EXTRACTED / f'{n}.parquet').sort_values('t_bag').reset_index(drop=True)
imu, dvl = rd('imu_data'), rd('dvl_raw')
lever, R_tf = static_tf('dvl_link')
tf_yaw = Rot.from_matrix(R_tf).as_euler('ZYX', degrees=True)[0]

# common 10 Hz grid over stretches where both streams are present
t0 = imu.t_bag.iloc[0]
ti, td = imu.t_bag.values - t0, dvl.t_bag.values - t0
grid = np.arange(max(ti[0], td[0]) + 1, min(ti[-1], td[-1]) - 1, 0.1)
gap_ok = np.ones(len(grid), bool)
for t in (ti, td):
    i = np.clip(np.searchsorted(t, grid), 1, len(t) - 1)
    gap_ok &= (t[i] - t[i - 1]) < 0.6
lerp = lambda t, x, g: np.column_stack([np.interp(g, t, x[:, i]) for i in range(x.shape[1])])
R_wb = Rot.from_quat(lerp(ti, imu[['qx', 'qy', 'qz', 'qw']].values, grid))   # body FLU -> ENU (renormalised)
w_b = lerp(ti, imu[['wx', 'wy', 'wz']].values, grid)
f_b = lerp(ti, imu[['ax', 'ay', 'az']].values, grid)
a_b = f_b + R_wb.inv().apply(G)                                               # linear acceleration, body
v_d = lerp(td, dvl[['vx', 'vy', 'vz']].values, grid)


def smooth(x, n=10):
    k = np.ones(n) / n
    return np.column_stack([np.convolve(x[:, i], k, mode='same') for i in range(x.shape[1])])


a_s, w_s = smooth(a_b), smooth(w_b)
ok = gap_ok.copy()
ok[:15] = ok[-15:] = False
results = []
for psi in np.arange(-180, 180, 1.0):
    c, s = np.cos(np.radians(psi)), np.sin(np.radians(psi))
    Rz = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
    v_b = v_d @ Rz.T - np.cross(w_b, lever)
    v_s = smooth(v_b)
    dv = np.gradient(v_s, 0.1, axis=0) + np.cross(w_s, v_s)
    r = (dv - a_s)[ok][:, :2]                                                  # horizontal components
    results.append((psi, float(np.sqrt(np.mean(np.sum(r ** 2, axis=1))))))
res = np.array(results)
best = res[np.argmin(res[:, 1])]
at_tf = res[np.argmin(np.abs(((res[:, 0] - tf_yaw) + 180) % 360 - 180)), 1]
zero = res[np.argmin(np.abs(res[:, 0])), 1]
lines = [f'Run {RUN or "2026-11-05"}: {ok.sum()} samples at 10 Hz used.',
         f'/tf_static says DVL yaw {tf_yaw:+.0f} deg -> residual {at_tf:.4f} m/s^2',
         f'best-fit DVL yaw          {best[0]:+.0f} deg -> residual {best[1]:.4f} m/s^2 (yaw 0: {zero:.4f})',
         'residual at other angles: ' + ', '.join(f'{p:+.0f}: {v:.4f}' for p, v in res[::45])]
text = '\n'.join(lines) + '\n'
print(text)
RESULTS.mkdir(parents=True, exist_ok=True)
(RESULTS / 'dvl_mount_fit.txt').write_text(text)
np.savetxt(RESULTS / 'dvl_mount_fit_curve.csv', res, delimiter=',', header='psi_deg,residual_m_s2', fmt='%.5f')
