"""Heading observability and consistency check (see estimator_m/README.md).

Compares ESKF yaw with the VN-200's own (magnetometer-aided) yaw, and the error with the filter's predicted
yaw sigma. Run after estimator_m/run_real_log.m has produced the variants listed below.
Writes results/heading_table.txt.
"""
import sys
from pathlib import Path

import numpy as np
from scipy.io import loadmat
from scipy.spatial.transform import Rotation as Rot

from paths import DATA, RESULTS
VARIANTS = sys.argv[1:] or ['eskf_ungated_zv', 'eskf_oc_zv', 'eskf_oc_robusthdg_zv', 'eskf_hdg1hz_zv',
                            'eskf_hdg0p2hz_zv', 'eskf_noheading_zv', 'eskf_noheading_oc_zv']

log = loadmat(DATA / 'real_log.mat', squeeze_me=True)
dt = float(log['dt'])
vn_yaw = log['vn_euler_grid'][:, 2]


def wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


lines = ['Yaw of the ESKF minus the VN-200 yaw (the VN-200 uses its magnetometer; treated as reference).',
         'NEES_yaw = e^2 / sigma^2 per sample: about 1 when consistent; share outside 2 sigma about 5 %.', '']
for v in VARIANTS:
    r = loadmat(DATA / f'{v}.mat', squeeze_me=True)
    t = np.asarray(r['t'])
    q = np.asarray(r['x'])[:, 6:10]
    R = Rot.from_quat(np.column_stack([q[:, 1], q[:, 2], q[:, 3], q[:, 0]])).as_matrix()
    yaw = np.arctan2(R[:, 1, 0], R[:, 0, 0])
    e = wrap(yaw - vn_yaw[np.clip(np.rint(t / dt).astype(int), 0, len(vn_yaw) - 1)])
    sd = np.asarray(r['yaw_sd'])
    eu = np.unwrap(e)
    drift = np.polyfit(t / 60, np.degrees(eu), 1)[0]
    nees = (e / sd) ** 2
    lines.append(
        f'{v:20s} yaw error RMS {np.degrees(np.sqrt(np.mean(e**2))):5.2f} deg, final {np.degrees(e[-1]):6.2f} deg, '
        f'trend {drift:+.3f} deg/min | sigma at end {np.degrees(sd[-1]):5.2f} deg | mean NEES {nees.mean():7.2f}, '
        f'outside 2 sigma {100 * np.mean(np.abs(e) > 2 * sd):5.1f} %')
    for frac in (0.25, 0.5, 0.75, 1.0):
        i = min(len(t) - 1, int(frac * (len(t) - 1)))
        lines.append(f'{"":22s}t = {t[i]:6.0f} s: error {np.degrees(e[i]):6.2f} deg, sigma {np.degrees(sd[i]):5.2f} deg')
text = '\n'.join(lines) + '\n'
print(text)
RESULTS.mkdir(exist_ok=True)
(RESULTS / 'heading_table.txt').write_text(text)
