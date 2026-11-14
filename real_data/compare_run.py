"""Compare ESKF variants and the Python EKF with the ROS estimators on any extracted run.

    REAL_DATA_RUN=<run> python compare_run.py

References: /odometry/filtered, /odometry/ekf (when the run has it) and DVL dead reckoning (DVL rotated
into body axes with the run's /tf_static). The recording can pause (all topics stop), so tracks are
compared per gap-free segment: each segment is re-aligned at its start, then RMS is pooled.
Writes <run results>/compare_table.txt and compare.png. (compare_eskf.py keeps the 2026-11-05 tables.)
"""
from pathlib import Path

import numpy as np
import pandas as pd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from scipy.io import loadmat
from scipy.spatial.transform import Rotation as Rot

from ekf_localization import EXTRACTED, RUNS, R_DVL_LEVER, load
from paths import DATA, RESULTS, RUN

ESKF = ['eskf_white_zv', 'eskf_babig_zv', 'eskf_ungated_zv', 'eskf_accel30_zv', 'eskf_robust_x5_zv', 'eskf_hdg0p2hz_zv', 'eskf_oc_zv', 'eskf_robust_zv',
        'eskf_gated_zv', 'eskf_noheading_zv', 'eskf_datasheet_zv']

imu, dvl, _, _ = load()
t0 = imu['t_bag'].iloc[0]
log = loadmat(DATA / 'real_log.mat', squeeze_me=True)
dt = float(log['dt'])
vn = log['vn_euler_grid']
gaps = np.asarray(log['imu_gaps'], float).reshape(-1, 2)
T = float(log['n']) * dt
segs = list(zip(np.r_[0.0, gaps[:, 1]], np.r_[gaps[:, 0], T]))


def odom(name):
    f = EXTRACTED / f'{name}.parquet'
    if not f.exists():
        return None
    d = pd.read_parquet(f).sort_values('t_bag')
    return (d['t_bag'].values - t0, d[['x', 'y']].values) if len(d) else None


def dvl_dr():
    t = dvl['t_bag'].values
    j = np.clip(imu['t_bag'].searchsorted(t), 0, len(imu) - 1)
    R = Rot.from_quat(imu[['qx', 'qy', 'qz', 'qw']].values[j])
    v = dvl[['vx', 'vy', 'vz']].values - np.cross(imu[['wx', 'wy', 'wz']].values[j], R_DVL_LEVER)
    d = np.diff(t, prepend=t[0])
    d = np.where(d > 1.0, 0.0, d)
    return t - t0, np.cumsum(R.apply(v) * d[:, None], axis=0)[:, :2]


refs = {k: v for k, v in {'robot_localization (/odometry/filtered)': odom('odometry_filtered'),
                          '/odometry/ekf': odom('odometry_ekf'), 'DVL dead reckoning': dvl_dr()}.items()
        if v is not None}

est = {}
for v in ESKF:
    f = DATA / f'{v}.mat'
    if f.exists():
        r = loadmat(f, squeeze_me=True, struct_as_record=False)
        x = np.asarray(r['x'])
        q = x[:, 6:10]
        Rm = Rot.from_quat(np.column_stack([q[:, 1], q[:, 2], q[:, 3], q[:, 0]])).as_matrix()
        eul = np.column_stack([np.arctan2(Rm[:, 2, 1], Rm[:, 2, 2]), -np.arcsin(np.clip(Rm[:, 2, 0], -1, 1)),
                               np.arctan2(Rm[:, 1, 0], Rm[:, 0, 0])])
        k = np.clip(np.rint(np.asarray(r['t']) / dt).astype(int), 0, len(vn) - 1)
        de = np.degrees((eul - vn[k] + np.pi) % (2 * np.pi) - np.pi)
        est[v] = dict(t=np.asarray(r['t_bag']) - t0, xy=np.column_stack([x[:, 1], x[:, 0]]),
                      att=np.sqrt(np.mean(de ** 2, axis=0)), nis=float(np.median(r['nis'].dvl)))
e = RUNS / 'ekf_E_C_plus_stale_zupt.parquet'
if e.exists():
    d = pd.read_parquet(e)
    est['python_E'] = dict(t=d['t_bag'].values - t0, xy=d[['px', 'py']].values, att=None, nis=None)


def seg_rms(t, xy, rt, rxy):
    """RMS of horizontal difference, each track re-aligned at every segment start. Segments are the
    gap-free stretches of the recording, further split where the reference itself goes silent > 1 s."""
    sq = []
    cuts = rt[np.r_[0, np.where(np.diff(rt) > 1.0)[0] + 1]]
    for a, b in segs:
        bounds = np.r_[a, cuts[(cuts > a) & (cuts < b)], b]
        for a2, b2 in zip(bounds[:-1], bounds[1:]):
            m = (rt >= a2) & (rt < b2)
            if m.sum() < 10:
                continue
            tt = rt[m]
            p = np.column_stack([np.interp(tt, t, xy[:, i]) for i in range(2)])
            q = rxy[m]
            sq.append(np.sum(((p - p[0]) - (q - q[0])) ** 2, axis=1))
    return float(np.sqrt(np.mean(np.concatenate(sq))))


def seg_path(t, xy):
    total = 0.0
    for a, b in segs:
        g = np.arange(a, b, 0.5)
        if len(g) < 2:
            continue
        p = np.column_stack([np.interp(g, t, xy[:, i]) for i in range(2)])
        total += float(np.sum(np.linalg.norm(np.diff(p, axis=0), axis=1)))
    return total


rows = []
for name, s in est.items():
    row = {'estimator': name, 'path_m': round(seg_path(s['t'], s['xy']), 1)}
    for rn, (rt, rxy) in refs.items():
        row[f'RMS vs {rn.split(" (")[0]}'] = round(seg_rms(s['t'], s['xy'], rt, rxy), 2)
    if s['att'] is not None:
        row['roll/pitch/yaw vs VN-200 (deg)'] = '/'.join(f'{a:.2f}' for a in s['att'])
        row['DVL median NIS'] = round(s['nis'], 2)
    rows.append(row)
for rn, (rt, rxy) in refs.items():
    row = {'estimator': rn, 'path_m': round(seg_path(rt, rxy), 1)}
    for rn2, (rt2, rxy2) in refs.items():
        if rn2 != rn:
            row[f'RMS vs {rn2.split(" (")[0]}'] = round(seg_rms(rt, rxy, rt2, rxy2), 2)
    rows.append(row)

table = pd.DataFrame(rows).set_index('estimator')
pd.set_option('display.width', 300); pd.set_option('display.max_columns', 30)
text = (f'Run: {RUN or "2026-11-05"} ({T:.0f} s). Recording gaps (all topics stop): '
        + (', '.join(f'{a:.1f}-{b:.1f} s' for a, b in gaps) or 'none')
        + '.\nRMS: horizontal difference, every track re-aligned at the start of each gap-free segment.\n\n'
        + table.to_string() + '\n')
print(text)
RESULTS.mkdir(parents=True, exist_ok=True)
(RESULTS / 'compare_table.txt').write_text(text)

fig, axs = plt.subplots(1, len(segs), figsize=(6 * len(segs), 5.6))
axs = np.atleast_1d(axs)
style = {'robot_localization (/odometry/filtered)': ('tab:orange', '-'), '/odometry/ekf': ('tab:purple', '-'),
         'DVL dead reckoning': ('gray', '--'), 'eskf_accel30_zv': ('tab:blue', '-'), 'python_E': ('tab:green', '-')}
main = next((n for n in ('eskf_white_zv', 'eskf_accel30_zv', 'eskf_ungated_zv') if n in est), None)
style[main] = ('tab:blue', '-')
for ax, (a, b) in zip(axs, segs):
    for name, (t, xy) in list(refs.items()) + [(n, (est[n]['t'], est[n]['xy'])) for n in (main, 'python_E') if n in est]:
        m = (t >= a) & (t <= b)
        if m.sum() < 2:
            continue
        p = xy[m] - xy[m][0]
        c, ls = style[name]
        ax.plot(p[:, 0], p[:, 1], color=c, ls=ls, lw=1.3, label=name)
    ax.set_aspect('equal'); ax.set_title(f'segment {a:.0f}-{b:.0f} s (aligned at start)', fontsize=11)
    ax.set_xlabel('east (m)'); ax.set_ylabel('north (m)')
axs[0].legend(fontsize=8)
plt.tight_layout()
plt.savefig(RESULTS / 'compare.png', dpi=120)
print('saved', RESULTS / 'compare.png')
