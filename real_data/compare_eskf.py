"""Compare the estimator_m ESKF on the recorded log with robot_localization, variant E and DVL dead reckoning.

Run after export_mat.py, estimator_m/run_real_log.m, ekf_localization.py.
Writes results/eskf_table.txt and results/eskf_comparison.png.
The ESKF works in NED; positions are converted to ENU (x = east, y = north) to match the ROS log.
"""
from pathlib import Path

import numpy as np
import pandas as pd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from scipy.io import loadmat
from scipy.spatial.transform import Rotation as Rot

from ekf_localization import EXTRACTED, RUNS, R_DVL_LEVER

HERE = Path(__file__).resolve().parent
DATA = HERE / 'data'
RESULTS = HERE / 'results'
ESKF_VARIANTS = ['eskf_datasheet_zv', 'eskf_ungated_zv', 'eskf_gated', 'eskf_gated_zv', 'eskf_loosegate_zv']
MAIN = 'eskf_gated_zv'          # rejection breakdown is shown for the 99% gate, where the spiral appears
GAP_S = 1.0

imu = pd.read_parquet(EXTRACTED / 'imu_data.parquet').sort_values('t_bag')
dvl = pd.read_parquet(EXTRACTED / 'dvl_raw.parquet').sort_values('t_bag')
odom = pd.read_parquet(EXTRACTED / 'odometry_filtered.parquet').sort_values('t_bag')
t0 = imu['t_bag'].iloc[0]
log = loadmat(DATA / 'real_log.mat', squeeze_me=True)
grid_t = np.arange(int(log['n']) + 1) * float(log['dt'])
vn_eul = log['vn_euler_grid']

odom_t = odom['t_bag'].values - t0
odom_xy = odom[['x', 'y']].values - odom[['x', 'y']].values[0]
odom_gap = np.diff(odom_t, prepend=odom_t[0]) > GAP_S
gap_starts = odom_t[np.where(odom_gap)[0] - 1] if odom_gap.any() else [odom_t[-1]]
odom_cutoff = gap_starts[0]


def interp(t_src, xy, t):
    return np.column_stack([np.interp(t, t_src, xy[:, i]) for i in range(xy.shape[1])])


def dvl_dead_reckoning():
    t = dvl['t_bag'].values
    j = np.clip(imu['t_bag'].searchsorted(t), 0, len(imu) - 1)
    R = Rot.from_quat(imu[['qx', 'qy', 'qz', 'qw']].values[j])
    w = imu[['wx', 'wy', 'wz']].values[j]
    v = dvl[['vx', 'vy', 'vz']].values - np.cross(w, R_DVL_LEVER)
    dt = np.diff(t, prepend=t[0])
    dt = np.where(dt > GAP_S, 0.0, dt)
    return t - t0, np.cumsum(R.apply(v) * dt[:, None], axis=0)[:, :2]


def zyx_from_wxyz(q):
    R = Rot.from_quat(np.column_stack([q[:, 1], q[:, 2], q[:, 3], q[:, 0]])).as_matrix()
    return np.column_stack([np.arctan2(R[:, 2, 1], R[:, 2, 2]), -np.arcsin(np.clip(R[:, 2, 0], -1, 1)),
                            np.arctan2(R[:, 1, 0], R[:, 0, 0])])


def wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


dr_t, dr_xy = dvl_dead_reckoning()
dr_dt = np.diff(dr_t)
longest = np.argmax(dr_dt)
dropout = (dr_t[longest], dr_t[longest + 1])
GRID = np.arange(0.0, grid_t[-1], 0.5)


def path_len(t, xy):
    g = interp(t, xy, GRID[(GRID >= t[0]) & (GRID <= t[-1])])
    return float(np.sum(np.linalg.norm(np.diff(g, axis=0), axis=1)))


def dvl_rejection_breakdown(r):
    """Where do gated DVL samples fall: tilted vehicle, just after a DVL gap, or neither."""
    acc = np.atleast_1d(r['accepted'].dvl).astype(bool)
    k = np.atleast_1d(r['dvl_k']).astype(int) - 1
    tilt = np.degrees(np.maximum(np.abs(vn_eul[k, 0]), np.abs(vn_eul[k, 1])))
    tk = k * float(log['dt'])
    since_gap = np.full(len(k), np.inf)
    last_gap_end = -np.inf
    for i in range(1, len(k)):
        if tk[i] - tk[i - 1] > GAP_S:
            last_gap_end = tk[i]
        since_gap[i] = tk[i] - last_gap_end
    lines = []
    for label, m in [('tilt <= 10 deg', tilt <= 10), ('tilt > 10 deg', tilt > 10),
                     ('< 2 s after a DVL gap', since_gap < 2), ('>= 2 s after a gap', since_gap >= 2)]:
        lines.append(f'  {label:24s} n={m.sum():5d}  rejected {100 * (~acc[m]).mean() if m.any() else float("nan"):5.1f}%')
    return '\n'.join(lines)


runs, extra = {}, {}
breakdown = ''
for v in ESKF_VARIANTS:
    f = DATA / f'{v}.mat'
    if not f.exists():
        continue
    r = loadmat(f, squeeze_me=True, struct_as_record=False)
    t = np.asarray(r['t_bag']) - t0
    x = np.asarray(r['x'])
    xy_enu = np.column_stack([x[:, 1], x[:, 0]])                     # NED (N, E) -> ENU (E, N)
    runs[v] = (t, xy_enu - xy_enu[0], -x[:, 2])
    eul = zyx_from_wxyz(x[:, 6:10])
    vn = vn_eul[np.clip(np.rint(np.asarray(r['t']) / float(log['dt'])).astype(int), 0, len(vn_eul) - 1)]
    d = np.degrees(wrap(eul - vn))
    extra[v] = {'roll_rms_deg': np.sqrt(np.mean(d[:, 0] ** 2)), 'pitch_rms_deg': np.sqrt(np.mean(d[:, 1] ** 2)),
                'yaw_rms_deg': np.sqrt(np.mean(d[:, 2] ** 2)),
                'dvl_rej_%': 100 * r['rejected'].dvl / len(np.atleast_1d(r['nis'].dvl)),
                'dvl_medNIS': float(np.median(r['nis'].dvl))}
    if v == MAIN:
        breakdown = f'DVL rejections in {MAIN} by condition (VN-200 tilt = max(|roll|, |pitch|)):\n' + dvl_rejection_breakdown(r)
e = pd.read_parquet(RUNS / 'ekf_E_C_plus_stale_zupt.parquet')
runs['python_E'] = (e['t_bag'].values - t0, e[['px', 'py']].values - e[['px', 'py']].values[0], e['pz'].values)

windows = [(20, 70), (240, 280), (round(dropout[0]), round(dropout[1])), (1050, 1150)]
keep = ~odom_gap & (odom_t < odom_cutoff)
rows = []
for name, (t, xy, z) in runs.items():
    on_odom = interp(t, xy, odom_t)
    row = {'estimator': name, 'path_m': round(path_len(t, xy), 1),
           f'rms_vs_odom_m(0-{odom_cutoff:.0f}s)': round(float(np.sqrt(np.mean(np.sum((on_odom[keep] - odom_xy[keep]) ** 2, axis=1)))), 2),
           'rms_vs_dvlDR_m': round(float(np.sqrt(np.mean(np.sum((interp(t, xy, dr_t) - dr_xy) ** 2, axis=1)))), 2)}
    for a, b in windows:
        p = interp(t, xy, np.array([a, b]))
        row[f'disp_{a}-{b}'] = np.round(p[1] - p[0], 2).tolist()
    row.update({k: round(v, 2) for k, v in extra.get(name, {}).items()})
    rows.append(row)
for label, (t, xy) in {'robot_localization': (odom_t, odom_xy), 'DVL dead-reckon': (dr_t, dr_xy)}.items():
    row = {'estimator': label, 'path_m': round(path_len(t, xy), 1)}
    for a, b in windows:
        row[f'disp_{a}-{b}'] = np.round(np.diff(interp(t, xy, np.array([a, b])), axis=0)[0], 2).tolist()
    rows.append(row)

table = pd.DataFrame(rows).set_index('estimator')
pd.set_option('display.width', 300); pd.set_option('display.max_columns', 30)
text = (table.to_string() + f'\n\nLongest DVL dropout: {dropout[0]:.1f}-{dropout[1]:.1f} s. '
        'Attitude columns: ESKF (raw gyro + accel, VN-200 yaw as heading aid) minus VN-200 internal attitude, RMS.\n'
        + breakdown + '\n')
print(text)
RESULTS.mkdir(exist_ok=True)
(RESULTS / 'eskf_table.txt').write_text(text)

colors = {'python_E': 'tab:green', 'eskf_ungated_zv': 'tab:blue', 'eskf_gated_zv': 'tab:red',
          'eskf_gated': 'tab:purple', 'eskf_loosegate_zv': 'tab:cyan'}


def plot_xy(ax, names, title):
    ax.plot(odom_xy[:, 0], odom_xy[:, 1], color='tab:orange', label='robot_localization', alpha=0.8)
    ax.plot(dr_xy[:, 0], dr_xy[:, 1], color='gray', ls='--', alpha=0.6, label='DVL dead-reckon')
    for name in [n for n in names if n in runs]:
        ax.plot(runs[name][1][:, 0], runs[name][1][:, 1], color=colors[name], lw=1, label=name)
    ax.set_aspect('equal'); ax.legend(fontsize=8); ax.set_title(title)
    ax.set_xlabel('east (m)'); ax.set_ylabel('north (m)')


fig, axs = plt.subplots(1, 3, figsize=(21, 7))
plot_xy(axs[0], ['python_E', 'eskf_ungated_zv'], 'Best ESKF vs references (ENU, start at origin)')
plot_xy(axs[1], ['eskf_ungated_zv', 'eskf_gated_zv'], 'Effect of 99% gating on the ESKF')
for name in [n for n in ('python_E', 'eskf_ungated_zv', 'eskf_gated') if n in runs]:
    t, xy, _ = runs[name]
    axs[2].plot(t, xy[:, 0], color=colors[name], lw=1, label=name)
axs[2].plot(odom_t, odom_xy[:, 0], color='tab:orange', lw=1, label='robot_localization')
axs[2].axvspan(*dropout, color='k', alpha=0.08, label='longest DVL dropout')
axs[2].set_xlabel('time (s)'); axs[2].set_ylabel('east (m)'); axs[2].legend(fontsize=8)
axs[2].set_title('East over time (eskf_gated: no zero-velocity aid)')
plt.tight_layout()
plt.savefig(RESULTS / 'eskf_comparison.png', dpi=120)
print('saved', RESULTS / 'eskf_comparison.png')
