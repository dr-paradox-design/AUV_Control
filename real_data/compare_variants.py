"""Compare EKF variants against /odometry/filtered and DVL dead reckoning.

Run after extract.py and ekf_localization.py.
Writes results/variants_table.txt and results/variants_comparison.png.
"""
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as Rot

from ekf_localization import EXTRACTED, RUNS, VARIANTS, R_DVL_LEVER

RESULTS = EXTRACTED.parent.parent / 'results'
RESULTS.mkdir(exist_ok=True)
GAP_S = 1.0

imu = pd.read_parquet(EXTRACTED / 'imu_data.parquet').sort_values('t_bag')
dvl = pd.read_parquet(EXTRACTED / 'dvl_raw.parquet').sort_values('t_bag')
odom = pd.read_parquet(EXTRACTED / 'odometry_filtered.parquet').sort_values('t_bag')
t0 = imu['t_bag'].iloc[0]

odom_t = odom['t_bag'].values - t0
odom_xy = odom[['x', 'y']].values - odom[['x', 'y']].values[0]
odom_dt = np.diff(odom_t, prepend=odom_t[0])
odom_gap = odom_dt > GAP_S
odom_gap_spans = [(odom_t[i - 1], odom_t[i]) for i in np.where(odom_gap)[0]]
odom_cutoff = odom_gap_spans[0][0] if odom_gap_spans else odom_t[-1]


def dvl_dead_reckoning():
    """Integrate lever-arm-corrected DVL velocity, rotated by the nearest IMU orientation. Gaps are skipped."""
    t = dvl['t_bag'].values
    j = np.clip(imu['t_bag'].searchsorted(t), 0, len(imu) - 1)
    R = Rot.from_quat(imu[['qx', 'qy', 'qz', 'qw']].values[j])
    w = imu[['wx', 'wy', 'wz']].values[j]
    v_body = dvl[['vx', 'vy', 'vz']].values - np.cross(w, R_DVL_LEVER)
    dt = np.diff(t, prepend=t[0])
    dt = np.where(dt > GAP_S, 0.0, dt)
    return t - t0, np.cumsum(R.apply(v_body) * dt[:, None], axis=0)[:, :2], dt


dr_t, dr_xy, dvl_dt = dvl_dead_reckoning()
dvl_gap_idx = np.where(np.diff(dr_t) > GAP_S)[0]
longest = dvl_gap_idx[np.argmax(np.diff(dr_t)[dvl_gap_idx])]
dvl_dropout = (dr_t[longest], dr_t[longest + 1])

runs = {}
for name in VARIANTS:
    d = pd.read_parquet(RUNS / f'ekf_{name}.parquet')
    runs[name] = (d['t_bag'].values - t0, d[['px', 'py']].values, d['pz'].values)


def interp(t_src, xy, t):
    return np.column_stack([np.interp(t, t_src, xy[:, i]) for i in range(xy.shape[1])])


GRID = np.arange(0.0, imu['t_bag'].iloc[-1] - t0, 0.5)   # common 2 Hz grid so path lengths are comparable


def path_len(t, xy):
    g = interp(t, xy, GRID[(GRID >= t[0]) & (GRID <= t[-1])])
    return float(np.sum(np.linalg.norm(np.diff(g, axis=0), axis=1)))


windows = [(20, 70), (240, 280), (round(dvl_dropout[0]), round(dvl_dropout[1])), (1050, 1150)]
keep = ~odom_gap & (odom_t < odom_cutoff)
rows = []
for name, (t, xy, z) in runs.items():
    on_odom = interp(t, xy, odom_t)
    rms_odom = float(np.sqrt(np.mean(np.sum((on_odom[keep] - odom_xy[keep]) ** 2, axis=1))))
    on_dr = interp(t, xy, dr_t)
    rms_dr = float(np.sqrt(np.mean(np.sum((on_dr - dr_xy) ** 2, axis=1))))
    r = {'variant': name, 'path_m': round(path_len(t, xy), 1),
         f'rms_vs_odom_m(0-{odom_cutoff:.0f}s)': round(rms_odom, 2), 'rms_vs_dvlDR_m': round(rms_dr, 2),
         'z_step_noise_cm': round(100 * float(np.std(np.diff(z))), 2)}
    for a_, b_ in windows:
        p = interp(t, xy, np.array([a_, b_]))
        r[f'disp_{a_}-{b_}'] = np.round(p[1] - p[0], 2).tolist()
    rows.append(r)

for label, (t, xy) in {'odom': (odom_t, odom_xy), 'DVL dead-reckon': (dr_t, dr_xy)}.items():
    r = {'variant': label, 'path_m': round(path_len(t, xy), 1)}
    for a_, b_ in windows:
        r[f'disp_{a_}-{b_}'] = np.round(np.diff(interp(t, xy, np.array([a_, b_])), axis=0)[0], 2).tolist()
    rows.append(r)

table = pd.DataFrame(rows).set_index('variant')
pd.set_option('display.width', 250)
pd.set_option('display.max_columns', 20)
text = (table.to_string()
        + f'\n\nLongest DVL dropout: {dvl_dropout[0]:.1f}-{dvl_dropout[1]:.1f} s'
        + f'\nOdometry gaps: ' + ', '.join(f'{a:.1f}-{b:.1f} s' for a, b in odom_gap_spans)
        + '\nDisplacements are [dx, dy] in m. Path length on a common 2 Hz grid.\n')
print(text)
(RESULTS / 'variants_table.txt').write_text(text)

fig, axs = plt.subplots(1, 2, figsize=(15, 7))
axs[0].plot(odom_xy[:, 0], odom_xy[:, 1], color='tab:orange', alpha=0.8, label='/odometry/filtered')
axs[0].plot(dr_xy[:, 0], dr_xy[:, 1], color='gray', alpha=0.6, ls='--', label='DVL dead-reckon')
for name, c in [('A_original', 'tab:red'), ('E_C_plus_stale_zupt', 'tab:blue')]:
    t, xy, _ = runs[name]
    axs[0].plot(xy[:, 0], xy[:, 1], color=c, label=name, lw=1)
axs[0].set_aspect('equal'); axs[0].legend(); axs[0].set_title('XY trajectory (all start at origin)')
axs[0].set_xlabel('x (m)'); axs[0].set_ylabel('y (m)')

for name, c in [('A_original', 'tab:red'), ('B_timing_fix', 'tab:green'), ('E_C_plus_stale_zupt', 'tab:blue')]:
    t, xy, _ = runs[name]
    axs[1].plot(t, xy[:, 0], color=c, lw=1, label=f'{name} x')
axs[1].plot(odom_t, odom_xy[:, 0], color='tab:orange', lw=1, label='odom x')
axs[1].plot(dr_t, dr_xy[:, 0], color='gray', ls='--', lw=1, label='DVL-DR x')
axs[1].axvspan(*dvl_dropout, color='k', alpha=0.08, label='longest DVL dropout')
for i, (a_, b_) in enumerate(odom_gap_spans):
    axs[1].axvspan(a_, b_, color='tab:orange', alpha=0.1, label='odom gaps' if i == 0 else None)
axs[1].set_xlabel('time (s)'); axs[1].set_ylabel('x (m)'); axs[1].legend(fontsize=8); axs[1].set_title('x over time')
plt.tight_layout()
plt.savefig(RESULTS / 'variants_comparison.png', dpi=120)
print(f'saved {RESULTS / "variants_comparison.png"}')
