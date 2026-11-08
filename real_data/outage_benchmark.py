"""Artificial DVL-outage benchmark on the recorded log (no ground truth needed).

    python outage_benchmark.py windows [L]   # pick windows of L s (default 20) -> data/outage_windows_<L>.mat
    (octave) setup_paths; run_outage_benchmark(L)   # ESKF variants with those windows removed
    python outage_benchmark.py report [L]    # Python EKF variants + scoring -> results/outage_table_<L>s.txt

Each window is L s (default 20) with the DVL removed, placed where the real DVL stream is continuous, one every 60 s
so the filters recover in between. The score is the horizontal difference between an estimator's
displacement over the window and the DVL dead-reckoning displacement over the same window (which uses
the removed DVL data). It measures how well each filter bridges a DVL outage, not absolute accuracy.
"""
import sys
from pathlib import Path

import numpy as np
import pandas as pd
from scipy.io import loadmat, savemat
from scipy.spatial.transform import Rotation as Rot

from ekf_localization import EXTRACTED, R_DVL_LEVER, load, run_ekf

HERE = Path(__file__).resolve().parent
DATA, RESULTS = HERE / 'data', HERE / 'results'
EVERY, FIRST = 60.0, 210.0
LEN = float(sys.argv[2]) if len(sys.argv) > 2 else 20.0
TAG = f'{LEN:g}'

imu = pd.read_parquet(EXTRACTED / 'imu_data.parquet').sort_values('t_bag')
dvl = pd.read_parquet(EXTRACTED / 'dvl_raw.parquet').sort_values('t_bag')
t0 = imu['t_bag'].iloc[0]
tv = dvl['t_bag'].values - t0


def pick_windows():
    gaps = [(tv[i], tv[i + 1]) for i in np.where(np.diff(tv) > 1.0)[0]]
    out = []
    a = FIRST
    while a + LEN < tv[-1] - 5:
        lo, hi = a - 10, a + LEN + 5
        if not any(g1 > lo and g0 < hi for g0, g1 in gaps):
            out.append((a, a + LEN))
        a += EVERY
    return np.array(out)


def dvl_dr():
    t = dvl['t_bag'].values
    j = np.clip(imu['t_bag'].searchsorted(t), 0, len(imu) - 1)
    R = Rot.from_quat(imu[['qx', 'qy', 'qz', 'qw']].values[j])
    w = imu[['wx', 'wy', 'wz']].values[j]
    v = dvl[['vx', 'vy', 'vz']].values - np.cross(w, R_DVL_LEVER)
    dt = np.diff(t, prepend=t[0])
    dt = np.where(dt > 1.0, 0.0, dt)
    return t - t0, np.cumsum(R.apply(v) * dt[:, None], axis=0)[:, :2]


def disp(t, xy, a, b):
    return np.array([np.interp(b, t, xy[:, i]) - np.interp(a, t, xy[:, i]) for i in range(2)])


def eskf_xy(f):
    r = loadmat(f, squeeze_me=True)
    x = np.asarray(r['x'])
    return np.asarray(r['t_bag']) - t0, np.column_stack([x[:, 1], x[:, 0]])      # NED -> ENU


def report():
    windows = loadmat(DATA / f'outage_windows_{TAG}.mat')['windows']
    imu_df, dvl_df, depth_df, _ = load()
    td = dvl_df['t_bag'].values - t0
    keep = np.ones(len(td), bool)
    for a, b in windows:
        keep &= ~((td >= a) & (td < b))
    dvl_cut = dvl_df[keep].reset_index(drop=True)

    runs = {}
    for name, kw in {'python_E (99% gate)': dict(gating=True), 'python_E_ungated': dict(gating=False)}.items():
        df, _ = run_ekf(imu_df, dvl_cut, depth_df, lever_arms=True, stale_dvl_zero_vel_std=0.1, **kw)
        runs[name] = (df['t_bag'].values - t0, df[['px', 'py']].values)
    for v in ['eskf_ungated', 'eskf_ungated_zv', 'eskf_robust_zv']:
        f = DATA / f'outage_{v}_{TAG}.mat'
        if f.exists():
            runs[v + ' (outages)'] = eskf_xy(f)
    for v in ['eskf_ungated_zv', 'eskf_robust_zv']:
        f = DATA / f'{v}.mat'
        if f.exists():
            runs[v + ' (full DVL, floor)'] = eskf_xy(f)

    drt, drxy = dvl_dr()
    ref = np.array([disp(drt, drxy, a, b) for a, b in windows])
    moved = np.linalg.norm(ref, axis=1)
    rows = []
    for name, (t, xy) in runs.items():
        err = np.linalg.norm(np.array([disp(t, xy, a, b) for a, b in windows]) - ref, axis=1)
        rows.append({'estimator': name, 'median_err_m': round(float(np.median(err)), 3),
                     'p90_err_m': round(float(np.percentile(err, 90)), 3), 'max_err_m': round(float(err.max()), 3),
                     'mean_err_%_of_motion': round(float(100 * err.sum() / moved.sum()), 1)})
    table = pd.DataFrame(rows).set_index('estimator')
    text = (f'{len(windows)} artificial DVL outages of {LEN:.0f} s ({windows[0, 0]:.0f}-{windows[-1, 1]:.0f} s). '
            f'Error = |estimator displacement - DVL dead-reckoning displacement| over each window.\n'
            f'Vehicle motion per window (DVL dead reckoning): median {np.median(moved):.2f} m, max {moved.max():.2f} m.\n\n'
            + table.to_string() + '\n')
    print(text)
    RESULTS.mkdir(exist_ok=True)
    (RESULTS / f'outage_table_{TAG}s.txt').write_text(text)


if __name__ == '__main__':
    if sys.argv[1:2] == ['windows']:
        w = pick_windows()
        savemat(DATA / f'outage_windows_{TAG}.mat', {'windows': w})
        print(f'{len(w)} windows: ' + ', '.join(f'{a:.0f}-{b:.0f}' for a, b in w))
    elif sys.argv[1:2] == ['report']:
        report()
    else:
        sys.exit(__doc__)
