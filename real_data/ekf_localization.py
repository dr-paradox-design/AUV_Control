"""
Loosely-coupled EKF for AUV localization on a recorded ROS 2 log.

World frame: odom, ENU (z-up), as in the ROS log. Note: the rest of this repository
(estimator_m, plant_m) uses NED / Fossen conventions.
Orientation comes straight from the VN-200 (/imu/data)
and is treated as known; the filter estimates position, velocity and (optionally)
body-frame accelerometer bias:
    x = [p(3), v(3)]            or   x = [p(3), v(3), b_a(3)]

Every event (IMU / depth / DVL) first propagates the state to its own timestamp using
the most recent IMU sample (zero-order hold), then applies its update.

Measurements:
    depth: -depth = pz + (R r_pressure)_z
    DVL:   v_dvl  = R^T v + w x r_dvl        (dvl_link has identity rotation wrt base_link)
"""
from pathlib import Path

import numpy as np
import pandas as pd
from scipy.spatial.transform import Rotation as Rot

from paths import DATA, EXTRACTED, RESULTS, RUNS, static_tf

G_WORLD = np.array([0.0, 0.0, 9.81])
R_DVL_LEVER, R_DVL_BODY = static_tf('dvl_link')    # base_link -> dvl_link from this run's /tf_static
R_PRESSURE_LEVER, _ = static_tf('pressure_link')   # base_link -> pressure_link
CHI2_1DOF_99 = 6.63
CHI2_3DOF_99 = 11.34
IMU_GAP_S = 0.5     # longer silences are recording gaps (all topics stop); 2026-11-05 log has none


def load():
    """IMU, DVL (rotated from dvl_link into body axes), depth and /odometry/filtered tables."""
    def rd(name):
        return pd.read_parquet(f'{EXTRACTED}/{name}.parquet').sort_values('t_bag').reset_index(drop=True)
    dvl = rd('dvl_raw')
    dvl[['vx', 'vy', 'vz']] = dvl[['vx', 'vy', 'vz']].values @ R_DVL_BODY.T
    return rd('imu_data'), dvl, rd('pressure_depth'), rd('odometry_filtered')


def depth_noise_std(depth):
    if len(depth) < 3:
        return 0.080                                # no depth in this run; value unused
    return float(np.std(np.diff(depth['depth'].values)) / np.sqrt(2))


def run_ekf(imu, dvl, depth, *, estimate_bias=False, lever_arms=True, gating=True,
            q_accel_std=0.6, q_bias_std=0.01, r_depth_std=None, r_dvl_std=0.05,
            max_consecutive_rejects=10, stale_dvl_zero_vel_std=None, stale_after=0.5,
            pseudo_period=0.2, legacy_timing=False):
    # legacy_timing reproduces the first version's error on purpose (variant A): the state is
    # propagated only at IMU events, so time before an interleaved depth/DVL event is lost.
    # stale_dvl_zero_vel_std: when DVL has been silent > stale_after s, apply a weak v≈0
    # pseudo-measurement. Assumes a slow / unpowered vehicle; replace with a thrust-driven
    # dynamics model for powered runs.
    if r_depth_std is None:
        r_depth_std = depth_noise_std(depth)

    n = 9 if estimate_bias else 6
    imu_t = imu['t_bag'].values
    imu_R = Rot.from_quat(imu[['qx', 'qy', 'qz', 'qw']].values).as_matrix()
    imu_a = imu[['ax', 'ay', 'az']].values
    imu_w = imu[['wx', 'wy', 'wz']].values
    dvl_t, dvl_v = dvl['t_bag'].values, dvl[['vx', 'vy', 'vz']].values
    dep_t, dep_z = depth['t_bag'].values, -depth['depth'].values

    src = np.concatenate([np.zeros(len(imu_t), int), np.ones(len(dep_t), int), np.full(len(dvl_t), 2)])
    idx = np.concatenate([np.arange(len(imu_t)), np.arange(len(dep_t)), np.arange(len(dvl_t))])
    tt = np.concatenate([imu_t, dep_t, dvl_t])
    order = np.lexsort((src, tt))   # IMU first on timestamp ties
    src, idx, tt = src[order], idx[order], tt[order]

    first_imu = np.argmax(src == 0)
    src, idx, tt = src[first_imu:], idx[first_imu:], tt[first_imu:]

    x = np.zeros(n)
    P = np.diag([1e-4] * 3 + [0.1] * 3 + ([0.05] * 3 if estimate_bias else []))
    I = np.eye(n)
    Rm, a_b, w_b = imu_R[0], imu_a[0], imu_w[0]
    t_prev = tt[0]
    rejects = {'depth': 0, 'dvl': 0}
    consec = {'depth': 0, 'dvl': 0}
    last_dvl_t = t_prev
    last_pseudo_t = t_prev
    n_pseudo = 0
    nis_dvl = []
    out = np.empty((len(tt), 1 + n))

    def update(x, P, H, z, h, Rn, kind, chi2):
        y = z - h
        S = H @ P @ H.T + Rn
        Sinv = np.linalg.inv(S)
        d2 = float(y @ Sinv @ y)
        if gating and d2 > chi2 and consec[kind] < max_consecutive_rejects:
            rejects[kind] += 1
            consec[kind] += 1
            return x, P, d2
        consec[kind] = 0
        K = P @ H.T @ Sinv
        IKH = I - K @ H
        return x + K @ y, IKH @ P @ IKH.T + K @ Rn @ K.T, d2

    for k in range(len(tt)):
        t = tt[k]
        dt = t - t_prev
        if legacy_timing and src[k] != 0:
            t_prev = t
            dt = 0.0
        if dt > IMU_GAP_S:                      # recording gap: no IMU data, do not propagate through it
            x[3:6] = 0.0
            P[3:6, :] = 0.0; P[:, 3:6] = 0.0
            P[3:6, 3:6] = np.eye(3) * 0.1 ** 2
            t_prev = t
            dt = 0.0
        if dt > 0:
            b = x[6:9] if estimate_bias else 0.0
            a_w = Rm @ (a_b - b) - G_WORLD
            F = I.copy()
            F[0:3, 3:6] = np.eye(3) * dt
            if estimate_bias:
                F[0:3, 6:9] = -0.5 * dt ** 2 * Rm
                F[3:6, 6:9] = -dt * Rm
            x[0:3] += x[3:6] * dt + 0.5 * a_w * dt ** 2
            x[3:6] += a_w * dt
            G = np.zeros((n, 3))
            G[0:3] = 0.5 * dt ** 2 * np.eye(3)
            G[3:6] = dt * np.eye(3)
            Q = G @ (np.eye(3) * q_accel_std ** 2) @ G.T
            if estimate_bias:
                Q[6:9, 6:9] += np.eye(3) * q_bias_std ** 2 * dt
            P = F @ P @ F.T + Q
            t_prev = t

        s, i = src[k], idx[k]
        if s == 0:
            Rm, a_b, w_b = imu_R[i], imu_a[i], imu_w[i]
            if (stale_dvl_zero_vel_std is not None and t - last_dvl_t > stale_after
                    and t - last_pseudo_t >= pseudo_period):
                H = np.zeros((3, n)); H[:, 3:6] = np.eye(3)
                y = -x[3:6]
                S = H @ P @ H.T + np.eye(3) * stale_dvl_zero_vel_std ** 2
                K = P @ H.T @ np.linalg.inv(S)
                IKH = I - K @ H
                x = x + K @ y
                P = IKH @ P @ IKH.T + K @ (np.eye(3) * stale_dvl_zero_vel_std ** 2) @ K.T
                last_pseudo_t = t
                n_pseudo += 1
        elif s == 1:
            H = np.zeros((1, n)); H[0, 2] = 1.0
            off = (Rm @ R_PRESSURE_LEVER)[2] if lever_arms else 0.0
            x, P, _ = update(x, P, H, np.array([dep_z[i]]), np.array([x[2] + off]),
                             np.array([[r_depth_std ** 2]]), 'depth', CHI2_1DOF_99)
        else:
            H = np.zeros((3, n)); H[:, 3:6] = Rm.T
            lever = np.cross(w_b, R_DVL_LEVER) if lever_arms else 0.0
            x, P, d2 = update(x, P, H, dvl_v[i], Rm.T @ x[3:6] + lever,
                              np.eye(3) * r_dvl_std ** 2, 'dvl', CHI2_3DOF_99)
            nis_dvl.append(d2)
            last_dvl_t = t
        out[k, 0] = t
        out[k, 1:] = x

    cols = ['t_bag', 'px', 'py', 'pz', 'vx', 'vy', 'vz'] + (['bax', 'bay', 'baz'] if estimate_bias else [])
    df = pd.DataFrame(out, columns=cols)
    info = {'rejects': rejects, 'n_dvl': len(dvl_t), 'n_depth': len(dep_t),
            'nis_dvl_median': float(np.median(nis_dvl)), 'r_depth_std': r_depth_std,
            'n_zero_vel_pseudo': n_pseudo}
    return df, info


VARIANTS = {
    'A_original': dict(lever_arms=False, gating=False, r_dvl_std=0.15, r_depth_std=0.02, legacy_timing=True),
    'B_timing_fix': dict(lever_arms=False, gating=False, r_dvl_std=0.15, r_depth_std=0.02),
    'C_lever_gate_tuned': dict(lever_arms=True, gating=True),
    'D_plus_accel_bias': dict(lever_arms=True, gating=True, estimate_bias=True),
    'E_C_plus_stale_zupt': dict(lever_arms=True, gating=True, stale_dvl_zero_vel_std=0.1),
    'F_D_plus_stale_zupt': dict(lever_arms=True, gating=True, estimate_bias=True, stale_dvl_zero_vel_std=0.1),
}

if __name__ == '__main__':
    imu, dvl, depth, odom = load()
    RUNS.mkdir(parents=True, exist_ok=True)
    lines = []
    for name, kw in VARIANTS.items():
        df, info = run_ekf(imu, dvl, depth, **kw)
        df.iloc[::4].reset_index(drop=True).to_parquet(RUNS / f'ekf_{name}.parquet')
        line = (f"{name}: rejected depth {info['rejects']['depth']}/{info['n_depth']}, "
                f"DVL {info['rejects']['dvl']}/{info['n_dvl']}; median DVL NIS {info['nis_dvl_median']:.3f} "
                f"(chi2(3) median 2.37); depth sigma {info['r_depth_std']:.3f} m; "
                f"zero-velocity updates {info['n_zero_vel_pseudo']}")
        if 'bax' in df:
            line += '; final accel bias (m/s^2) ' + str(df[['bax', 'bay', 'baz']].iloc[-1].round(3).to_dict())
        print(line)
        lines.append(line)
    RESULTS.mkdir(parents=True, exist_ok=True)
    (RESULTS / 'variant_diagnostics.txt').write_text('\n'.join(lines) + '\n')
