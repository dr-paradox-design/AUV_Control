"""Export an extracted log to the .mat file that estimator_m/load_real_log.m reads.

Run after extract.py (set REAL_DATA_RUN for a run other than the 2026-11-05 log).
Writes <run data>/real_log.mat (gitignored).

Conventions (match estimator_m):
- body FRD, world NED; quaternion Hamilton [w x y z], body -> NED;
- IMU as delta-angle and delta-velocity on a fixed grid at the run's IMU rate (rates interpolated at
  interval midpoints; through recording gaps this interpolation is a guess, so gaps are saved as imu_gaps);
- DVL velocity rotated from dvl_link into body axes with the run's /tf_static, so the filter uses R_bd = I;
- measurement k = 1-based index of the grid sample nearest the measurement time.
The ROS log is body FLU / world ENU: FLU -> FRD is diag(1,-1,-1); ENU -> NED swaps x,y and negates z.
"""
import numpy as np
import pandas as pd
from scipy.io import savemat
from scipy.spatial.transform import Rotation as Rot

from paths import DATA, EXTRACTED, static_tf

OUT = DATA / 'real_log.mat'
HEADING_PERIOD = 0.1                  # VN-200 yaw used as a 10 Hz heading measurement
GAP_S = 0.5
FLU_TO_FRD = np.diag([1.0, -1.0, -1.0])
ENU_TO_NED = np.array([[0.0, 1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, -1.0]])


def rd(name):
    return pd.read_parquet(EXTRACTED / f'{name}.parquet').sort_values('t_bag').reset_index(drop=True)


imu, dvl, depth = rd('imu_data'), rd('dvl_raw'), rd('pressure_depth')
t_imu = imu['t_bag'].values
t0 = t_imu[0]
gi = np.where(np.diff(t_imu) > GAP_S)[0]
imu_gaps = np.array([[t_imu[i] - t0, t_imu[i + 1] - t0] for i in gi]).reshape(-1, 2)
live = (t_imu[-1] - t0) - (imu_gaps[:, 1] - imu_gaps[:, 0]).sum()
DT = 1.0 / round((len(t_imu) - 1) / live)    # sample count over time with data (stamps arrive in bursts)
n = int(np.floor((t_imu[-1] - t0) / DT))
grid = t0 + np.arange(n + 1) * DT
mid = grid[:-1] + 0.5 * DT

w_frd = imu[['wx', 'wy', 'wz']].values @ FLU_TO_FRD.T
a_frd = imu[['ax', 'ay', 'az']].values @ FLU_TO_FRD.T
interp = lambda t, x: np.column_stack([np.interp(t, t_imu, x[:, i]) for i in range(3)])
dtheta = interp(mid, w_frd) * DT
dvel = interp(mid, a_frd) * DT

R_enu_flu = Rot.from_quat(imu[['qx', 'qy', 'qz', 'qw']].values).as_matrix()
R_ned_frd = ENU_TO_NED @ R_enu_flu @ FLU_TO_FRD
rot_nb = Rot.from_matrix(R_ned_frd)
yaw_vn = np.arctan2(R_ned_frd[:, 1, 0], R_ned_frd[:, 0, 0])             # Fossen zyx yaw


def k_of(t):
    return np.clip(np.rint((t - t0) / DT).astype(int) + 1, 1, n + 1)


_, R_bd = static_tf('dvl_link')                                            # dvl_link axes in base_link (FLU)
dvl_k = k_of(dvl['t_bag'].values)
dvl_z = (dvl[['vx', 'vy', 'vz']].values @ R_bd.T) @ FLU_TO_FRD.T         # into body axes, then FRD
dep_k = k_of(depth['t_bag'].values) if len(depth) else np.zeros(0, int)
dep_z = depth['depth'].values if len(depth) else np.zeros(0)               # positive down = NED z
hd_t = np.arange(t0, t_imu[-1], HEADING_PERIOD)
hd_i = np.clip(np.searchsorted(t_imu, hd_t), 0, len(t_imu) - 1)
hd_k = k_of(t_imu[hd_i])
hd_z = yaw_vn[hd_i]

q0 = rot_nb[0].as_quat()                                                    # scipy: [x y z w]
q0 = np.array([q0[3], q0[0], q0[1], q0[2]])

g_idx = np.clip(np.searchsorted(t_imu, grid), 0, len(t_imu) - 1)
eul_vn = np.column_stack([np.arctan2(R_ned_frd[g_idx, 2, 1], R_ned_frd[g_idx, 2, 2]),
                          -np.arcsin(np.clip(R_ned_frd[g_idx, 2, 0], -1, 1)),
                          yaw_vn[g_idx]])

savemat(OUT, {
    't0_bag': t0, 'dt': DT, 'n': n,
    'dtheta': dtheta, 'dvel': dvel,
    'dvl_k': dvl_k.astype(float), 'dvl_z': dvl_z,
    'depth_k': dep_k.astype(float), 'depth_z': dep_z,
    'heading_k': hd_k.astype(float), 'heading_z': hd_z,
    'q0': q0, 'depth0': float(dep_z[0]) if len(dep_z) else 0.0,
    'vn_euler_grid': eul_vn,                                                # VN-200 roll, pitch, yaw on the grid
    'imu_gaps': imu_gaps,                                                   # [start, end] s from t0
}, do_compression=True)
print(f'n={n} steps of {DT:.5f} s ({1/DT:.0f} Hz, {n * DT:.1f} s); dvl {len(dvl_k)}, depth {len(dep_k)}, heading {len(hd_k)}')
print(f'DVL rotation into body: {Rot.from_matrix(R_bd).as_euler("ZYX", degrees=True).round(1)} deg (z, y, x)')
print(f'IMU gaps > {GAP_S} s: {np.round(imu_gaps, 1).tolist()}')
print(f'specific force at rest check: median a_z (FRD) = {np.median(a_frd[:, 2]):.3f} m/s^2 (expect about -9.8)')
print(f'wrote {OUT}')
