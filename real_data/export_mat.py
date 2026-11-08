"""Export the extracted log to a .mat file that estimator_m/load_real_log.m reads.

Run after extract.py. Writes real_data/data/real_log.mat (gitignored).

Conventions (match estimator_m):
- body FRD, world NED; quaternion Hamilton [w x y z], body -> NED;
- IMU as delta-angle and delta-velocity on a fixed grid (rates interpolated at interval midpoints);
- measurement k = 1-based index of the grid sample at or nearest the measurement time.
The ROS log is body FLU / world ENU: FLU -> FRD is diag(1,-1,-1); ENU -> NED swaps x,y and negates z.
"""
from pathlib import Path

import numpy as np
import pandas as pd
from scipy.io import savemat
from scipy.spatial.transform import Rotation as Rot

HERE = Path(__file__).resolve().parent
EXTRACTED = HERE / 'data' / 'extracted'
OUT = HERE / 'data' / 'real_log.mat'

DT = 1.0 / 80.0                       # IMU rate in the log
HEADING_PERIOD = 0.1                  # VN-200 yaw used as a 10 Hz heading measurement
FLU_TO_FRD = np.diag([1.0, -1.0, -1.0])
ENU_TO_NED = np.array([[0.0, 1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, -1.0]])


def rd(name):
    return pd.read_parquet(EXTRACTED / f'{name}.parquet').sort_values('t_bag').reset_index(drop=True)


imu, dvl, depth = rd('imu_data'), rd('dvl_raw'), rd('pressure_depth')
t_imu = imu['t_bag'].values
t0 = t_imu[0]
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


dvl_k = k_of(dvl['t_bag'].values)
dvl_z = dvl[['vx', 'vy', 'vz']].values @ FLU_TO_FRD.T                     # dvl_link = base_link rotation
dep_k = k_of(depth['t_bag'].values)
dep_z = depth['depth'].values                                              # positive down = NED z
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
    'q0': q0, 'depth0': dep_z[0],
    'vn_euler_grid': eul_vn,                                                # VN-200 roll, pitch, yaw on the grid
}, do_compression=True)
print(f'n={n} steps of {DT}s ({n * DT:.1f} s); dvl {len(dvl_k)}, depth {len(dep_k)}, heading {len(hd_k)}')
print(f'q0 [w x y z] = {np.round(q0, 4)}, |w| mean body rate {np.degrees(np.linalg.norm(w_frd, axis=1).mean()):.1f} deg/s')
print(f'specific force at rest check: median a_z (FRD) = {np.median(a_frd[:, 2]):.3f} m/s^2 (expect about -9.8)')
print(f'wrote {OUT}')
