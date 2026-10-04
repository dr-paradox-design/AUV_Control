"""Estimator study parameters.

EVERY VALUE IN THIS FILE IS ASSUMED until replaced by a measured or datasheet value.
Each entry notes where the assumed number came from. Replace with:
  - VN-200 datasheet for the exact variant (rugged / SMD) and an Allan-variance test,
  - Water Linked A50 datasheet,
  - the analog pressure sensor datasheet and bench calibration,
  - measured lever arms and the DVL mounting rotation.

Units: SI (m, s, rad) unless the name says otherwise.
"""
import numpy as np

DEG = np.pi / 180.0
G0 = 9.80665  # standard gravity, only used to convert "mg" specs

# --- Earth --------------------------------------------------------------------
EARTH = dict(
    lat_deg=20.0,               # ASSUMED dive-site latitude
    omega_ie=7.292115e-5,       # Earth rotation rate [rad/s] (WGS-84 constant)
)

# --- VN-200 IMU ---------------------------------------------------------------
IMU = dict(
    rate_hz=100.0,                          # ASSUMED filter IMU rate (VN-200 can output up to 800 Hz)
    gyro_nd=0.0035 * DEG,                   # [rad/s/sqrt(Hz)] VN-200 datasheet snippet, UNVERIFIED
    accel_nd=0.14e-3 * G0,                  # [m/s^2/sqrt(Hz)] VN-200 datasheet snippet, UNVERIFIED
    gyro_bias_sigma=5.0 * DEG / 3600.0,     # [rad/s] in-run bias stability, snippet, UNVERIFIED
    accel_bias_sigma=0.04e-3 * G0,          # [m/s^2] in-run bias stability, snippet, UNVERIFIED
    bias_tau=300.0,                         # [s] Gauss-Markov time constant, ASSUMED (needs Allan variance)
)

# --- Water Linked DVL A50 -----------------------------------------------------
DVL = dict(
    rate_hz=10.0,                       # ASSUMED (A50 datasheet not yet read)
    sigma=0.01,                         # [m/s] per axis, ASSUMED
    lever=np.array([0.20, 0.0, 0.15]),  # [m] body FRD, ASSUMED
    R_bd=np.eye(3),                     # body -> DVL frame rotation, ASSUMED aligned
    min_depth=1.0,                      # [m] treat as invalid near the surface, ASSUMED
    seabed_depth=30.0,                  # [m] flat seabed in the scenario, ASSUMED
    max_altitude=50.0,                  # [m] bottom-lock limit, ASSUMED (unverified A50 value)
    dropout=(300.0, 360.0),             # [s] forced loss of bottom lock in the scenario
)

# --- Analog pressure (depth) sensor -----------------------------------------
DEPTH = dict(
    rate_hz=20.0,                       # ASSUMED
    full_scale=30.0,                    # [m] user value
    adc_counts=1024,                    # 10-bit ADC, user value (assumes full ADC span used)
    sigma=0.01,                         # [m] sensor noise before quantisation, ASSUMED
    lever=np.array([-0.10, 0.0, 0.05]), # [m] body FRD, ASSUMED
    bias_sigma0=0.02,                   # [m] residual zero offset after surface zeroing, ASSUMED
    bias_rw=1e-4,                       # [m/sqrt(s)] bias random walk (temperature drift), ASSUMED
)

# --- VN-200 GNSS (surface only) ----------------------------------------------
GNSS = dict(
    rate_hz=5.0,                          # ASSUMED
    sigma_pos=np.array([1.0, 1.0, 1.5]),  # [m] N, E, D, datasheet snippet, UNVERIFIED
    sigma_vel=0.05,                       # [m/s] datasheet snippet, UNVERIFIED
    lever=np.array([0.0, 0.0, -0.30]),    # [m] antenna 0.3 m above IMU, ASSUMED
)

# --- Magnetometer heading -----------------------------------------------------
HEADING = dict(
    rate_hz=10.0,       # ASSUMED
    sigma=2.0 * DEG,    # [rad] after hard/soft-iron calibration, ASSUMED
)

# --- Initial uncertainty (1 sigma) --------------------------------------------
INIT = dict(
    pos=1.0,                # [m] ASSUMED
    vel=0.1,                # [m/s] ASSUMED
    roll_pitch=1.0 * DEG,   # [rad] ASSUMED
    yaw=5.0 * DEG,          # [rad] ASSUMED
)

# --- Scenario -----------------------------------------------------------------
SCENARIO = dict(
    duration=600.0,                     # [s]
    surge=0.8,                          # [m/s] cruise speed
    dive_depth=10.0,                    # [m]
    dive_start=60.0, dive_len=50.0,     # [s]
    ascent_start=500.0, ascent_len=50.0,
    turn_times=(150.0, 250.0, 350.0, 450.0),
    turn_signs=(+1, -1, +1, -1),
    turn_len=30.0,                      # [s] each turn is 180 deg
)


def normal_gravity(lat_rad):
    """WGS-84 normal gravity on the ellipsoid (Somigliana form) [m/s^2]."""
    s2 = np.sin(lat_rad) ** 2
    return 9.7803253359 * (1 + 0.00193185265241 * s2) / np.sqrt(1 - 0.00669437999013 * s2)


def earth_vectors():
    """Earth rate in NED and gravity vector in NED (down positive)."""
    lat = EARTH["lat_deg"] * DEG
    w = EARTH["omega_ie"]
    omega_ie_n = w * np.array([np.cos(lat), 0.0, -np.sin(lat)])
    g_n = np.array([0.0, 0.0, normal_gravity(lat)])
    return omega_ie_n, g_n
