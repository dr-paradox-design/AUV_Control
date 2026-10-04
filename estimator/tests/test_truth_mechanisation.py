"""Truth trajectory and mechanisation checks."""
import numpy as np

from estimator import config as C
from estimator.eskf import State, propagate_nominal
from estimator.rotations import (euler_to_quat, euler_to_R, quat_exp, quat_log, quat_mul,
                                 quat_to_R, R_to_euler, skew, vn_quat_to_hamilton)
from estimator.sim import get_truth
from estimator.truth import kinematics


def test_rotation_conventions():
    phi, th, psi = 0.3, -0.4, 2.0
    R = quat_to_R(euler_to_quat(phi, th, psi))
    np.testing.assert_allclose(R, euler_to_R(phi, th, psi), atol=1e-12)
    np.testing.assert_allclose(R.T @ R, np.eye(3), atol=1e-12)
    assert np.isclose(np.linalg.det(R), 1.0)
    np.testing.assert_allclose(R_to_euler(R), (phi, th, psi), atol=1e-12)
    rv = np.array([0.1, -0.2, 0.3])
    np.testing.assert_allclose(quat_log(quat_exp(rv)), rv, atol=1e-12)
    np.testing.assert_allclose(vn_quat_to_hamilton([1, 2, 3, 4]), [4, 1, 2, 3])


def test_body_rates_match_attitude_derivative():
    """R_dot = R [w_nb]x, checked by finite differences of the truth attitude."""
    t, h = 160.0, 1e-5      # inside a turn, with roll/pitch motion
    k0, kp, km = kinematics(np.array([t])), kinematics(np.array([t + h])), kinematics(np.array([t - h]))
    R = lambda k: euler_to_R(k["phi"][0], k["theta"][0], k["psi"][0])
    Rdot = (R(kp) - R(km)) / (2 * h)
    np.testing.assert_allclose(Rdot, R(k0) @ skew(k0["w_nb"][0]), atol=1e-8)


def test_depth_integral_consistent():
    tr = get_truth()
    assert np.max(np.abs(tr._pz_check - tr.p[:, 2])) < 1e-6


def test_noise_free_mechanisation_tracks_truth():
    """Noise- and bias-free increments through the filter's own mechanisation,
    no aiding, full 600 s. Any sign or frame error would show as drift."""
    tr = get_truth()
    x = State(tr.p[0], tr.v[0], tr.q[0], np.zeros(3), np.zeros(3), 0.0)
    for k in range(tr.n):
        x = propagate_nominal(x, tr.dtheta[k], tr.dvel[k], tr.dt, tr.w_ie, tr.g, "rw")[0]
    pos_err = np.linalg.norm(x.p - tr.p[-1])
    att_err = np.linalg.norm(quat_log(quat_mul(np.array([1, -1, -1, -1]) * x.q, tr.q[-1])))
    print(f"600 s noise-free: position error {pos_err:.3e} m, attitude error {att_err:.3e} rad")
    assert pos_err < 0.05
    assert att_err < 1e-6
