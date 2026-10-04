"""Analytic Jacobians vs finite differences (log 12.4, 12.7, 12.8, 12.9).

The measurement Jacobians and the lever-arm / Earth-rate terms were derived by
hand and are not in Sola or MSS, so this is their primary check.
"""
import numpy as np
import pytest

from estimator import config as C
from estimator.eskf import (MEAS, NX, State, error_transition, propagate_nominal)
from estimator.rotations import euler_to_quat, wrap

W_IE, G = C.earth_vectors()


def _state(seed=0):
    rng = np.random.default_rng(seed)
    q = euler_to_quat(0.3, -0.4, 2.0)   # large angles so small-angle shortcuts would fail
    return State(rng.normal(0, 5, 3), rng.normal(0, 1, 3), q,
                 rng.normal(0, 0.01, 3), rng.normal(0, 0.01, 3), 0.05)


def _num_jac(fun, x, eps=1e-6):
    f0 = fun(x)
    J = np.zeros((f0.size, NX))
    for i in range(NX):
        d = np.zeros(NX); d[i] = eps
        fp, fm = fun(x.oplus(d)), fun(x.oplus(-d))
        diff = fp - fm
        J[:, i] = diff / (2 * eps)
    return J


@pytest.mark.parametrize("name", list(MEAS))
def test_measurement_jacobian(name):
    x = _state()
    w_ib = np.array([0.1, -0.2, 0.3])
    hfun = MEAS[name][0]
    _, H = hfun(x, w_ib, W_IE)

    # the gyro bias enters the lever-arm terms through w_nb = w_ib_meas - b_g
    w_meas = w_ib + x.bg

    def h_of(xx):
        h, _ = hfun(xx, w_meas - xx.bg, W_IE)
        return h

    Hn = _num_jac(h_of, x)
    if name == "heading":
        Hn = wrap(Hn)
    np.testing.assert_allclose(H, Hn, atol=1e-7, rtol=1e-5)


@pytest.mark.parametrize("bias_model", ["rw", "gm"])
def test_transition_matrix(bias_model):
    """F from finite differences of the full nonlinear propagation.

    F is a first-order (Euler) discretisation (Sola 269), so it differs from the
    exact Jacobian by O(dt^2). Check that the mismatch shrinks ~4x when dt halves.
    """
    x = _state(1)
    w_true = np.array([0.05, -0.1, 0.2])
    f_true = np.array([0.3, -0.2, -9.6])

    def mismatch(dt):
        dth, dv = (w_true + x.bg) * dt, (f_true + x.ba) * dt
        xn, w, a, R = propagate_nominal(x, dth, dv, dt, W_IE, G, bias_model)
        F = error_transition(R, w, a, dt, W_IE, bias_model)

        def prop(xx):
            return propagate_nominal(xx, dth, dv, dt, W_IE, G, bias_model)[0].ominus(xn)
        Fn = _num_jac(prop, x, eps=1e-7)
        return np.max(np.abs(F - Fn))

    # largest neglected term is the position row's 0.5 R [a]x dt^2 (about |f| dt^2 / 2)
    dt = 0.01
    e1, e2 = mismatch(2 * dt), mismatch(dt)
    assert e2 < 1.1 * 0.5 * np.linalg.norm(f_true) * dt ** 2, e2
    assert 3.5 < e1 / e2 < 4.5, (e1, e2)
