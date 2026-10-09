"""Independent check of report eqs. (3.8), (3.9), (3.11) from Kirchhoff's equations.

Written separately from the MATLAB code and its tests. Needs only numpy.
  1. M_RB (3.8) must be the Hessian of the rigid-body kinetic energy
        T = 1/2 m |v + w x rg|^2 + 1/2 w' Ig w      (no M or C used).
  2. With P = dT/dv and L = dT/dw for T = 1/2 nu' M nu, Kirchhoff's equations give
        F  = P_dot + w x P,     Mo = L_dot + w x L + v x P.
     M nu_dot + C(nu) nu, with C from (3.9) and (3.11) as printed, must equal [F; Mo].
Each check is also given a deliberately wrong matrix and must reject it.
Usage: python kirchhoff_check.py
"""
import sys
import numpy as np

rng = np.random.default_rng(5)
S = lambda a: np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])

# parameters: Wu (2018) Tables 5.1, 5.2, with a general CG offset so no term vanishes
m = 11.5
rg = np.array([0.03, -0.02, 0.02])
Ig = np.array([[0.16, 0.01, -0.02], [0.01, 0.18, 0.015], [-0.02, 0.015, 0.20]])
A = np.diag([5.5, 12.7, 14.57, 0.12, 0.12, 0.12])        # M_A = -diag(X_udot ...)
Io = Ig - m * S(rg) @ S(rg)


def M_RB():                                               # (3.8)
    return np.block([[m * np.eye(3), -m * S(rg)], [m * S(rg), Io]])


def C_RB(nu, sign=1.0):                                   # (3.9); sign=-1 flips the m S(S(v) rg) term
    v, w = nu[:3], nu[3:]
    X = -m * S(v) - m * S(S(w) @ rg)
    return np.block([[np.zeros((3, 3)), X], [X, sign * m * S(S(v) @ rg) - S(Io @ w)]])


def C_A(nu, sign=1.0):                                    # (3.11), entry by entry from the printed matrix
    Xu, Yv, Zw, Kp, Mq, Nr = -np.diag(A)
    u, v, w, p, q, r = nu
    return np.array([
        [0, 0, 0, 0, -Zw * w, Yv * v],
        [0, 0, 0, Zw * w, 0, -Xu * u],
        [0, 0, 0, -Yv * v, Xu * u, 0],
        [0, -Zw * w, Yv * v, 0, -Nr * r, sign * Mq * q],
        [Zw * w, 0, -Xu * u, Nr * r, 0, -Kp * p],
        [-Yv * v, Xu * u, 0, -sign * Mq * q, Kp * p, 0]])


def T_rigid(nu):
    v, w = nu[:3], nu[3:]
    vg = v + np.cross(w, rg)
    return 0.5 * m * vg @ vg + 0.5 * w @ Ig @ w


def hessian(f, n=6, h=1e-3):
    H = np.zeros((n, n)); E = np.eye(n) * h
    for i in range(n):
        for j in range(n):
            H[i, j] = (f(E[i] + E[j]) - f(E[i] - E[j]) - f(-E[i] + E[j]) + f(-E[i] - E[j])) / (4 * h * h)
    return H


def kirchhoff(M, nu, nud):
    v, w = nu[:3], nu[3:]
    mom, momd = M @ nu, M @ nud
    P, L = mom[:3], mom[3:]
    return np.concatenate([momd[:3] + np.cross(w, P), momd[3:] + np.cross(w, L) + np.cross(v, P)])


results = []
def record(name, err, tol, control=False):
    ok = (err > tol) if control else (err < tol)
    results.append(ok)
    print(f"{'PASS' if ok else 'FAIL'} {'control' if control else 'check  '} {name}: err {err:.3e} tol {tol:.0e}")

e = np.max(np.abs(hessian(T_rigid) - M_RB()))
record("M_RB (3.8) is the Hessian of the rigid-body kinetic energy", e, 1e-7)
bad = M_RB(); bad[:3, 3:] *= -1; bad[3:, :3] *= -1
record("same check, CG coupling blocks sign-swapped", np.max(np.abs(hessian(T_rigid) - bad)), 1e-7, control=True)

eRB = eA = eTot = cRB = cA = 0.0
for _ in range(200):
    nu, nud = rng.normal(size=6), rng.normal(size=6)
    eRB = max(eRB, np.max(np.abs(M_RB() @ nud + C_RB(nu) @ nu - kirchhoff(M_RB(), nu, nud))))
    eA = max(eA, np.max(np.abs(A @ nud + C_A(nu) @ nu - kirchhoff(A, nu, nud))))
    eTot = max(eTot, np.max(np.abs((M_RB() + A) @ nud + (C_RB(nu) + C_A(nu)) @ nu - kirchhoff(M_RB() + A, nu, nud))))
    cRB = max(cRB, np.max(np.abs(M_RB() @ nud + C_RB(nu, -1.0) @ nu - kirchhoff(M_RB(), nu, nud))))
    cA = max(cA, np.max(np.abs(A @ nud + C_A(nu, -1.0) @ nu - kirchhoff(A, nu, nud))))
record("M_RB nu_dot + C_RB(nu) nu equals Kirchhoff (3.9)", eRB, 1e-11)
record("M_A nu_dot + C_A(nu) nu equals Kirchhoff (3.11)", eA, 1e-11)
record("total M and C equal Kirchhoff", eTot, 1e-11)
record("C_RB with the m S(S(v) rg) sign flipped", cRB, 1e-11, control=True)
record("C_A with the M_qdot entries sign-flipped", cA, 1e-11, control=True)
print(f"{sum(results)} of {len(results)} as intended")
sys.exit(0 if all(results) else 1)
