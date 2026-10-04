"""Compare random-walk vs Gauss-Markov bias models (log 12.6, user decision: do both).

Truth bias model x filter bias model, N Monte Carlo runs each. The two truth
models share the same driving noise density, so they differ only in decay.

Usage: python -m estimator.run_compare_bias [N]
"""
import sys
from multiprocessing import Pool

import numpy as np
from scipy.stats import chi2

from estimator.sim import get_truth, run_once

DOF = 16


def _job(args):
    seed, tb, fb = args
    r = run_once(seed, tb, fb)
    return tb, fb, r


def summarise(runs, t):
    nees = np.array([r["nees"] for r in runs])          # (N, T)
    err = np.array([r["err"] for r in runs])            # (N, T, 16)
    n = len(runs)
    lo, hi = chi2.ppf([0.025, 0.975], n * DOF) / n
    anees = nees.mean(0)
    inside = np.mean((anees >= lo) & (anees <= hi))
    horiz = np.linalg.norm(err[:, :, 0:2], axis=2)
    out = dict(anees_mean=anees.mean(), anees_inside=inside, lo=lo, hi=hi)
    for tt in (300.0, 360.0, 500.0, 599.0):
        i = np.searchsorted(t, tt)
        out[f"hrms_{int(tt)}"] = np.sqrt(np.mean(horiz[:, i] ** 2))
    i = -1
    out["bg_rms_end_deg_h"] = np.degrees(np.sqrt(np.mean(err[:, i, 12:15] ** 2))) * 3600
    out["ba_rms_end_mg"] = np.sqrt(np.mean(err[:, i, 9:12] ** 2)) / 9.80665 * 1e3
    nis = {}
    for name in runs[0]["nis"]:
        allv = np.concatenate([r["nis"][name] for r in runs])
        if allv.size:
            nis[name] = allv.mean()
    out["nis"] = nis
    return out


def main(N=20, seed0=1000):
    get_truth()
    combos = [(tb, fb) for tb in ("rw", "gm") for fb in ("rw", "gm")]
    jobs = [(seed0 + i, tb, fb) for tb, fb in combos for i in range(N)]
    with Pool() as pool:
        res = pool.map(_job, jobs)
    lines = [f"Bias-model comparison, N = {N} runs per cell, 600 s scenario, all parameters ASSUMED",
             "Same seeds across filter models within a truth model (paired comparison).", ""]
    hdr = (f"{'truth':>5} {'filter':>6} | {'ANEES':>6} {'in95%':>6} | "
           f"{'h@300':>6} {'h@360':>6} {'h@500':>6} {'h@599':>6} [m RMS] | "
           f"{'bg_end':>7} [deg/h] {'ba_end':>7} [mg]")
    lines.append(hdr)
    t = None
    for tb, fb in combos:
        runs = [r for (a, b, r) in res if a == tb and b == fb]
        t = runs[0]["t"]
        s = summarise(runs, t)
        lines.append(f"{tb:>5} {fb:>6} | {s['anees_mean']:6.2f} {100*s['anees_inside']:5.1f}% | "
                     f"{s['hrms_300']:6.2f} {s['hrms_360']:6.2f} {s['hrms_500']:6.2f} {s['hrms_599']:6.2f}        | "
                     f"{s['bg_rms_end_deg_h']:7.2f}         {s['ba_rms_end_mg']:7.4f}")
        lines.append("      mean NIS: " + ", ".join(f"{k} {v:.2f}" for k, v in s["nis"].items()))
    lines.append("")
    lines.append(f"ANEES 95% bounds for N={N}, dof 16: [{s['lo']:.2f}, {s['hi']:.2f}]. "
                 "Expected mean NIS = measurement dimension (dvl 3, depth 1, gnss 3, heading 1).")
    lines.append("Timeline: surface 0-60 s (GNSS), dive 60-110 s, DVL dropout 300-360 s, "
                 "ascent 500-550 s, surface 550-600 s.")
    text = "\n".join(lines)
    print(text)
    return text


if __name__ == "__main__":
    N = int(sys.argv[1]) if len(sys.argv) > 1 else 20
    text = main(N)
    import os
    os.makedirs("estimator/results", exist_ok=True)
    with open("estimator/results/compare_bias.txt", "w") as fh:
        fh.write(text + "\n")
