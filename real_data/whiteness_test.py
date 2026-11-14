"""Innovation autocorrelation (whiteness) test for the ESKF on a recorded run.

    [REAL_DATA_RUN=<run>] python whiteness_test.py [variant ...]

A correctly modelled Kalman filter has white innovations (Bar-Shalom, Li & Kirubarajan 2001, ch. 5).
The ESKF saves normalized innovations nu = L^-1 r (S = L L'). For each sensor this script computes
- the time-averaged normalized autocorrelation  rho(l) = sum nu_k' nu_{k+l} / sqrt(sum |nu_k|^2 sum |nu_{k+l}|^2)
  over pairs l samples apart inside one continuous stretch (stretches end at gaps > 3x the median spacing),
  with the 95 % whiteness band +-1.96 / sqrt(K) (K = number of pairs);
- the per-component autocorrelation (same formula, one component);
- the mean test: |mean(nu)| against 1.96 / sqrt(N).
Writes <run results>/whiteness_table.txt and whiteness_acf.png.
"""
import sys

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from scipy.io import loadmat

from paths import DATA, RESULTS, RUN

VARIANTS = sys.argv[1:] or ['eskf_ungated_zv', 'eskf_accel30_zv', 'eskf_hdg0p2hz_zv']
SENSORS = ['dvl', 'depth', 'heading']
MAX_LAG_S = 10.0


def stretches(t):
    d = np.diff(t)
    cut = np.where(d > 3 * np.median(d))[0] + 1
    return np.split(np.arange(len(t)), cut)


def acf(nu, t, max_lag):
    """rho(l), band(l), and per-component rho for l = 1..max_lag (pairs within one stretch only)."""
    segs = [s for s in stretches(t) if len(s) > max_lag + 5]
    rho, band, comp = [], [], []
    for lag in range(1, max_lag + 1):
        a = np.concatenate([nu[s[:-lag]] for s in segs])
        b = np.concatenate([nu[s[lag:]] for s in segs])
        num = np.sum(a * b)
        rho.append(num / np.sqrt(np.sum(a * a) * np.sum(b * b)))
        band.append(1.96 / np.sqrt(len(a)))
        comp.append(np.sum(a * b, axis=0) / np.sqrt(np.sum(a * a, axis=0) * np.sum(b * b, axis=0)))
    return np.array(rho), np.array(band), np.array(comp)


lines = [f'Run: {RUN or "2026-11-05 (uncorrected DVL mounting)"}. Innovation whiteness (autocorrelation) test.',
         'rho(l): time-averaged normalized autocorrelation of the normalized innovations at lag l;',
         'white innovations stay inside +-1.96/sqrt(K) at about 95 % of lags. "decorrelates" = first lag inside the band.', '']
curves = {}
for v in VARIANTS:
    f = DATA / f'{v}.mat'
    if not f.exists():
        continue
    r = loadmat(f, squeeze_me=True, struct_as_record=False)
    if not hasattr(r['innov'], 'dvl'):
        lines.append(f'{v}: no innovations saved (re-run run_real_log)')
        continue
    lines.append(f'== {v}')
    for s in SENSORS:
        nu = np.atleast_2d(np.asarray(getattr(r['innov'], s), float))
        if nu.size == 0:
            continue
        if nu.shape[0] == 1 and nu.shape[1] > 3:
            nu = nu.T
        t = np.asarray(getattr(r['innov_t'], s), float).ravel()
        ok = np.all(np.isfinite(nu), axis=1) & np.isfinite(t)
        nu, t = nu[ok], t[ok]
        if len(t) < 50:
            continue
        dt = float(np.median(np.diff(t)))
        L = int(round(MAX_LAG_S / dt))
        rho, band, comp = acf(nu, t, L)
        out = np.abs(rho) > band
        first_in = int(np.argmax(~out)) + 1 if (~out).any() else None
        mean = nu.mean(axis=0)
        mean_ok = np.abs(mean) < 1.96 / np.sqrt(len(nu))
        pick = [l for l in (1, 2, 5, 10, 20, 50) if l <= L]
        lines.append(
            f'  {s:8s} N={len(nu):6d} spacing {dt * 1000:5.0f} ms | rho at lag ' +
            ', '.join(f'{l} ({l * dt:.2f} s): {rho[l - 1]:+.3f}' for l in pick) +
            f' | band ±{band[0]:.3f} | outside band {100 * out.mean():4.0f} % of lags 1..{L}'
            f' | decorrelates at {"never within %.0f s" % MAX_LAG_S if first_in is None else f"lag {first_in} ({first_in * dt:.2f} s)"}'
            f' | mean {np.round(mean, 3).tolist()} {"ok" if mean_ok.all() else "BIASED"}')
        if nu.shape[1] > 1:
            lines.append(f'  {"":8s} per component at lag 1: ' + ', '.join(f'{c:+.3f}' for c in comp[0]))
        curves[(v, s)] = (np.arange(1, L + 1) * dt, rho, band)
text = '\n'.join(lines) + '\n'
print(text)
RESULTS.mkdir(parents=True, exist_ok=True)
(RESULTS / 'whiteness_table.txt').write_text(text)

main = next((v for v in VARIANTS if any(k[0] == v for k in curves)), None)
if main:
    shown = [s for s in SENSORS if any(k[1] == s for k in curves)]
    fig, axs = plt.subplots(1, len(shown), figsize=(5 * len(shown), 4.2), sharey=True)
    axs = np.atleast_1d(axs)
    for ax, s in zip(axs, shown):
        for v, c in zip(VARIANTS, ('tab:blue', 'tab:orange', 'tab:green')):
            if (v, s) in curves:
                lag_s, rho, band = curves[(v, s)]
                ax.plot(lag_s, rho, color=c, lw=1.5, label=v)
                ax.plot(lag_s, band, color='k', lw=0.8, ls='--')
                ax.plot(lag_s, -band, color='k', lw=0.8, ls='--')
        ax.axhline(0, color='0.7', lw=0.8)
        ax.set_title(f'{s} innovations', fontsize=11)
        ax.set_xlabel('lag (s)')
    axs[0].set_ylabel('normalized autocorrelation')
    axs[0].legend(fontsize=8)
    fig.suptitle(f'Innovation whiteness test, {RUN or "2026-11-05 uncorrected"} (dashed: 95 % band for white noise)', fontsize=12)
    plt.tight_layout()
    plt.savefig(RESULTS / 'whiteness_acf.png', dpi=120)
    print('saved', RESULTS / 'whiteness_acf.png')
