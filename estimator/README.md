# Estimator study (Phase 2) — FROZEN PYTHON REFERENCE

> **Frozen.** The main version is now MATLAB, in `../estimator_m/`. Don't develop this
> code further. It is kept only as an independent reference.
> `export_reference.py` writes `estimator_m/tests/reference_python.mat`, and the MATLAB test
> `test_cross_check_python` must reproduce it.

Error-state Kalman filter for the manta AUV, tested on a designed truth trajectory
with simulated sensors. No plant model yet (Phase 4). Derivation: `log/2026-10-04_estimator_log.docx`, section 12.

**All parameters are ASSUMED** and live in one file, `config.py`. Each line says where
the number came from. Replace them with the VN-200 datasheet for your variant (plus an
Allan-variance test), the Water Linked A50 datasheet, the pressure-sensor datasheet and
your measured lever arms before reading anything into absolute numbers.

## Files

| File | Content |
|---|---|
| `config.py` | Every sensor, Earth and scenario parameter (ASSUMED) |
| `rotations.py` | Hamilton quaternions (scalar first), Fossen zyx Euler angles, VN-200 reorder |
| `truth.py` | Analytic trajectory, exact IMU increments, simulated DVL, depth, GNSS, heading |
| `eskf.py` | 16-state ESKF: Earth rate + Coriolis, lever arms, depth bias, RW or GM IMU biases |
| `sim.py` | One Monte Carlo run: errors, covariance, NEES, NIS |
| `run_compare_bias.py` | Random-walk vs Gauss–Markov comparison (2 × 2, N runs per cell) |
| `tests/` | Jacobians, conventions, mechanisation, consistency |

## Sensors and scenario

- VN-200 IMU at 100 Hz (delta-angle and delta-velocity), A50 DVL 3-D velocity at 10 Hz,
  analog depth sensor (30 m, 10-bit ADC) at 20 Hz, VN-200 GNSS at 5 Hz when the antenna is
  above water, magnetometer heading at 10 Hz.
- 600 s: surface with GNSS (0–60 s), dive to 10 m, four 180° turns, **DVL dropout
  300–360 s**, ascent (500–550 s), surface with GNSS (550–600 s).

## How to run

```bash
pip install numpy scipy pytest
python -m pytest estimator/tests -s          # ~40 s on 4 cores
python -m estimator.run_compare_bias 20      # ~5 min on 4 cores
```

## Results (as run on 2026-10-04)

**Tests: 11 passed, 1 expected failure (explained below).**

- Measurement Jacobians (DVL, depth, GNSS position and velocity, heading), including lever
  arms and Earth-rate terms, match finite differences.
- The transition matrix matches finite differences up to the neglected ½·dt² position
  term, and the mismatch drops 4× when dt is halved, as a first-order discretisation should.
- With noise-free sensors the mechanisation stays within 4.5e-5 m and 5e-15 rad of the
  truth after 600 s.
- Consistency, 8 runs with matched models: ANEES 16.73 (dof 16), inside the 95% band at
  96.7% of time points. Mean NIS: DVL 3.027, GNSS position 3.021, GNSS velocity 2.990,
  heading 0.997, all inside their bands.
- **Expected failure: depth NIS = 0.916, outside its band [0.988, 1.012].**
  - The 10-bit ADC gives 2.9 cm steps. While depth is held, the rounding error is almost
    constant, so it is not white noise. The depth NIS samples are correlated, and the
    white-noise χ² band does not apply.
  - Per-run depth NIS swings widely in both directions: 0.84–1.12 over 4 runs here, and
    1.044 in the MATLAB Monte Carlo with other seeds.
  - With a 16-bit ADC the same check gives 1.000 with little spread, which confirms the
    cause.
  - Correction: an earlier version of this README said the filter is "conservative in
    depth". The MATLAB run showed that was overclaimed.

**Bias-model comparison** (`results/compare_bias.txt`, N = 20 per cell, paired seeds):

| Truth | Filter | ANEES (band 13.62–18.57) | Time inside band | Horiz. RMS at 360 s (end of dropout) | Horiz. RMS at 500 s |
|---|---|---|---|---|---|
| RW | RW | 15.87 | 98.3% | 1.49 m | 1.13 m |
| RW | GM | 21.26 | **25.7%** | 1.51 m | 1.15 m |
| GM | RW | 15.05 | 82.6% | 1.49 m | 1.11 m |
| GM | GM | 16.04 | 98.6% | 1.47 m | 1.11 m |

- With these assumed values, the bias model hardly changes accuracy over 600 s.
- It does change consistency. A Gauss–Markov filter on a bias that really wanders is
  overconfident (ANEES 21.3). A random-walk filter on a Gauss–Markov bias is slightly
  conservative.
- Random walk is the safer default until an Allan-variance test on your VN-200 tells us
  which model fits.

## Results that look too good (read before trusting the numbers)

- **Heading:** heading noise is simulated as white 2°. Real magnetometer errors near
  thrusters are biased and slowly varying, so the heading and horizontal-position
  results are optimistic.
- **DVL:** the simulated DVL has white noise only, with no scale factor or misalignment
  error. Real DVL scale error gives drift proportional to distance.
- **Short dropout:** the dropout is 60 s. Longer dropouts need the thrust-model aid (Phase 4).
- **Comparison with the VN-200's internal INS:** not done. It needs recorded VN-200 data,
  because the internal filter cannot be simulated here.
