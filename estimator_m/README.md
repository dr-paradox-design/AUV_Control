# Estimator (MATLAB) — main version

Error-state Kalman filter for the manta AUV, tested on a designed truth trajectory with
simulated sensors. This is the **main version**. The Python code in `../estimator/` is a
**frozen reference**: don't develop it further; it exists only for the cross-check test.

Derivation: `log/2026-10-04_estimator_log.docx`, section 12 (corrections in 12.10).
Plain MATLAB, no toolbox needed. It also runs in GNU Octave 8, which is how it was tested here.

**All parameters are ASSUMED** and live in one file, `config_estimator.m` (same values as
the Python reference). Replace them with the VN-200 datasheet for your variant plus an
Allan-variance test, the A50 datasheet, the pressure-sensor datasheet and your measured
lever arms.

## Files

| File | Content |
|---|---|
| `config_estimator.m` | Every sensor, Earth and scenario parameter (ASSUMED) |
| `lib/` | Quaternion and rotation helpers (Hamilton, scalar first; Fossen zyx), `vn2hamilton`, `chi2inv_core` |
| `truth_trajectory.m` | Analytic trajectory and exact IMU increments |
| `sim_sensors.m` | IMU biases (RW/GM) and noise, DVL, depth (10-bit), GNSS, heading |
| `eskf_nominal.m`, `eskf_F.m`, `eskf_Q.m` | Prediction: mechanisation with Earth rate and Coriolis, transition, process noise |
| `eskf_meas.m`, `eskf_update.m` | Measurement models with lever arms; Joseph update, injection |
| `state_oplus.m`, `state_ominus.m`, `make_state.m`, `initial_covariance.m` | Error-state algebra |
| `run_filter.m` | Runs the filter over a data set (simulated now, recorded later) |
| `run_once.m`, `run_compare_bias.m` | One Monte Carlo run; RW vs GM comparison |
| `tests/` | `run_all_tests.m` and the tests; `reference_python.mat` for the cross-check |

## How to run (MATLAB)

```matlab
cd estimator_m
setup_paths
run_all_tests              % fast tests
run_all_tests(true)        % plus the Monte Carlo consistency test
run_compare_bias(20)       % RW vs GM comparison, writes results/compare_bias.txt
r = run_once(1, 'rw', 'rw');            % one run: r.t, r.err, r.sd, r.nees, r.nis
plot(r.t, vecnorm(r.err(:,1:2), 2, 2))  % horizontal position error
```

With the Parallel Computing Toolbox, `run_compare_bias` and `test_consistency` use `parfor`.
In Octave they run serially, at about 42 s per 600 s run.

## Test results (GNU Octave 8.4, 2026-10-04)

**Fast tests: 3 of 3 passed.**

- **Jacobians:** all five measurement Jacobians match finite differences to ≤ 4e-10. `F` matches up to the neglected
  ½·dt² position term (3.99e-4 at dt = 0.01 s, ratio 4.00 when dt halves).
- **Conventions and truth:** rotation conventions pass. The truth depth integral matches the analytic depth to 3e-14 m. Body
  rates match the attitude derivative (1.7e-7, central difference). Noise-free
  mechanisation over 600 s: 4.5e-5 m, 5e-15 rad.
- **Cross-check with the frozen Python reference** (same inputs, 120 s):
  - truth matches to 3.6e-15;
  - filter states match to 1.4e-14;
  - final covariance matches to a relative 6.7e-16.

  So the MATLAB port reproduces the Python results to floating-point rounding.

**Monte Carlo consistency (`run_all_tests(true)`, 8 runs, Octave, 363 s):**

- ANEES 16.32 (dof 16), inside the 95% band at 98.2% of points.
- Mean NIS:
  - DVL 3.009, GNSS position 2.997, GNSS velocity 3.017, heading 0.999 — all inside their
    bands;
  - depth 1.044, outside its band [0.988, 1.012]. This is the expected failure, explained
    below.

**Bias comparison:** `results/compare_bias.txt` is a 2-runs-per-cell smoke test (Octave,
373 s). It confirms the script works, but it is too few runs to quote. It shows the same
pattern as the Python 20-run result: a GM filter on a random-walk bias has the highest
ANEES (20.36). Run `run_compare_bias(20)` in MATLAB for the real numbers.

## Things to know

- **Random numbers:** MATLAB and Python use different random generators, so individual Monte Carlo
  runs differ between the two. Statistics should agree. The deterministic cross-check
  above is the exact comparison.
- **Optimistic assumptions:** white 2° heading noise, no DVL scale error, a 60 s dropout
  only. See `../estimator/README.md`.
- **Known modelling mismatch:** the 10-bit depth quantisation error is time-correlated. Depth
  NIS samples are therefore correlated, the white-noise band does not apply, and per-run
  depth NIS swings in both directions: 1.044 here, 0.84–1.12 in Python runs. With a 16-bit
  ADC it is 1.000. Fix options: a higher-resolution ADC, or model the quantisation error
  as a correlated state.
