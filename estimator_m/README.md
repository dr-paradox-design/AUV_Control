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
| `config_real_log.m`, `load_real_log.m`, `run_real_log.m` | Recorded-data path: log-supported config, loader, variants (see "Running on recorded data") |
| `run_outage_benchmark.m` | Artificial DVL outages on the recorded log (with `../real_data/outage_benchmark.py`) |
| `unobservable_directions.m` | Yaw and horizontal-translation directions for the optional observability constraint |
| `run_segments.m` | Runs the filter per gap-free segment when a recording has gaps (restart, biases carried over) |

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

## Running on recorded data (GNU Octave 6.4, 2026-11-08)

First run of this ESKF on a real log (2026-11-05, 1250 s, vehicle unarmed and moved by hand; see
`../real_data/README.md`). **There is no ground truth**, so the results are consistency checks and
agreement with two references, not accuracy.

**What changed in the filter code.** All changes are additions; with default arguments the behaviour is
identical, and the Python cross-check still matches to 1.4e-14.
- `run_filter`:
  - a truth-free mode (no `tr.p`: `err` and `nees` are NaN);
  - an optional `opts` argument with per-sensor χ² gates and a cap on consecutive rejections;
  - a sixth measurement type, `zero_vel`.
- `eskf_update`: an optional NIS gate; a rejected update leaves `x` and `P` unchanged.
- `eskf_meas`: `zero_vel`, a "velocity ≈ 0" pseudo-measurement. It is an assumption about slow or unpowered
  motion, not a sensor.
- `eskf_update`: an optional robust mode (`opts.robust.<sensor>`). When a measurement's NIS exceeds the
  cap, its noise is inflated just enough to bring the NIS down to the cap. The measurement is used, never
  dropped, so the gating spiral cannot start.
- `tests/test_gating.m` (new) and a `zero_vel` row in `test_jacobians`. `test_gating` also
  checks the robust mode. **Fast tests: 4 of 4 pass on Octave 6.4.**
- `../compat/octave6/rng.m`: Octave < 7 has no `rng`. `setup_paths` adds this stand-in only when `rng` is missing.

**How to run**

```bash
python real_data/extract.py <bag folder>        # decode the bag
python real_data/export_mat.py                  # FLU/ENU -> FRD/NED, IMU increments at 80 Hz
cd estimator_m && octave-cli --eval "setup_paths; run_real_log"   # ~30 s per variant
cd ../real_data && python ekf_localization.py && python compare_eskf.py
```

**Inputs from the log** (`config_real_log.m`; every line is tagged with its source):
- **Lever arms** from `/tf_static`.
- **Depth noise** measured from the log: 0.080 m.
- **Heading:** the VN-200's own yaw at 10 Hz. No magnetometer is logged.
- **Attitude** is integrated from the raw gyro and corrected through gravity. The frame conversion was
  checked first: 5 s of gyro-only integration matches the VN-200 attitude to a median 0.32°, and to 8.8°
  with the gyro sign flipped.

**Tuning was done on NIS only, never on agreement with the references.**

| Accelerometer noise | DVL median NIS (target 2.37) | DVL rejected at 99 % gate |
|---|---|---|
| datasheet ×1 | 4.93 | 40.3 % |
| ×10 | 3.18 | 31.1 % |
| ×15 | 2.63 | 22.2 % |
| **×20 (used)** | **2.12** | 16.5 % |
| ×30 | 1.43 | 7.9 % |

- **The datasheet value is a floor.** It describes the sensor, not a hand-moved vehicle.
- **Heading σ:** 1.3°, which gives a median NIS of 0.37 (target 0.45).
- **DVL time shift:** 0.1–0.2 s changed the rejection rate by under one point, so timestamps are used as
  recorded.

**Results** (`../real_data/results/eskf_table.txt`; references start at the origin; RMS over 0–797 s):

| Estimator | Path (m) | RMS vs robot_localization | RMS vs DVL dead reckoning | Drift in 49.5 s DVL dropout | Attitude vs VN-200 (roll / pitch / yaw RMS) |
|---|---|---|---|---|---|
| ESKF, datasheet noise | 313.7 | 7.74 m | 13.25 m | 1.3 m | 0.50° / 0.27° / 1.25° |
| **ESKF, tuned, no gate, zero-vel** | **87.1** | **0.39 m** | **0.35 m** | 0.17 m | **0.34° / 0.27° / 1.18°** |
| ESKF, tuned, 99 % gate, no zero-vel | 434.5 | 27.09 m | 24.01 m | 117.3 m | 0.49° / 0.29° / 1.29° |
| ESKF, tuned, 99 % gate, zero-vel | 137.7 | 2.11 m | 2.71 m | 0.14 m | 0.45° / 0.28° / 1.28° |
| ESKF, tuned, 99.99 % gate (cap 3), zero-vel | 88.9 | 0.90 m | 0.88 m | 0.16 m | 0.37° / 0.28° / 1.19° |
| ESKF, tuned, robust (NIS capped at 99 %), zero-vel | 94.2 | 0.95 m | 0.86 m | 0.15 m | 0.38° / 0.28° / 1.20° |
| Python EKF, variant E (`../real_data`) | 85.6 | 0.53 m | 0.62 m | 0.08 m | — |
| robot_localization | 87.0 | — | — | 0.17 m | — |
| DVL dead reckoning | 81.8 | — | — | (gap skipped) | — |

**Findings**
- **Attitude from the raw gyro works.** With only the VN-200 yaw as an aid, roll and pitch agree with the
  VN-200's internal solution to about 0.3° RMS.
- **The estimated biases are physically sensible.** The vertical accelerometer bias settles at −0.087 m/s²,
  close to the gap between the gravity the accelerometer reads (median 9.90 m/s²) and this config's model
  gravity (9.79 m/s² at the assumed 20° latitude).
- **A 99 % gate makes this filter diverge.**
  - Ungated, 7.4 % of DVL innovations exceed the 99 % threshold (1 % expected) and 2.7 % exceed 99.99 %. So
    some DVL readings really are outliers.
  - With the gate on, the filter rejects 15 % of DVL samples even when level (tilt ≤ 10°). It then drifts,
    which makes the next samples look worse, and it spirals.
  - Rejections are highest when tilted > 10° (50 %) and within 2 s after a DVL gap (52 %), but most come
    from the spiral.
- **The zero-velocity aid is what bridges the 49.5 s dropout.** Without it, the drift is about 117 m.

**Read before trusting these numbers**
- **The references use every DVL sample.** Dead reckoning integrates all of them, and robot_localization
  shows no sign of gating (its config is not in the log). So the ungated ESKF agrees with them best almost by
  construction. Whether gating the extreme 2.7 % helps *accuracy* cannot be decided without ground truth.
- **The tuned noise is specific to this log.** It is the effective noise of a hand-moved vehicle. A powered
  run needs re-tuning, and the zero-velocity aid is invalid there.
- **The heading comes from the VN-200.** It is the VN-200's yaw, so yaw is not independent of it. A logged
  magnetometer is needed for that.
- **Unchanged assumptions:** latitude (20°), gyro noise and bias values are still ASSUMED.

**Robust update instead of gating.**
- **What it does:** no sample is dropped. 873 DVL samples (9.8 %) and 2,344 depth samples are down-weighted,
  and the median NIS stays consistent (DVL 2.06).
- **Against the references** it scores 0.95 m and 0.86 m RMS, about the same as the loose gate. That is
  worse than ungated, which the "references use every DVL sample" caveat predicts.
- **Which is better for accuracy is still open.** It needs ground truth.

**How long can the IMU bridge a DVL outage?** (`../real_data/outage_benchmark.py`, `run_outage_benchmark(L)`)
DVL samples are removed in 13–14 windows of L seconds, one every 60 s, wherever the real DVL stream is
continuous. The error is the horizontal difference between each filter's displacement over the window and
DVL dead reckoning over the same window, which uses the removed data.

| Outage | Vehicle motion (median) | Pure inertial ESKF | ESKF + "velocity ≈ 0" | ESKF robust + "velocity ≈ 0" | Python E | ESKF with DVL (floor) |
|---|---|---|---|---|---|---|
| 2 s | 0.05 m | 0.08 m | 0.05 m | 0.05 m | 0.05 m | 0.004 m |
| 5 s | 0.12 m | 0.48 m | 0.14 m | 0.14 m | 0.13 m | 0.005 m |
| 10 s | 0.15 m | 1.86 m | 0.14 m | 0.14 m | 0.15 m | 0.008 m |
| 20 s | 0.33 m | 8.44 m | 0.33 m | 0.33 m | 0.31 m | 0.018 m |

(Median errors. The p90 and max are in `../real_data/results/outage_table_<L>s.txt`.)

- **Pure inertial error grows about as t².** It is roughly ½ · 0.04 m/s² · t², which corresponds to a
  residual acceleration error such as a 0.25° tilt error or accelerometer bias.
- **On this slow, unpowered log, "velocity ≈ 0" wins at every length.** But its error is about 75–95 % of the
  real motion, so it is "assume the vehicle stopped", not navigation.
- **At 0.5 m/s, "assume stopped" is wrong by 0.5 m per second.** By extrapolation the pure inertial solution
  would be better for outages up to roughly 20 s. That is not measured: there is no powered log.
- **The DVL carries the position estimate.** Fewer DVL dropouts (15 gaps > 1 s here, longest 49.5 s) is
  worth more than any filter change. A thrust-model aid (`plant_m/`) is the route for longer outages.

## Improvements tested from the literature (2026-11-09)

Papers in `../input/bibliography/` (PDFs in the local research library):
- **Klein & Diamant 2015:** what a DVL + depth + IMU system can observe.
- **Huang et al. 2010:** EKF overconfidence in unobservable directions.
- **Or 2025:** how gating biases NIS.

Scripts: `run_real_log` variants `eskf_noheading*`, `eskf_oc*`, `eskf_hdg*`, plus `../real_data/heading_check.py`
(`results/heading_table.txt`). The VN-200's own magnetometer-aided yaw is used as the yaw reference. It is
**not** ground truth (see 2).

**1. Observability (Klein & Diamant): confirmed on real data.**
- **Without the VN-200 yaw aid, yaw is not observable** from DVL, depth and gravity:
  - The vertical gyro bias estimate stays at its start value (−0.001 °/s, against −0.023 °/s with the aid).
  - Gyro-only yaw ends 21° from the VN-200 yaw after 21 min.
- **Position effect is limited.** Against the references, RMS goes from 0.39 to 0.65 m (DVL dead reckoning:
  0.35 → 0.68 m).

**2. The VN-200 yaw itself is the weak link.**
- Most of the 21° disagreement builds up while the vehicle is level (−13.3° of −21°), including single 10 s
  windows with 5° jumps and only about 30° of rotation.
- A gyro cannot be wrong by that much in 10 s: it would need a 0.5 °/s bias or a 17 % scale error. So the
  magnetometer-aided VN-200 heading most likely jumps in the pool environment.

**3. Observability constraint (after Huang et al.): no measurable effect here.**
- **What it does** (`opts.oc`): projects the DVL, depth and zero-velocity Jacobians so that H N = 0 along yaw
  and north/east translation. `test_gating` checks it: with an isotropic prior, an update no longer changes
  the variance along N.
- **What it changed on the log:** nothing beyond noise. Yaw, NIS and position are unchanged (RMS 0.39 /
  0.38 m).
- **Why:** worked out by hand, our DVL and depth Jacobians already satisfy H N ≈ 0, apart from an Earth-rate
  term. Only the zero-velocity pseudo-measurement violates it.
- **Not done:** the propagation-side constraint (first-estimates Jacobians or a modified Φ) is not
  implemented.
- **Retracted claim:** the yaw σ used here is √(nᵀ P n) with n = Rᵀ e_z. It changes whenever the vehicle
  tilts, so counting "σ decreases" is **not** evidence of spurious information.

**4. Heading-aid rate (correlated measurement): the change that helps.**
- **The problem:** the VN-200 yaw is its own filter's output, so its errors are correlated over time. Fed in
  at 10 Hz as if white, it makes the filter claim 0.07° of yaw uncertainty while sitting about 1° from that
  same yaw.

| Heading aid | Yaw σ at end | Yaw vs VN-200 RMS | Mean yaw NEES | RMS vs robot_localization | RMS vs DVL DR |
|---|---|---|---|---|---|
| 10 Hz, σ 1.3° (current) | 0.07° | 1.18° | 325 | 0.39 m | 0.35 m |
| 1 Hz, σ 1.3° (`eskf_hdg1hz_zv`) | 0.15° | 1.58° | 110 | 0.34 m | 0.37 m |
| **0.2 Hz, σ 3° (`eskf_hdg0p2hz_zv`)** | 0.49° | 2.70° | 34 | **0.33 m** | **0.33 m** |
| none (`eskf_noheading_zv`) | 1.16° | 17.9° | 359 | 0.65 m | 0.68 m |

- **Using the VN-200 yaw less often helps both ways:** overconfidence falls tenfold, and agreement with both
  references improves.
- **It is still not consistent** (NEES 34, against 1 for a consistent filter). A correlated heading error
  cannot be tuned into white noise.
- **The proper fixes:** a Gauss–Markov heading-error state, or a better heading source (a calibrated raw
  magnetometer, or visual heading from pool features).
- **Caveat:** both references also use the VN-200 orientation, so neither can fully judge yaw.
- **A robust heading update** (NIS cap at 99 %, `eskf_oc_robusthdg_zv`) down-weights 179 samples and changes
  nothing measurable.

**5. Gated NIS (Or 2025): judge gated filters against gate-conditioned targets.**

| Gate | Accepted-only target mean / median | 99 % gated ESKF, accepted only |
|---|---|---|
| DVL, 99 % (χ² 3 dof ≤ 11.34) | 2.894 / 2.339 | mean 3.22, median **1.76** |
| DVL, 99.99 % | 2.998 / 2.366 | (loose gate) mean 3.61, median 1.97 |
| depth / heading, 99 % (1 dof) | 0.925 / 0.444 | — |

- **The accepted-only mean looks consistent; the median shows it is not.** The middle of the distribution is
  too cautious and the tail is heavy, while 16 % of readings were thrown away.
- **Our tuning is unaffected:** it used the median NIS of *all* readings, the unconditional statistic.

**Recommendation:**
- Use the VN-200 yaw at 0.2 Hz with σ 3° until a better heading source exists. Not yet made the default in
  `config_real_log.m`: it needs confirming on a log with ground truth.
- Log the raw magnetometer, so heading can be calibrated and modelled properly.

## Second log, 2026-11-11 tape test (see `../real_data/README.md`)

- **The DVL is rotated −135° in this run.** The export rotates it into body axes, so `cfg.dvl.R_bd` stays the
  identity.
- **The recording has two gaps, handled by `run_segments.m`.** Through them the old export interpolated IMU
  rates, which drove DVL NIS to 15.6 and zero-velocity NIS to 389.
- **The NIS-tuned accelerometer noise is ×30 here, against ×20 on the first log:** `eskf_accel30_zv`.
- **All DVL-based estimates agree within 2–4 cm.** The 99 % gate diverges again (0.19 m); datasheet noise gives
  0.60 m.

## Correction (2026-11-11): DVL mounting on the 2026-11-05 log

**The finding:** the 2026-11-05 `/tf_static` gave the DVL 0° of rotation, but it was mounted at −135°. Confirmed
by `../real_data/dvl_mount_fit.py`; corrected run `sensors_20261105_130603`. All results above for that log use
the wrong rotation.

**With the correct mounting:**
- **The DVL innovation tail halves** (3.9 % beyond 99 %, against 7.4 %).
- **The 99 % gate rejects 6.5 %, not 16 %,** and gets 0.33 m to DVL DR, not 0.91 m.
- **Gyro-only yaw ends 4.3° from the VN-200,** not 21°.
- **robot_localization is about 4 m from the corrected DVL track,** because it used the same wrong rotation.
- **×20 accelerometer noise is still the best tested setting.**

See `../real_data/README.md` for the table.
