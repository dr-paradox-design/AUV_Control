# Real-log localization (Python) — first work on recorded data

Everything else in this repository runs on **simulated** sensors. This folder is the first work on a
**recorded** log from the vehicle: a data-quality study and a small loosely-coupled EKF compared against the
vehicle's own `robot_localization` output. The vehicle is named `makara` in the ROS log; it is the same
vehicle as manta.

**Status, read this first.**
- **No ground truth.** The log has no independent position reference, so every result here is
  *consistency* with two other estimates that use the same sensors. It is not *accuracy*.
- **Unpowered run.** All eight ESC channels stayed at 1500 µs and `armed` was false for the whole log. The
  vehicle was moved by hand or drifted. Thrust dynamics are not tested.
- **One log** (2026-11-05, 1250 s). Nothing here has been checked on a second recording.
- **Frames differ from the rest of the repo.** This code uses the ROS frames of the log: world `odom` is ENU
  (z up) and `base_link` is FLU (x forward, z up). `estimator_m/` and `plant_m/` use NED and FRD (Fossen).

This is Python on purpose: decoding ROS 2 MCAP bags needs `rosbags`. It does **not** replace the MATLAB
ESKF in `estimator_m/`. See `../decisions/ADR-001-localization-architecture.md` for how the two relate.

## Files

| File | Content |
|---|---|
| `extract.py` | Decodes the bag into one parquet table per topic (`data/extracted/`, gitignored) |
| `ekf_localization.py` | The EKF and the variant definitions (A–F); writes runs to `data/runs/` and `results/variant_diagnostics.txt` |
| `compare_variants.py` | DVL dead reckoning, metrics, `results/variants_table.txt` and `results/variants_comparison.png` |
| `export_mat.py` | Exports the log for the MATLAB ESKF: FLU/ENU → FRD/NED, IMU increments on an 80 Hz grid (`data/real_log.mat`) |
| `compare_eskf.py` | ESKF variants vs robot_localization, variant E and DVL dead reckoning: `results/eskf_table.txt`, `results/eskf_comparison.png` |
| `outage_benchmark.py` | Artificial DVL outages of 2–20 s; scores how well each filter bridges them: `results/outage_table_<L>s.txt` |
| `heading_check.py` | ESKF yaw vs the VN-200 yaw and the filter's own yaw σ: `results/heading_table.txt` |
| `paths.py` | Per-run folders (`REAL_DATA_RUN`) and per-run sensor geometry read from `/tf_static` |
| `compare_run.py` | Any run: ESKF variants and the Python EKF vs every ROS estimator and DVL dead reckoning, per gap-free segment |
| `dvl_mount_fit.py` | Estimates the DVL mounting yaw from IMU vs DVL kinematics (no ground truth needed) |
| `mount_overrides.json` | Corrected mountings for runs whose `/tf_static` is wrong, with the evidence |
| `requirements.txt` | Python packages |
| `results/` | Committed outputs of the run below |

## How to run

The bag (132 MB) is **not** in the repository. Point `extract.py` at the rosbag2 folder that holds
`metadata.yaml` and the `.mcap` file.

```bash
pip install -r real_data/requirements.txt
python real_data/extract.py <path>/sensors_20261105_130603
python real_data/ekf_localization.py                            # 6 variants, ~5 s each
cd real_data && python compare_variants.py
```

## The filter

- **State:** position and velocity in ENU, optionally plus body-frame accelerometer bias (6 or 9 states).
- **Orientation is not estimated.** It is taken from the VN-200 quaternion in `/imu/data` and treated as known.
- **Prediction:** accelerometer (gravity removed) at 80 Hz. Every message first propagates the state to
  its own timestamp (zero-order hold on the last IMU sample).
- **Updates:**
  - depth, with the pressure-sensor lever arm;
  - DVL body velocity, with the lever-arm term ω × r.
  - Both use the Joseph form.
- **Outlier gating:** 99 % χ² test (6.63 for depth, 11.34 for DVL). A measurement is accepted anyway after 10
  consecutive rejections, so the filter cannot lock itself out.
- **DVL dropouts (variants E, F):** when no DVL message has arrived for 0.5 s, a weak "velocity ≈ 0" update is
  applied (σ = 0.1 m/s, 5 Hz).
  - **This is an assumption about this unpowered log, not physics.**
  - Replace it with the thrust-model aid for powered runs.

**Parameters are hand-set**, except depth noise. That is measured from the log as std(first difference)/√2 =
0.080 m, and it includes real bobbing near the surface, not only sensor noise.

## Data-quality findings (2026-11-05 log)

| Item | Finding |
|---|---|
| IMU `/imu/data` | 80 Hz, largest gap 0.39 s |
| Depth `/pressure/depth` | 50 Hz; 0–0.81 m (near the surface the whole time); median step between samples 6.8 cm |
| DVL `/dvl/raw` | ≈7.6 Hz (median Δt 0.13 s); 15 gaps > 1 s, longest 49.5 s (141.4–190.9 s); covariance always 0.01 m²/s² |
| `/odometry/filtered` | ≈54 Hz; yaw equals IMU yaw to ±0.07° (5–95 %); two output gaps (below) |
| Thrusters | all 8 channels 1500 µs; `armed` false throughout |
| Magnetometer | **no topic recorded** |

### robot_localization stops publishing while its inputs keep coming

`/odometry/filtered` is silent during 797.0–825.6 s and 861.9–957.5 s. During both gaps every input keeps
arriving: `/imu/data`, `/dvl/raw`, `/pressure/depth` and the Pico topics, each with its largest message gap
at 0.39 s or less. So the gaps come from the node, not from its sensors.

| Gap | Odom position change across the gap (E, N) | ESKF motion during the gap (E, N) | DVL speed during the gap (median / max) |
|---|---|---|---|
| 797.0–825.6 s | (−0.18, −3.09) m | (−0.29, −0.70) m | 0.04 / 0.35 m/s |
| 861.9–957.5 s | (0.00, 0.00) m, resumes exactly where it stopped | (−0.05, 0.55) m | 0.02 / 0.30 m/s |

- **The first gap ends with a jump of about 2.4 m** that the DVL-aided motion does not support.
- **The second gap ends with the estimate frozen**, although the vehicle moved about half a metre.
- **The cause is not known.** The node's configuration is not in the log, and `robot_localization` is not
  installed on the analysis machine (ROS 2 Humble is), so it could not be re-run on the bag.

### Where the log disagrees with the assumed values in `estimator/config.py`

Lever arms are converted to FRD for comparison: FLU (x, y, z) becomes FRD (x, −y, −z).

| Item | Assumed (config) | In the log |
|---|---|---|
| IMU rate | 100 Hz | 80 Hz |
| DVL rate | 10 Hz | ≈7.6 Hz, with dropouts up to 49.5 s |
| Depth rate | 20 Hz | 50 Hz |
| DVL lever arm (FRD) | (0.20, 0, 0.15) m | (−0.180, 0, 0.109) m: **behind** the body origin, not ahead |
| Depth lever arm (FRD) | (−0.10, 0, 0.05) m | (−0.400, 0, 0) m |
| DVL valid only below | 1.0 m depth | the whole log is shallower than 0.81 m, so this rule would reject every DVL sample |
| Heading aid | magnetometer, 10 Hz | not recorded |

The log's lever arms come from `/tf_static`, which someone entered by hand. They are better evidence than
"ASSUMED", but they are still not a measurement of the hull.

## Results (as run on 2026-11-08)

Full table: `results/variants_table.txt`. Per-variant rejections and NIS: `results/variant_diagnostics.txt`.

| Variant | What it adds | Path (m) | RMS vs odom 0–797 s (m) | Drift in 49.5 s DVL dropout [Δx, Δy] (m) |
|---|---|---|---|---|
| A | first version; time-step error reproduced on purpose | 23.1 | 1.74 | [0.74, 0.33] |
| B | time-step fix | 110.2 | 9.05 | [6.17, 2.71] |
| C | + lever arms, gating, measured depth noise | 111.6 | 7.84 | [6.06, 1.07] |
| D | C + accelerometer bias states | 184.6 | 4.61 | [1.84, −4.81] |
| **E** | C + "velocity ≈ 0" during DVL dropouts | **85.6** | **0.53** | [−0.00, −0.08] |
| F | D + "velocity ≈ 0" during DVL dropouts | 86.5 | 0.48 | [−0.01, −0.09] |
| odom | reference | 87.0 | — | [0.05, −0.16] |
| DVL dead reckoning | reference | 81.8 | — | gap skipped |

- **A loses time.** It propagated only at IMU messages but took Δt from the previous message of any type, so
  the path shrank to about 27 %. Its low RMS only means it stayed near the start. The `legacy_timing` flag
  keeps the error reproducible.
- **Without velocity aiding, dropouts drift by metres** (B, C). Bias states alone (D) only change the
  direction of the drift, because the bias is not observable without a velocity measurement.
- **E was the recommended Python variant.** F is about 5 cm better, which does not justify three more
  states here. It is now superseded as the estimator by the tuned ESKF below, and kept as the baseline.

## The estimator_m ESKF on this log (added 2026-11-08)

`export_mat.py` feeds the log to the MATLAB ESKF. That filter integrates attitude from the raw gyro and uses
the VN-200 yaw as a heading aid. Full details, tuning and caveats are in `../estimator_m/README.md`
("Running on recorded data").

| Estimator | Path (m) | RMS vs robot_localization (m) | RMS vs DVL dead reckoning (m) |
|---|---|---|---|
| **ESKF, NIS-tuned, no gate, "velocity ≈ 0" in dropouts** | 87.1 | **0.39** | **0.35** |
| ESKF, same with a 99 % gate | 137.7 | 2.11 | 2.71 |
| Python variant E | 85.6 | 0.53 | 0.62 |
| robot_localization | 87.0 | — | — |

- **Roll and pitch agree with the VN-200.** The ESKF's raw-gyro attitude matches the VN-200's internal
  solution to 0.34° / 0.27° RMS (yaw 1.18°, which is aided by the VN-200 yaw itself).
- **The datasheet noise was badly wrong for this log.** Accelerometer noise had to be raised ×20 for
  consistent DVL NIS (median 2.12, target 2.37).
- **Gating at 99 % makes the ESKF diverge.** It rejects 15 % of DVL samples even when the vehicle is level,
  drifts, then rejects more.
- **A robust update avoids the spiral.** It caps each outlier's NIS instead of rejecting it, and scores
  0.95 m RMS against robot_localization.
- **The papers changed the heading handling, not the maths.** The observability constraint (Huang et al.)
  had no measurable effect. Using the VN-200 yaw at 0.2 Hz with σ 3°, instead of 10 Hz with σ 1.3°, improved
  agreement to 0.33 m and cut yaw overconfidence tenfold. Details: `../estimator_m/README.md`.
- **The DVL carries position.** In artificial outages, pure inertial drift reaches 0.08 m after 2 s, 1.9 m
  after 10 s and 8.4 m after 20 s. Full tables are in `../estimator_m/README.md`.
- **The unexplained window is explained.** Variant E's 1050–1150 s window was also gating: rerun without
  gating, E moves [0.23, −0.31] m there, the same as DVL dead reckoning ([0.23, −0.33]).

## Results that look too good (read before trusting the numbers)

- **Both references share our sensors.** Odom and DVL dead reckoning use the same DVL and the same VN-200
  heading, so all three could carry the same error.
- **The dropout result is not independent.** Odom also stays almost still during the DVL dropout, so
  agreeing with it there proves nothing about the true motion.
- **The Python variants are not tuned.** Median DVL NIS is 0.49 (E) and 0.18 (F), against 2.37 for a
  well-tuned filter, so their assumed noise is larger than the real errors. The ESKF was tuned with NIS;
  these were not.
- **The references favour using every DVL sample.** Odom and dead reckoning both use every DVL sample, so
  ungated filters agree with them best almost by construction. 2.7 % of DVL innovations are extreme
  (> 99.99 % χ²). Whether rejecting those helps accuracy needs ground truth.

## Next

1. **Record a powered log with a reference:** start and end at a marked point, measured waypoints, camera
   topics, and magnetometer output if the VN-200 driver can publish it.
2. **Decide on DVL outlier handling with that reference.** Options: no gate, a loose gate (99.99 % with a
   cap of 3 gave 0.90 m), or a robust (heavy-tailed) DVL update.
3. **Replace the "velocity ≈ 0" rule** with the thrust-model aid (Phase 4), and re-tune noise on a powered run.

## Second log: tape test, 2026-11-11 (`tape_test_20261111_135132`)

**Running any log:**

```bash
python real_data/extract.py <bag folder> --run <name>
REAL_DATA_RUN=<name> python real_data/export_mat.py
cd estimator_m && octave-cli --eval "setup_paths; run_real_log({'eskf_ungated_zv','eskf_accel30_zv'}, '<name>')"
cd ../real_data && REAL_DATA_RUN=<name> python ekf_localization.py && REAL_DATA_RUN=<name> python compare_run.py
```

Without `--run` / `REAL_DATA_RUN`, the scripts use the original folders, which hold the 2026-11-05 log. Its
results were checked unchanged after this work: the same exported file, and the same ESKF state to 0.

`tape_test_20261111_133345` is 9.6 s long with no depth data, too short to evaluate.

**What differs from the first log** (all of it handled by the code now):

| Item | 2026-11-11 tape test |
|---|---|
| Length | 340.7 s |
| **DVL mounting** | `/tf_static` rotates dvl_link **−135° about z**; the first log had none. Applied when the DVL velocity is rotated into body axes. DVL dead reckoning matches both ROS estimators to 0.53 / 0.57 m with the rotation, 3.7 m without |
| **Recording gaps** | all topics stop at 96.7–106.0 s and 268.0–293.1 s. The ESKF restarts after each gap (`run_segments.m`) and keeps its biases; the Python EKF does not propagate through gaps longer than 0.5 s. Both ROS estimators resume about 0.5 m away, so the vehicle moved during the gaps |
| Depth | `/pressure/depth` has no messages |
| Thrusters | no thruster or arming topics |
| Second ROS estimator | `/odometry/ekf` (22 Hz) next to `/odometry/filtered`. `/ekf/dvl_twist` is an exact copy of `/dvl/raw` |
| IMU | 80 Hz with the two gaps (the 72 Hz average includes them). The frame check holds: 5 s gyro-only vs VN-200, median 0.33° |

**Noise tuning does not carry over between runs.**
- With the first log's ×20 accelerometer noise, DVL median NIS is 4.04 (target 2.37). ×30 gives 1.91, so
  `eskf_accel30_zv` is the NIS-tuned variant for this run.
- The DVL noise σ barely matters (0.05 → 0.12 m/s moves NIS only 4.04 → 3.95).
- A DVL yaw misalignment of ±8° does not change NIS at all. Without a position reference it cannot be
  calibrated: it only rotates the track.

**Results** (`results/tape_test_20261111_135132/compare_table.txt`). RMS is per gap-free segment, each track
re-aligned at its segment start:

| Estimator | Path | vs robot_localization | vs /odometry/ekf | vs DVL DR | DVL NIS |
|---|---|---|---|---|---|
| ESKF ×20 (first-log tuning) | 23.1 m | 0.02 m | 0.02 m | 0.03 m | 4.04 |
| **ESKF ×30 (NIS-tuned here)** | 23.1 m | 0.02 m | 0.02 m | 0.03 m | **1.91** |
| ESKF, heading 0.2 Hz σ 3° | 23.1 m | 0.04 m | 0.03 m | 0.03 m | 4.04 |
| ESKF, robust update | 23.0 m | 0.03 m | 0.03 m | 0.04 m | 4.04 (12 down-weighted) |
| ESKF, 99 % gate | 25.9 m | 0.19 m | 0.19 m | 0.20 m | 25 % of DVL rejected |
| ESKF, no heading aid | 23.2 m | 0.10 m | 0.10 m | 0.10 m | yaw 3.2° off the VN-200 |
| ESKF, datasheet noise | 34.1 m | 0.60 m | 0.60 m | 0.58 m | 35.5 |
| Python EKF E | 22.9 m | 0.02 m | 0.01 m | 0.02 m | — |

- **Every DVL-based estimate agrees within 2–4 cm here.** That includes both ROS estimators, the ESKF, the
  Python EKF and dead reckoning. The motion is gentle (roll −16 to 7°, pitch ±5°), and the DVL data within
  each segment has almost no outliers.
- **The first log's lessons repeat:** the 99 % gate still diverges, and datasheet noise is still far off.
- **The heading-rate change** that helped on the first log makes no difference here.

**The taped course** (from the tracks; the layout still needs confirming):
- the second segment is a closed **rectangle of about 3.0 m (north-south) by 3.2–3.4 m (east-west, including
  corner wiggles)**;
- the third segment is a **straight line of 3.06 m** on every estimator.

If the tape line is 3.00 m, all DVL-based estimates are about 2 % long, which is a DVL scale factor. If the
rectangle was walked back to its start, every estimator's loop-closure error is 0.52–0.58 m. **These become
real accuracy numbers once the tape layout is known.**

## Correction: the DVL on the 2026-11-05 log was mounted at −135°, not 0° (2026-11-11)

**Everything above about the 2026-11-05 log used the DVL rotation from its `/tf_static` (0°), and that was
wrong.**
- **How it came to light:** the team reported that the DVL was misaligned, and `dvl_mount_fit.py` confirms it.
- **The method:** the IMU's linear acceleration must match the change in DVL velocity once the DVL is rotated
  into body axes by the true angle.

| Log | DVL yaw in `/tf_static` | Best fit | Residual: `/tf_static` → best fit |
|---|---|---|---|
| 2026-11-11 tape test (validation) | −135° | −131° | 0.0375 → 0.0373 m/s² |
| **2026-11-05** | **0°** | **−135°** | **0.1166 → 0.0763 m/s²** |

**How it's applied:**
- The corrected log is processed as its own run, `sensors_20261105_130603`, through `mount_overrides.json`.
- The original folders (`data/`, `results/`) keep the uncorrected analysis, so it stays reproducible.

**What changes** (`results/sensors_20261105_130603/compare_table.txt`; RMS per gap-free segment):

| | Uncorrected (0°) | Corrected (−135°) |
|---|---|---|
| ESKF ×20 vs DVL dead reckoning | 0.17 m | 0.17 m |
| ESKF ×20 vs robot_localization | 0.35 m | **3.85 m** |
| robot_localization vs DVL dead reckoning | 0.48 m | **4.30 m** |
| DVL innovations beyond the 99 % χ² value (×20, ungated) | 7.4 % | **3.9 %** |
| 99 % gate: DVL rejected / vs DVL DR | 16 % / 0.91 m | **6.5 % / 0.33 m** |
| Gyro-only yaw vs VN-200 at the end | −21.2° | **−4.3°** |
| Path length (ESKF / DVL DR / robot_localization) | 87.1 / 81.8 / 87.0 m | 79.3 / 74.6 / 87.0 m |

**Revised conclusions for the 2026-11-05 log:**
1. **robot_localization was wrong on this log.** It used the same wrong 0° rotation, so its track is about 4 m RMS
   from the correctly rotated DVL. The earlier "0.39 m agreement" only showed that two estimators shared the same
   error.
2. **A large part of the "DVL outliers" was the mounting.** The extreme tail halves, and the gating spiral is much
   weaker. A real tail remains (3.9 % against 1 %), consistent with the rough hand handling.
3. **Noise tuning barely moves.** ×20 is still the best tested setting; ×5 with the robust update diverges (path
   247 m), and datasheet noise still fails (2.1 m against DVL DR).
4. **Heading: the long-term drift came from the mounting; the short VN-200 jumps did not.**
   - Gyro-only yaw ends 4.3° from the VN-200, against 21° before. Most of the old drift was the wrong DVL
     rotation pulling the filter's yaw.
   - The short VN-200 jumps remain (largest 10 s change 5.6°; 9 windows over 2°).
5. **Still uncorrected:** the outage benchmark, the Python variant table and the literature tests above were run
   on the uncorrected log.

**Lesson:** check `/tf_static` against the data. A mounting error is invisible when every estimator, including
the vehicle's own, shares it. A 30-line kinematic fit finds it without any ground truth.
