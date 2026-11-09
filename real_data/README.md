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
