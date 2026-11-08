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
| `/odometry/filtered` | ≈54 Hz; gaps 797.0–825.6 s and 861.9–957.5 s, cause unknown; yaw equals IMU yaw to ±0.07° (5–95 %) |
| Thrusters | all 8 channels 1500 µs; `armed` false throughout |
| Magnetometer | **no topic recorded** |

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
- **E is the recommended variant.** F is about 5 cm better, which does not justify three more states here.

## Results that look too good (read before trusting the numbers)

- **Both references share our sensors.** Odom and DVL dead reckoning use the same DVL and the same VN-200
  heading, so all three could carry the same error.
- **The dropout result is not independent.** Odom also stays almost still during the DVL dropout, so
  agreeing with it there proves nothing about the true motion.
- **The noise settings are too conservative.** Median DVL NIS is 0.49 (E) and 0.18 (F), against 2.37 for a
  well-tuned filter. The assumed noise is larger than the real errors.
- **One window disagrees and is not explained.** In 1050–1150 s, E moves [−0.17, −1.09] m, while odom moves
  [−0.12, −0.18] and DVL dead reckoning [0.23, −0.33].

## Next

1. **Record a powered log with a reference:** start and end at a marked point, measured waypoints, camera
   topics, and magnetometer output if the VN-200 driver can publish it.
2. **Export the extracted tables to `.mat`** and run `estimator_m/run_filter.m` on recorded data.
3. **Tune the noise with NIS**, and replace the "velocity ≈ 0" rule with the thrust-model aid (Phase 4).
