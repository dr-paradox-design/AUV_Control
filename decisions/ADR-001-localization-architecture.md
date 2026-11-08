# ADR-001: Localization filter architecture for the manta AUV

**Status:** Proposed
**Date:** 2026-11-08
**Deciders:** [Navigation/software lead], [Team captain]

> **Revision note.** A first draft of this ADR was written before this repository was read. It treated an
> error-state filter as weeks of new work and proposed the small Python EKF as the main design. The repository
> already contains a tested 16-state ESKF (`estimator_m/`), so the decision below has changed: the ESKF is the
> target, and the Python EKF becomes the real-data baseline.

## Context

The vehicle needs a position estimate for SAUVC pool tasks. GPS is unavailable underwater, so position must
come from fusing the VN-200 IMU, the DVL and the pressure sensor. The vehicle currently runs
`robot_localization`, published on `/odometry/filtered`. In the ROS log the vehicle is named `makara`; it is
the same vehicle as manta.

**What the repository already has (all on simulated data):**
- **`estimator_m/`, the main version:** a 16-state ESKF in MATLAB.
  - Earth rate and Coriolis terms, lever arms, depth bias, random-walk or Gauss–Markov IMU biases.
  - Updates from DVL, depth, GNSS and magnetometer heading.
  - Checked by tests: Jacobians match finite differences, Monte Carlo consistency (ANEES 16.32, dof 16),
    and a cross-check that matches the frozen Python reference to floating-point rounding.
- **`plant_m/`:** a 6-DOF plant with thrust allocation, intended for a thrust-model aid (Phase 4).
- **What the READMEs list as not done:** running on recorded data, and comparing with the VN-200's internal
  solution.

**What the first recorded log shows** (2026-11-05, 1250 s; details in `real_data/README.md`):
- **Sensor rates and quality:**
  - IMU at 80 Hz.
  - Depth at 50 Hz with ≈0.08 m of noise near the surface.
  - DVL at ≈7.6 Hz with 15 gaps longer than 1 s (the longest is 49.5 s) and a constant, uninformative
    covariance.
- **No magnetometer topic.**
- **Assumed values are contradicted:**
  - the DVL lever arm has the opposite x sign;
  - the pressure-sensor lever arm is −0.40 m, not −0.10 m;
  - the "DVL valid only below 1 m" rule would reject the whole log.
- **Unpowered run, no ground truth.** Results measure consistency, not accuracy.
- **Python EKF in `real_data/`, variant E:** 0.53 m RMS against `robot_localization` over 0–797 s.
  Its path is 85.6 m, against 87.0 m for odom and 81.8 m for DVL dead reckoning.
- **Odom output gaps:** `/odometry/filtered` has two gaps (797.0–825.6 s and 861.9–957.5 s) with unknown cause.

**Forces:**
- A navigation failure costs a competition attempt.
- Team and pool time are limited.
- One tested design is easier to maintain than two parallel ones.
- The seminar needs an explainable design.

## Decision

**Proposed:**
- **Target estimator:** the existing ESKF in `estimator_m/`. Adapt it to recorded data instead of building a
  new filter.
- **Baseline and data tooling:** the Python EKF in `real_data/`. Recorded data is decoded and checked there,
  and the ESKF must match or beat its result on the same log before it is trusted.
- **On the vehicle:** keep `robot_localization` as the live estimator until the ESKF has been checked against
  a log with an independent position reference.

## Options considered

### Option A: Keep and tune `robot_localization`

| Dimension | Assessment |
|---|---|
| Complexity | Low: configuration only |
| Cost | Low; already deployed |
| Extensibility | Medium: generic model; hard to add a thrust-model aid or custom dropout logic |
| Team familiarity | Configured by the team (confirm who maintains it) |
| Evidence on this log | Consistent with DVL dead reckoning; two unexplained output gaps |

**Pros:**
- Running today.
- Widely used.
- Has per-sensor Mahalanobis rejection.

**Cons:**
- A black box to the team, and its output gaps are unexplained.
- No path to the thrust-model aid that `plant_m/` is being built for.

### Option B: Python loosely-coupled EKF (`real_data/`) as the main estimator

| Dimension | Assessment |
|---|---|
| Complexity | Low: about 170 lines, 6 states |
| Cost | Exists; would need a ROS 2 port |
| Extensibility | Medium: no attitude or gyro states; would grow into a second ESKF |
| Team familiarity | High |
| Evidence on this log | 0.53 m RMS vs odom; < 0.1 m drift in the 49.5 s dropout, using the "velocity ≈ 0" assumption |

**Pros:**
- Simple.
- Already works on recorded data.
- Easy to explain.

**Cons:**
- Trusts the VN-200 attitude completely.
- Its dropout rule only fits unpowered runs.
- Making it the main estimator would duplicate the ESKF and split effort between two estimators.

### Option C: Adapt the existing ESKF (`estimator_m/`) to recorded data — **proposed**

| Dimension | Assessment |
|---|---|
| Complexity | High, but already written and tested |
| Cost | An adapter, not a new filter (see action items) |
| Extensibility | Highest: attitude and bias states, heading update, a planned thrust-model aid |
| Team familiarity | Derivation in `log/2026-10-04_estimator_log.docx`; tests document the conventions |
| Evidence on this log | **Not run yet.** No MATLAB or Octave was available where the log was analysed |

**Pros:**
- One main estimator with tests already in place.
- Directly answers the "not done: recorded data" item in the estimator README.

**Cons:**
- **Needs a data adapter:**
  - ROS FLU/ENU to FRD/NED;
  - IMU rates resampled to delta-angle and delta-velocity on a fixed time step (the log has jitter up to
    0.39 s);
  - measurements mapped to IMU sample numbers;
  - a run mode without truth.
- **No logged heading source.** Until a magnetometer is logged, the heading update has to use the VN-200's own
  yaw, which keeps the dependence on its internal filter.
- **Assumed parameters must be replaced first**, at least the lever arms, sensor rates and the DVL
  minimum-depth rule.

*Briefly considered:* a factor-graph smoother. It was rejected for now: there are no loop closures or landmark
measurements to justify it.

## Trade-off analysis

- **One estimator vs two.** Building the Python EKF into the main estimator would duplicate work the ESKF
  already does, and it would be tested less. Keeping it small, as a baseline and decoding tool, gets the
  benefit of real-data checks without splitting effort.
- **Readiness vs capability.** The Python EKF works on the log today, while the ESKF needs an adapter. The
  adapter is bounded work. Upgrading the Python EKF to the ESKF's capability is not.
- **Heading.** Without a magnetometer, neither filter observes heading on its own. Both depend on the VN-200's
  yaw for now. That doesn't favour either option, but it is the largest open risk.
- **Evidence quality.** Every comparison so far is consistency on one unpowered log. That is why the status is
  *Proposed*.

## Consequences

**Easier:**
- Testing on recorded data: decoding, data-quality checks and a baseline result already exist.
- One estimator to maintain, tune and present.

**Harder:**
- Two languages: Python for ROS data, MATLAB for the estimator. The `.mat` export becomes an interface that
  must be kept stable.

**To revisit:**
- Feeding VN-200 yaw as the heading measurement, once a magnetometer or visual heading is logged.
- The "velocity ≈ 0" dropout rule in the baseline, once powered logs and the thrust-model aid exist.
- This decision, once a log with ground truth is available.

## Evidence from running Option C on the log (2026-11-08)

Details: `estimator_m/README.md`, "Running on recorded data".
- **It works on real data.**
  - Attitude is integrated from the raw gyro, corrected by gravity, with the VN-200 yaw as a heading aid.
    It matches the VN-200's own attitude to 0.34° roll, 0.27° pitch and 1.18° yaw RMS.
  - With noise tuned on NIS, no gate, and the "velocity ≈ 0" aid in DVL dropouts, it agrees with
    `robot_localization` to 0.39 m RMS and with DVL dead reckoning to 0.35 m. The Python baseline (variant E)
    gets 0.53 m and 0.62 m.
- **Two lessons that apply to any filter on this vehicle:**
  - The datasheet accelerometer noise is about 20× too optimistic for this log.
  - A 99 % innovation gate makes the filter diverge: it rejects 15 % of DVL samples when level, drifts, and
    rejects more.
- **Robust update instead of gating.** Outliers are down-weighted so their NIS is capped at the 99 % value.
  No data is dropped, so it does not diverge. Against the references it scores 0.95 m RMS.
- **Bridging DVL outages** (artificial 2–20 s outages):
  - Pure inertial error grows about as ½ · 0.04 m/s² · t²: 0.08 m after 2 s, 1.9 m after 10 s, 8.4 m after
    20 s.
  - On this slow, unpowered log, "velocity ≈ 0" does as well as any filter, because the vehicle hardly moved.
  - The DVL carries position, so reducing DVL dropouts and adding the thrust-model aid matter more than the
    choice between filters.
- **robot_localization's two output gaps** happen while all of its inputs keep arriving. The first ends with
  a 2.4 m jump that the DVL-aided motion does not support; the second ends frozen.
- **Still not decided:** accuracy. Both references use every DVL sample, so agreement favours ungated
  filters. The status stays *Proposed* until a log with ground truth exists.

## Action items

1. [x] Log-supported lever arms, rates and measured depth noise are in `estimator_m/config_real_log.m`.
   It is a separate file, so `config_estimator.m` keeps the simulation values. The DVL minimum-depth rule is
   not applied to recorded data.
2. [x] Export to `.mat`: `real_data/export_mat.py`. It does the frame conversion, writes IMU increments on an
   80 Hz grid, indexes measurements by IMU sample, and adds the VN-200 yaw at 10 Hz. The frames were checked
   by gyro-only integration against the VN-200 attitude.
3. [x] Truth-free mode, optional χ² gating and a `zero_vel` pseudo-measurement in `run_filter` /
   `eskf_update` / `eskf_meas`. Tests pass, including the new `test_gating`.
4. [x] Ran on the 2026-11-05 log and compared: `real_data/compare_eskf.py`, `real_data/results/eskf_table.txt`.
5. [ ] Record a powered reference log: start and end at a marked point, measured waypoints, camera topics,
   magnetometer output.
6. [~] Investigate the `/odometry/filtered` gaps (797.0–825.6 s, 861.9–957.5 s). Inputs were continuous,
   so the node stopped publishing (see `real_data/README.md`). Its cause needs the node's config and logs.
7. [ ] Decide on the switch from `robot_localization` (update this ADR to *Accepted* or *Superseded*).
8. [ ] With the reference log, decide DVL outlier handling: no gate, a loose gate (99.99 %, cap 3), or a
   robust heavy-tailed DVL update. Re-tune noise with NIS on a powered run.
9. [ ] Replace the "velocity ≈ 0" dropout aid with the thrust-model aid from `plant_m/` before any powered
   run.
10. [ ] Port the ESKF to a ROS 2 node and run it in shadow mode next to `robot_localization`.
