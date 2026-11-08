# Literature guide — ESKF with attitude from raw IMU (manta AUV)

Focus: an error-state Kalman filter that estimates attitude from raw VN-200 gyro and accelerometer data,
aided by DVL and depth, compared against the ROS 2 `robot_localization` node, and how to improve it.

**How this guide was built.** Not a new search. The sources are the papers already cited in
`research_notes/Model aided AUV navigation review/` and `reports/Model aided AUV navigation review.md`
(21 DOIs, 23 arXiv IDs, plus author–year citations), with metadata checked against Crossref, OpenAlex and
the arXiv API on 2026-11-08. One source was added because the focus needs it: `moore-2015-generalized`
(robot_localization). Grades are for this focus only:
A = must read for Option C, B = supporting, C = cited but outside this focus (not acquired).

## 1. Weighted source table

| Grade | Autor Jahr | Kurztitel | OA/Zugang | DOI/Link |
|---|---|---|---|---|
| A | Solà 2017 | Quaternion kinematics for the error-state Kalman filter | OA | [arXiv:1711.02508](https://arxiv.org/abs/1711.02508) |
| A | Trawny & Roumeliotis 2005 | Indirect Kalman Filter for 3D Attitude Estimation | no DOI, no OA copy found | — |
| A | Hegrenaes & Hallingstad 2011 | Model-Aided INS With Sea Current Estimation for Robust Underwater Navi | paywalled / no OA copy | [10.1109/JOE.2010.2100470](https://doi.org/10.1109/JOE.2010.2100470) |
| A | Klein & Diamant 2015 | Observability Analysis of DVL/PS Aided INS for  a Maneuvering AUV | OA | [10.3390/s151026818](https://doi.org/10.3390/s151026818) |
| A | Arnold & Medagoda 2018 | Robust Model-Aided Inertial Localization for Autonomous Underwater Veh | OA | [arXiv:1805.08011](https://arxiv.org/abs/1805.08011) |
| A | Moore & Stouch 2015 | A Generalized Extended Kalman Filter Implementation for the Robot Oper | paywalled / no OA copy | [10.1007/978-3-319-08338-4_25](https://doi.org/10.1007/978-3-319-08338-4_25) |
| B | Paull et al. 2014 | AUV Navigation and Localization: A Review | paywalled / no OA copy | [10.1109/JOE.2013.2278891](https://doi.org/10.1109/JOE.2013.2278891) |
| B | Huang et al. 2009 | Observability-based Rules for Designing Consistent EKF SLAM Estimators | paywalled / no OA copy | [10.1177/0278364909353640](https://doi.org/10.1177/0278364909353640) |
| B | Li & Mourikis 2012 | Improving the accuracy of EKF-based visual-inertial odometry | paywalled / no OA copy | [10.1109/ICRA.2012.6225229](https://doi.org/10.1109/ICRA.2012.6225229) |
| B | Bar‐Shalom et al. 2002 | Estimation with Applications to Tracking and Navigation | paywalled / no OA copy | [10.1002/0471221279](https://doi.org/10.1002/0471221279) |
| B | Hegrenæs et al. 2007 | Towards Model-Aided Navigation of Underwater Vehicles | OA | [10.4173/mic.2007.4.3](https://doi.org/10.4173/mic.2007.4.3) |
| B | Or 2025 | Selection-Induced Contraction of Innovation Statistics in Gated Kalman | OA | [arXiv:2512.18508](https://arxiv.org/abs/2512.18508) |
| B | Frutuoso et al. 2022 | Influence of Integration Schemes and Maneuvers on the Initial Alignmen | OA | [10.3390/s22093287](https://doi.org/10.3390/s22093287) |
| B | Frutuoso et al. 2025 | Assessment of Maneuvering Influence on the Fine Alignment of Autonomou | OA | [10.1002/rob.22551](https://doi.org/10.1002/rob.22551) |
| B | Balasubramanian et al. 2021 | Comparison of Dynamic and Kinematic Model Driven Extended Kalman Filte | OA | [arXiv:2105.12309](https://arxiv.org/abs/2105.12309) |
| B | Groves 2013 | Principles of GNSS, Inertial, and Multisensor Integrated Navigation Sy | no DOI, no OA copy found | — |
| B | Fossen 2021 | Handbook of Marine Craft Hydrodynamics and Motion Control, 2nd ed. | paywalled / no OA copy | [10.1002/9781119575016](https://doi.org/10.1002/9781119575016) |

## 2. Why each A/B source is in scope

- `sola-2017-quaternion` (A): ESKF derivation the repo ESKF follows: error-state kinematics, injection, reset
- `trawny-2005-indirect` (A): indirect (error-state) KF for 3-D attitude from gyros; the attitude half of Option C
- `hegrenaes-2011-model` (A): model-aided INS with sea-current estimation; reference design for DVL dropouts
- `klein-2015-observability` (A): observability of DVL + pressure aided INS on a manoeuvring AUV; same sensor set as manta
- `arnold-2018-robust` (A): robust model-aided inertial localization; bridging DVL dropouts with model states
- `moore-2015-generalized` (A): robot_localization itself; the ROS 2 node to compare against (added: not cited in notes)
- `paull-2014-auv` (B): AUV navigation review; context for the seminar and for design choices
- `huang-2009-observability` (B): observability rules for consistent EKFs; relevant to unobservable yaw without a magnetometer
- `li-2012-improving` (B): globally-defined attitude error has better consistency (cited by Sola)
- `barshalom-2002-estimation` (B): NEES/NIS consistency tests and gating; needed to tune against real data
- `hegrenaes-2007-model` (B): open-access precursor of the model-aided INS design
- `or-2025-selection` (B): gating shrinks innovation statistics; affects NIS-based tuning of a gated filter
- `frutuoso-2022-influence` (B): initial alignment and calibration of an AUV INS; needed to start a raw-IMU ESKF
- `frutuoso-2025-assessment` (B): manoeuvre influence on fine alignment of an AUV INS
- `balasubramanian-2021-comparison` (B): kinematic vs dynamic-model EKF for AUV localization; comparison method
- `groves-2013-principles` (B): inertial mechanisation, lever arms, vertical channel; textbook reference
- `fossen-2021-handbook` (B): Fossen conventions used by plant_m and estimator_m; INS chapter

## 3. Grade C (cited in the notes, not acquired for this focus)

- `benzon-2022-open` — An Open-Source Benchmark Simulator: Control of a BlueROV2 Underwater Robot
- `cohen-2022-beamsnet` — BeamsNet: A data-driven Approach Enhancing Doppler Velocity Log Measurements for Autonomou
- `cohen-2022-libeamsnet` — LiBeamsNet: AUV Velocity Vector Estimation in Situations of Limited DVL Beam Measurements
- `cohen-2023-inertial` — Inertial Navigation Meets Deep Learning: A Survey of Current Trends and Future Directions
- `cohen-2023-leveraging` — Leveraging Past DVL Velocity Measurements for Acceleration-Aided AUV Navigation
- `cui-2025-observability` — Observability Analysis on DVL and Range Assisted AUV Navigation in Presence of Unknown Cur
- `damari-2026-ai` — AI-Aided Advancements in Autonomous Underwater Vehicle Navigation
- `damari-2026-unified` — A Unified Neural-Aided Alignment and Calibration Method for AUVs
- `eliav-2018-ins` — INS/Partial DVL Measurements Fusion with Correlated Process and Measurement Noise
- `engelsman-2023-information` — Information Aided Navigation: A Review
- `engelsman-2026-information` — Information-Aided Navigation: Theory and Applications
- `fernandes-2023-cooperative` — Cooperative Localization for Autonomous Underwater Vehicles -- a comprehensive review
- `grip-2013-nonlinear` — Nonlinear observer for GNSS-aided inertial navigation with quaternion-based attitude estim
- `he-2023-deep` — Deep Learning-Based Inertial Navigation Technology for Autonomous Underwater Vehicle Long-
- `inc-2022-baro` — Baro-inertial vertical channel
- `joshi-2025-cascade` — Cascade IPG Observer for Underwater Robot State Estimation
- `kurtz-2019-kalman` — Kalman Filtering with Gaussian Processes Measurement Noise
- `liu-2025-mid` — Mid-Water Ocean Current Field Estimation Using Radial Basis Functions Based on Multibeam B
- `lv-2024-integrated` — An Integrated Navigation Method Aided by Position Correction Model and Velocity Model for 
- `marquardt-2026-online` — Online Localization With Current Disturbances for Autonomous Underwater Vehicles Without G
- `martinez-2015-model` — Model-Aided Navigation with Sea Current Estimation for an Autonomous Underwater Vehicle
- `naik-2026-underwater` — Underwater Dead Reckoning with Deployable Situation-Triggered Covariance Scheduling
- `nascimento-2018-modeling` — Modeling and Soft-fault Diagnosis of Underwater Thrusters with Recurrent Neural Networks
- `randeni-2017-parameter` — Parameter identification of a nonlinear model: replicating the motion response of an auton
- `sahoo-2026-dvl` — DVL-DeepONet: A Physics-Guided Operator Learning for Resilient Underwater Navigation
- `sahoo-2026-pidr` — PiDR: Physics-Informed Inertial Dead Reckoning for Autonomous Platforms
- `saksvik-2021-deep` — A Deep Learning Approach To Dead-Reckoning Navigation For Autonomous Underwater Vehicles W
- `shmaliy-2020-kalman` — Kalman and UFIR state estimation with coloured measurement noise using backward Euler meth
- `shu-2026-attitude` — Attitude-Compensated and Acoustics-Calibrated Model-Aided Navigation Framework for AUVs
- `stolero-2025-auv` — AUV Acceleration Prediction Using DVL and Deep Learning
- `syntakas-2026-ocean` — Ocean Current-Harnessing Stage-Gated MPC: Monotone Cost Shaping and Speed-to-Fly for Energ
- `tokta-2026-data` — A data-driven filter bank framework for IMU-based heave motion estimation
- `yona-2023-missbeamnet` — MissBeamNet: Learning Missing Doppler Velocity Log Beam Measurements

## 4. Gaps

- No source yet on running an ESKF on **pool** data without a magnetometer (yaw unobservable from DVL,
  depth and gravity). Candidate topic for a real literature search.
- No source comparing a custom ESKF against `robot_localization` on the same AUV log.
