# Observability, Consistency, Current/Parameter Estimation and Depth in Sensor-Minimal Model-Aided AUV Navigation

Scope: VN-200 only (MEMS IMU + GNSS + magnetometer). No DVL. No pressure sensor. GNSS only at the surface. A thrust/hydrodynamic-model velocity aid while submerged. Large, uncertain added mass from a flooded cavity.

**How these notes were sourced (read first).** Every full-text fetch in this session was blocked by the network egress proxy: arxiv.org, ncbi/PMC, researchgate, sagepub, wiley, dfki.de, navlab.net, flinders.edu.au, aau.dk and cyberleninka. All findings below therefore come from **search-engine abstracts and snippets only**, plus bibliographic metadata. None of them was checked against the full text. Each item is tagged [abstract/snippet] or [metadata only]. Treat every number as "as quoted in the abstract". The report writer should not add equations, limits or numerical results from these papers unless someone has read the full text.

---

## 1. Observability of INS aided only by a model-derived velocity (no position, no depth), and INS with intermittent GNSS

### Takeaway
No model-aided INS observability analysis covers this exact setup: model velocity only, no depth, intermittent GNSS. The nearest analysis is DVL plus pressure-sensor (PS) aided INS (Klein and Diamant 2015). Its result is that horizontal position (latitude/longitude) error is unobservable for all manoeuvres evaluated. Some attitude and bias states only become observable through manoeuvres. A model-velocity aid is a velocity aid with worse, correlated error, so it can only give the same or weaker observability. Without a depth aid, the vertical channel is also unconstrained.

### Cited Findings
- Klein, I. and Diamant, R., "Observability Analysis of DVL/PS Aided INS for a Maneuvering AUV," *Sensors* 15(10):26818–26837, 2015, DOI 10.3390/s151026818. [metadata + snippet]. The paper uses the observability Gramian to derive the unobservable subspace analytically as a function of vehicle dynamics. A later paper citing it summarises that "INS/DVL/PS integration makes the system unobservable for all maneuvers evaluated, as longitude and latitude errors are unobservable states" — [ResearchGate entry](https://www.researchgate.net/publication/283303357_Observability_Analysis_of_DVLPS_Aided_INS_for_a_Maneuvering_AUV); [search snippet / PMC mirror](https://www.ncbi.nlm.nih.gov/pmc/articles/PMC4634482/)
- A related open-access paper studies how integration schemes and manoeuvres affect AUV initial alignment and calibration using observability and degree-of-observability analysis: "Influence of Integration Schemes and Maneuvers on the Initial Alignment and Calibration of AUVs: Observability and Degree of Observability Analyses," *Sensors* 22(9):3287, 2022, DOI 10.3390/s22093287. [title/DOI only; content not read] — [DOI](https://doi.org/10.3390/s22093287)
- Manoeuvre-dependent observability during fine alignment: Frutuoso et al., "Assessment of Maneuvering Influence on the Fine Alignment of Autonomous Underwater Vehicle," *Journal of Field Robotics*, 2025, DOI 10.1002/rob.22551. [title only] — [Wiley](https://onlinelibrary.wiley.com/doi/10.1002/rob.22551?af=R)
- Instantaneous observability of tightly coupled SINS/GPS during manoeuvres. This is the surfaced-phase analogue, not AUV-specific: "Instantaneous Observability of Tightly Coupled SINS/GPS during Maneuvers," *Sensors* 2016. [title only] — [PubMed](https://pubmed.ncbi.nlm.nih.gov/27240369/)
- Land-vehicle analogue: nonholonomic constraints act as velocity pseudo-measurements, and their observability has been analysed ("Observability Analysis of Non-Holonomic Constraints for Land-Vehicle Navigation Systems"). [title only] — [ResearchGate](https://www.researchgate.net/publication/278245054_Observability_Analysis_of_Non-Holonomic_Constraints_for_Land-Vehicle_Navigation_Systems)
- Model-aided INS with unknown current: snippet text says "If the current is non-zero and unknown, the model is only weakly observable." [snippet; the snippet does not show which paper the sentence comes from — do not attribute it without checking] — [search result set](https://www.researchgate.net/publication/41719915_Towards_Model-Aided_Navigation_of_Underwater_Vehicles)
- Range-only single-beacon analyses (not the user's sensors, but they show the current-observability logic). With range and depth measurements and nonzero yaw rate, the states are observable even at zero flight-path angle with unknown constant currents. Source: "Observability analysis of 3D AUV trimming trajectories in the presence of ocean currents using range and depth measurements," *Annual Reviews in Control*, 2015. [snippet] — [ScienceDirect abstract](https://www.sciencedirect.com/science/article/abs/pii/S1367578815000462); [ResearchGate companion](https://www.researchgate.net/publication/301436179_Observability_Analysis_of_3D_AUV_Trimming_Trajectories_in_the_Presence_of_Ocean_Currents_Using_Single_Beacon_Navigation)
- Observability of DVL plus range-aided AUV navigation with unknown current and range drift (2025 Springer chapter). [title only] — [Springer](https://link.springer.com/chapter/10.1007/978-981-96-2244-3_45)
- With intermittent GNSS, the Kalman filter propagates the INS model and folds in GNSS fixes when the vehicle surfaces to correct accumulated error. [snippet, generic] — [Information Aided Navigation: A Review, arXiv 2301.01114](https://arxiv.org/pdf/2301.01114)
- EKF linearisation can make the linearised system look more observable than the nonlinear one. The covariance then shrinks in directions where there is no information, which is a main cause of inconsistency. Remedies are the Observability-Constrained EKF and the First-Estimates-Jacobian EKF. Source: Huang, Mourikis and Roumeliotis, "Observability-based Rules for Designing Consistent EKF SLAM Estimators," *IJRR* 29(5):502–528, 2010, DOI 10.1177/0278364909353640. [abstract/snippet] — [author PDF](https://www-users.cse.umn.edu/~stergios/papers/IJRR-Observability-based-rules-consistency-2010.pdf)

### Inferences
- Submerged, the user's system has no position or depth measurement at all. Horizontal position, and the integral of any unestimated current, are therefore unobservable between surfacings. This follows from Klein and Diamant, where position was unobservable even with DVL plus PS.
- Roll and pitch are observed through gravity once velocity is aided. Heading is aided by the magnetometer, which is a separate issue. Accelerometer and gyro biases need manoeuvres (turns, accelerations) to separate from attitude, as in the DVL-aided case. This is by analogy and is not a published result for model-aided INS.
- Each GNSS fix at the surface gives a position "tie-point". Over a dive, the position innovation at resurfacing constrains the time-integral of (current + model-velocity error). With one surfacing you cannot separate those two terms. With several dives, different headings and different speeds, they start to become distinguishable. No paper found proves this for model-aided INS.
- Huang et al. predict a specific risk for an EKF/ESKF carrying unobservable states (current, model parameters, vertical states). The filter can falsely collapse covariance on those states. This argues for FEJ/OC-style care or for conservative process noise.

### Gaps
- No formal observability analysis (Gramian, Lie-derivative or piecewise-constant system) was found for INS + vehicle-model velocity + magnetometer + intermittent GNSS **without** a depth sensor. This is a clear gap and could be a contribution.
- No published result was found on how many surfacings, or which inter-surfacing manoeuvres, make current and model-parameter states observable.

---

## 2. Depth/heave without a pressure sensor; vertical-channel instability

### Takeaway
The pure-inertial vertical channel is unstable. Gravity-model feedback makes vertical error grow exponentially, and the standard fix is an external height or depth sensor. No AUV paper was found that navigates submerged **without** a depth sensor. Every model-aided AUV paper found assumes a pressure sensor is available. Depth-free underwater navigation therefore looks like an open problem, and without some vertical aid depth will diverge.

### Cited Findings
- The INS vertical position solution is dynamically unstable. Gravity is computed from estimated height, and this creates feedback that makes vertical error diverge exponentially. In practice the channel is stabilised by a height sensor, which for underwater systems means a depth gauge. [snippet] — [Inside GNSS, "Inertial Error Propagation: Understanding Inertial Behavior"](https://insidegnss.com/inertial-error-propagation-understanding-inertial-behavior/); [IET "Baro-inertial vertical channel" chapter](https://digital-library.theiet.org/doi/10.1049/sbra550e_ch23); [USPTO 5359889 "Vertical position aided inertial navigation system"](https://image-ppubs.uspto.gov/dirsearch-public/print/downloadPdf/5359889)
- In loosely coupled SINS/DVL pseudo-velocity work, the vertical velocity component is typically not used. Trajectory evaluation is done in the horizontal plane only. [snippet] — [Attention-guided TCN pseudo-velocity paper, PMC](https://pmc.ncbi.nlm.nih.gov/articles/PMC13503946/)
- Heave velocity estimation is usually said to be better "especially if the pressure sensor is active." Relative heave is usually taken from the rate of change of measured depth. [snippet] — [search result set incl. Partial-DVL INS fusion, PMC5335996](https://pmc.ncbi.nlm.nih.gov/articles/PMC5335996/)
- Ship heave estimation from MEMS inertial data alone does exist. It relies on heave being oscillatory and zero-mean, using high-pass or filter-bank approaches. Sources: "Attitude and Heave Estimation for Ships using MEMS-based Inertial Measurements" [title only] and "A data-driven filter bank framework for IMU-based heave motion estimation," arXiv 2606.02771 [title only] — [ResearchGate](https://www.researchgate.net/publication/309626518_Attitude_and_Heave_Estimation_for_Ships_using_MEMS-based_Inertial_Measurements); [arXiv](https://arxiv.org/pdf/2606.02771)
- Klein and Diamant (above) treat the pressure sensor as a standard part of the aided INS. [metadata]

### Inferences
- Divergence rate (textbook derivation, not taken from a fetched source): the error-growth time constant is about sqrt(R/(2g)), roughly 9–10 minutes. Over a short dive a MEMS accelerometer bias dominates anyway. A vertical-accelerometer bias b gives depth error of about ½·b·t² before the exponential term matters. The report writer should cite Groves (2013) or Farrell (2008) for the exact expression. This session could not verify it.
- Possible vertical aids for the user's vehicle, none validated in the found literature:
  - (a) Use the hydrodynamic model's heave velocity (w in the body frame) as part of the full 3-D model-velocity aid. This bounds vertical-velocity error, but depth still drifts at the rate of model w error. That error is likely large because of uncertain buoyancy and the flooded-cavity added mass.
  - (b) Use a zero-vertical-velocity or "constant depth" pseudo-measurement during commanded depth-hold segments, with appropriate noise.
  - (c) GNSS altitude at each surfacing resets depth to roughly 0 m. The VN-200 GNSS vertical accuracy is covered by another researcher.
  - (d) Detect the surface from GNSS acquisition or loss as a depth ≈ 0 event.
- Ship-heave methods assume zero-mean oscillation. They cannot track a sustained depth change, so they do not transfer to an AUV holding depth.
- Practical implication for the report: without a pressure sensor, depth is the weakest state. A cheap pressure sensor (for example a Bar30) would remove the single largest observability hole. This is an inference from the literature's universal reliance on depth sensors.

### Gaps
- **No AUV or UUV paper was found that performs submerged navigation without a depth/pressure sensor.** No paper was found that evaluates model-based heave or zero-vertical-velocity pseudo-measurements as a depth substitute. This is a confirmed literature gap within this search budget.
- The vertical-channel time-constant formula and numbers were not verified from a fetched textbook. Cite Groves 2013, Ch. 5, or Farrell 2008 directly.

---

## 3. Sea-current estimation in model-aided INS

### Takeaway
The standard approach models the current as a constant or slowly varying irrotational horizontal current in the navigation frame, carried as filter states. Hegrenæs and Hallingstad (2011, HUGIN 4500) and Martinez et al. (2015, HRC-AUV, 3-DOF linear model) are the main examples. Current is only weakly observable from model aiding alone. The abstracts found do not show a principled way to separate current from model error unless there is an independent velocity reference (DVL or ADCP) or position fixes.

### Cited Findings
- Hegrenæs, Ø. and Hallingstad, O., "Model-Aided INS With Sea Current Estimation for Robust Underwater Navigation," *IEEE J. Oceanic Eng.* 36(2):316–337, 2011, DOI 10.1109/JOE.2010.2100470. Evaluated on HUGIN 4500 raw data. Treats ocean current as constant and irrotational. [metadata + secondary snippets; full text not read; covered in depth by another researcher] — [ResearchGate](https://www.researchgate.net/publication/224238828_Model-Aided_INS_With_Sea_Current_Estimation_for_Robust_Underwater_Navigation)
- Martinez, A., Hernandez, L., Sahli, H., Valeriano-Medina, Y., Orozco-Monteagudo, M. and Garcia-Garcia, D., "Model-Aided Navigation with Sea Current Estimation for an Autonomous Underwater Vehicle," *Int. J. Advanced Robotic Systems*, July 2015, DOI 10.5772/60415. MA-INS on the HRC-AUV, based on a **three-DOF linear dynamic model**, implemented in a Kalman filter. It estimates the sea current in real time as the main environmental disturbance. [abstract] — [SAGE](https://journals.sagepub.com/doi/10.5772/60415); [CyberLeninka](https://cyberleninka.org/article/n/1449049)
- Arnold, S. and Medagoda, L., "Robust Model-Aided Inertial Localization for Autonomous Underwater Vehicles," *ICRA 2018*, arXiv 1805.08011, IEEE doc 8460839. Manifold UKF. **ADCP-aiding provides further information for model-aiding** when DVL bottom lock is lost, which gives water-relative velocity and so constrains current. Uses a tactical-grade IMU (FlatFish AUV), and the filter observes Earth rotation for heading. [abstract] — [arXiv](https://arxiv.org/abs/1805.08011); [IEEE](https://ieeexplore.ieee.org/document/8460839/)
- Marquardt, C. A. and Kang, H., "Online Localization With Current Disturbances for Autonomous Underwater Vehicles Without Global References," *J. Field Robotics*, 2026 (online 15 July 2026), DOI 10.1002/rob.70279. Uses online Pruned Exact Linear Time (PELT) change-point detection to find statistical anomalies such as current disturbances, combined with Kalman filter estimation. Validated with 3 hours of field data at two offshore sites. [abstract] — [Wiley](https://onlinelibrary.wiley.com/doi/abs/10.1002/rob.70279)
- RBF-based mid-water current field estimation from multibeam survey data for AUV navigation: *JMSE* 13(5):841, 2025, DOI 10.3390/jmse13050841. [title only] — [DOI](https://doi.org/10.3390/jmse13050841)
- "Attitude-Compensated and Acoustics-Calibrated Model-Aided Navigation Framework for AUVs," *JMSE* 14(7):612, 2026, DOI 10.3390/jmse14070612. [title only; appeared in model-aided search results] — [DOI](https://doi.org/10.3390/jmse14070612)

### Inferences
- From the filter's point of view, a constant current in the navigation frame and a constant model-velocity bias in the body frame look alike while heading is constant. They separate only when heading changes, because the body-frame bias rotates with the vehicle and the earth-frame current does not. Heading changes during a dive are therefore what make current and model bias distinguishable. This is the standard argument and matches the "weakly observable" snippet, but no paper confirming it was read in this session.
- For the user's vehicle, the large added-mass uncertainty mainly hurts transient (acceleration-phase) velocity prediction. Steady-state speed error comes from damping and thrust-curve error. A speed-proportional error plus a constant current will be poorly separable on straight legs at constant speed.

### Gaps
- No paper was found that gives formal identifiability conditions for (current + model parameter) states in MA-INS without DVL or ADCP.
- Martinez et al.'s quantitative results and current process model could not be read because fetch was blocked.

---

## 4. Online estimation of uncertain hydrodynamic/thruster parameters inside the navigation filter

### Takeaway
Augmenting the state with hydrodynamic or thrust coefficients is common for identification. In navigation, Arnold and Medagoda (2018) explicitly carry drag and thrust model parameters as filter states, so that the correlated nature of model-parameter error is accounted for. The identification literature reports that UKF tends to beat EKF for this. Identifiability problems with only IMU and intermittent GNSS are not addressed in the abstracts found.

### Cited Findings
- Arnold and Medagoda (ICRA 2018): "The drag and thrust model-aiding accounts for the correlated nature of vehicle model parameter error by applying them as states in the filter." [abstract] — [Semantic Scholar](https://www.semanticscholar.org/paper/Robust-Model-Aided-Inertial-Localization-for-Arnold-Medagoda/8eb7dcd6b1cccdfb1a7e4ee7bde5769729317224)
- Augmented-state filtering turns coefficient estimation into state estimation, with the unknown hydrodynamic coefficients appended to the state and estimated online. "Extended and Unscented Kalman filters for parameter estimation of an autonomous underwater vehicle," *Ocean Engineering*, 2014 (PII S0029801814003370). It reports that the UKF consistently outperforms the EKF at comparable complexity. [abstract snippet] — [ScienceDirect abstract](https://www.sciencedirect.com/science/article/abs/pii/S0029801814003370)
- "Identification of an Autonomous Underwater Vehicle hydrodynamic model using three Kalman filters," *Ocean Engineering*, 2021 (PII S0029801821003978). [title only] — [ResearchGate](https://www.researchgate.net/publication/350918901_Identification_of_an_Autonomous_Underwater_Vehicle_hydrodynamic_model_using_three_Kalman_filters)
- "Online parameter identification and dynamic model reconstruction for AUV based on adaptive extended Kalman filter and prediction error method" (AEKF + PEM). [abstract snippet; PubMed listing, year and venue not verified] — [PubMed 42336677](https://pubmed.ncbi.nlm.nih.gov/42336677/)
- Randeni, S., Rypkema, N. R., Fischell, E. M., Forrest, A. L., Benjamin, M. R. and Schmidt, H., "Implementation of a Hydrodynamic Model-Based Navigation System for a Low-Cost AUV Fleet," *IEEE/OES AUV Symposium*, 2018. The hydrodynamic model predicts linear velocities directly from measured angular rates and propeller speed. [abstract] — [MIT DSpace](https://dspace.mit.edu/handle/1721.1/137998)

### Inferences
- Added mass appears only when the vehicle accelerates. In steady cruise it is unobservable from velocity aiding, because its effect on predicted velocity is near zero. It becomes partly identifiable only during speed changes, and then only if there is an independent velocity or position reference. Between surfacings there is none. Online estimation of added mass inside the navigation filter is therefore likely to be poorly conditioned. Offline identification (tank or surface trials with GNSS velocity) plus a conservative fixed uncertainty may be more robust. This is an inference, not a published result.
- A practical middle ground is to estimate a small number of lumped parameters: a thrust gain, a linear/quadratic damping scale, and a body-frame velocity bias. These should be modelled as random walk or first-order Gauss–Markov states. This is in line with Arnold and Medagoda.

### Gaps
- No paper was found that reports identifiability failures (parameter drift or covariance collapse) when hydrodynamic parameters are estimated inside a navigation filter that has no DVL. No paper was found on added-mass estimation in navigation specifically.
- No paper was found on estimating flooded-cavity or entrained-water added mass online.

---

## 5. Filter consistency under model mismatch (NEES/NIS, inflation, coloured model-aid error, correlated pseudo-measurements)

### Takeaway
The consistency tools are standard: Bar-Shalom NEES/NIS with χ² bounds. So are the remedies for coloured noise: Bryson–Henrikson state augmentation or measurement differencing, which have been shown equivalent for AR(1) noise. They are rarely applied rigorously in model-aided AUV work. Arnold and Medagoda (2018) is the main AUV example that treats model error as correlated, through parameter states. No dedicated AUV model-aided consistency study was found.

### Cited Findings
- Bar-Shalom, Y., Li, X. R. and Kirubarajan, T., *Estimation with Applications to Tracking and Navigation: Theory, Algorithms and Software*, Wiley, 2001, DOI 10.1002/0471221279. This is the standard reference for NEES/NIS χ² consistency tests. [bibliographic] — [Wiley](https://onlinelibrary.wiley.com/doi/book/10.1002/0471221279)
- Consistency means the actual MSE matches the filter covariance. It is tested with NEES (needs ground truth) and NIS (innovations only) against χ² confidence regions. [snippet] — [arXiv 2512.18508 "Selection-Induced Contraction of Innovation Statistics in Gated Kalman Filters"](https://arxiv.org/pdf/2512.18508)
- Gating, meaning rejecting outlier innovations, can itself bias innovation statistics low ("selection-induced contraction"). This matters if NIS is used to tune a filter that gates surface fixes. [title/snippet] — [arXiv 2512.18508](https://arxiv.org/pdf/2512.18508)
- Coloured (AR(1)/Gauss–Markov) measurement noise:
  - Bryson and Henrikson's state augmentation and measurement differencing are the two classic approaches.
  - Differencing whitens the noise but correlates it with process noise.
  - The two approaches are theoretically equivalent for AR(1).
  - Sources: [snippet] — [Practical Approaches to Kalman Filtering with Time-Correlated Measurement Errors (ResearchGate)](https://www.researchgate.net/publication/254057826_Practical_Approaches_to_Kalman_Filtering_with_Time-Correlated_Measurement_Errors); [Kalman Filtering with Gaussian Processes Measurement Noise, arXiv 1909.10582](https://arxiv.org/pdf/1909.10582); [Shmaliy et al., IET Signal Processing 2020](https://ietresearch.onlinelibrary.wiley.com/doi/full/10.1049/iet-spr.2019.0166)
- Treating correlated measurement noise as white is overconfident. A "variance inflation factor" has been derived to compensate: "Kalman Filter and Correlated Measurement Noise: The Variance Inflation Factor," IEEE, 2021, doc 9521680. [title/snippet] — [IEEE](https://ieeexplore.ieee.org/document/9521680/)
- INS/partial-DVL fusion with **correlated process and measurement noise** has been studied (ECSA-5 proceedings, DOI 10.3390/ecsa-5-05727). This is directly relevant: a model velocity driven by the same IMU-measured angular rates, as in Randeni et al., is correlated with INS process noise. [title only] — [DOI](https://doi.org/10.3390/ecsa-5-05727)
- EKF inconsistency from spurious observability, and the OC-EKF / FEJ remedies: Huang et al. 2010 (see Section 1). [abstract] — [UMN PDF](https://www-users.cse.umn.edu/~stergios/papers/IJRR-Observability-based-rules-consistency-2010.pdf)
- UKF on manifolds (UKF-M) is reported to give more consistent uncertainty than an EKF for challenging AUV navigation ("A real-time unscented Kalman filter on manifolds for challenging AUV navigation"). [snippet] — [academia.edu](https://www.academia.edu/71055969/A_real_time_unscented_Kalman_filter_on_manifolds_for_challenging_AUV_navigation)

### Inferences
- Model-velocity error is dominated by parameter and current error, so it is strongly time-correlated over a dive. Feeding it as white noise at high rate (for example 50–100 Hz) will make the filter badly overconfident, roughly in proportion to the ratio of update rate to correlation bandwidth. Options:
  - (i) Augment a body-frame Gauss–Markov velocity-bias state (Bryson–Henrikson).
  - (ii) Reduce the aiding rate and/or inflate R.
  - (iii) Carry the parameters as states, as Arnold and Medagoda do.
- A model velocity computed from IMU gyro rates (Randeni-style) shares noise with the INS propagation. This cross-correlation should be modelled or avoided, for example by driving the model from thrust commands only.
- NEES needs ground truth. For this vehicle the only truth is GNSS at the surface. A practical consistency check is the NIS of the **first GNSS fix after each dive**: the normalised resurfacing position innovation, collected over many dives and checked against χ²(2). That quantity directly tests whether the dive-long covariance growth is honest.

### Gaps
- No AUV-specific study was found that runs NEES/NIS consistency evaluation of a model-aided INS. No study was found on resurfacing-innovation consistency for surfacing AUVs.
- No paper was found that quantifies the correlation time of model-aid velocity error for any AUV.

---

## 6. Typical drift numbers: MEMS-IMU-only vs model-aided dead reckoning

### Takeaway
The only hard field numbers found for a MEMS-only, no-DVL vehicle are from Randeni et al. (2018) on Bluefin SandShark AUVs. Position uncertainty with model aiding stayed under 100 m by the end of hour-long missions, against more than 1 km/h of drift for the default IMU-based solution. Those vehicles had a pressure sensor and a well-characterised torpedo hull. Expect worse for a 6-DOF hovering-capable vehicle with uncertain added mass.

### Cited Findings
- Randeni et al. (2018), Bluefin SandShark fleet, MEMS IMU only (no DVL): "model-based localization system was able to limit the position uncertainty to less than 100 m by the end of hour-long missions, whereas the drift in the default IMU-based localization solution was over 1 km per hour." [abstract] — [MIT DSpace](https://dspace.mit.edu/handle/1721.1/137998); [ResearchGate](https://www.researchgate.net/publication/328857560_Implementation_of_a_Hydrodynamic_Model-Based_Navigation_System_for_a_Low-Cost_AUV_Fleet)
- Companion work by the same group: "A Navigation Solution Using a MEMS IMU, Model-Based Dead-Reckoning, and One-Way-Travel-Time Acoustic Range Measurements for Autonomous Underwater Vehicles." It gives DVLs a cost of roughly $20–40k and MEMS IMUs under $100. [snippet] — [ResearchGate](https://www.researchgate.net/publication/325884228_A_Navigation_Solution_Using_a_MEMS_IMU_Model-Based_Dead-Reckoning_and_One-Way-Travel-Time_Acoustic_Range_Measurements_for_Autonomous_Underwater_Vehicles)
- Generic MEMS grades from a vendor guide: gyro drift greater than 60°/h, accelerometer bias 0.01–1 mg, against 0.001–1°/h for higher-grade INS. [snippet; vendor/aggregator, low authority] — [Unmanned Systems Technology guide](https://www.unmannedsystemstechnology.com/resources/inertial-navigation-guide-for-uav-uuv-ugv/)
- A learning-based dead-reckoning alternative for AUVs with limited sensor payloads: "A Deep Learning Approach To Dead-Reckoning Navigation For Autonomous Underwater Vehicles With Limited Sensor Payloads," arXiv 2110.00661. [title only] — [arXiv](https://arxiv.org/pdf/2110.00661)
- Comparison of dynamic-model-driven and kinematic-model-driven EKFs for AUV localisation, arXiv 2105.12309. [title only] — [arXiv](https://arxiv.org/pdf/2105.12309)

### Inferences
- Order of magnitude for planning: an unaided MEMS INS diverges to kilometres within an hour. A well-identified model aid brings this to around 100 m per hour on a torpedo AUV with a depth sensor. For the user's vehicle, short dives with frequent surfacing are the main lever on error, rather than filter sophistication.
- The SandShark numbers are "position uncertainty" (filter covariance), which is not necessarily the same as measured error. The abstract does not say.

### Gaps
- No drift numbers were found for model-aided navigation of a hovering, BlueROV2-class or 6-DOF vehicle without DVL.
- Hegrenæs' and Martinez's quantitative results could not be retrieved because fetch was blocked. Another researcher covers Hegrenæs.

---

## 7. T200 / BlueROV2 thruster and hydrodynamic models (verification of named sources)

### Takeaway
Both named sources exist and are verified: von Benzon et al. 2022 (JMSE) and the Wu 2018 thesis. Note that the thesis author is **Chu-Jou Wu**, not "Chao-Jen". Common T200 models are lookup or polynomial fits of thrust against PWM (1100–1900 µs, deadband around 1500 µs) at a given voltage, usually from Blue Robotics' published data. Some papers use power-law fits of thrust against electrical power.

### Cited Findings
- von Benzon, M., Sørensen, F. F., Uth, E., Jouffroy, J., Liniger, J. and Pedersen, S., "An Open-Source Benchmark Simulator: Control of a BlueROV2 Underwater Robot," *J. Mar. Sci. Eng.* 10(12):1898, 2022, DOI 10.3390/jmse10121898. [abstract]
  - The model is a Simulink implementation based on Fossen's equations, with kinematics, hydrodynamics, a dynamic thruster model and restoring forces.
  - "The hydrodynamic parameters and thruster model have been validated in a test facility."
  - The case study is monopile inspection with a sliding-mode controller.
  - The code is on GitHub (ROV-Simulator).
  - Volume 10, article 1898 is inferred from the AAU PDF filename "jmse_10_01898"; check it before citing.
  - Sources: [Semantic Scholar](https://www.semanticscholar.org/paper/An-Open-Source-Benchmark-Simulator:-Control-of-a-Benzon-S%C3%B8rensen/6148ab3a4db300c0cfbb543ae62210e25dbef960); [AAU portal](https://vbn.aau.dk/en/publications/an-open-source-benchmark-simulator-control-of-a-bluerov2-underwat/); [GitHub](https://github.com/ROV-Simulator/ROV-Simulator)
- Wu, Chu-Jou, "6-DoF Modelling and Control of a Remotely Operated Vehicle," Master of Engineering (Electronics) thesis, Flinders University, July 2018. [abstract]
  - Covers modelling and system identification of the **BlueROV2 Heavy**, including a thruster model and a 6-DoF dynamic model.
  - Compares a nonlinear model-based controller against a simpler controller in simulation.
  - Sources: [Flinders PDF](https://flex.flinders.edu.au/file/27aa0064-9de2-441c-8a17-655405d5fc2e/1/ThesisWu2018.pdf); [Blue Robotics page](https://bluerobotics.com/6-dof-modelling-and-control-of-a-remotely-operated-vehicle/)
- Parameter values from the Wu thesis and von Benzon (added mass, damping, thrust coefficients) could **not** be extracted because fetch was blocked.
- T200 command interface: PWM 1100–1900 µs, 1500 µs stop, forward above about 1525 µs, reverse below about 1475 µs (deadband). [snippet] — [Blue Robotics T200 product page](https://bluerobotics.com/store/thrusters/t100-t200-thrusters/t200-thruster-r2-rp/); [T200/T500 thruster guide PDF](https://cdn.robotshop.com/media/B/Blu/RB-Blu-445/pdf/t200-t500-thruster-guide.pdf)
- A power-law model T = a·P^b, fitted by least squares in log-log space with separate forward and reverse coefficients, calibrated on Blue Robotics' 16 V data. [snippet] — [arXiv 2602.00823, Ocean Current-Harnessing Stage-Gated MPC](https://arxiv.org/pdf/2602.00823)
- "Propeller Characterization Testing of a Blue Robotics T200 Thruster" (ResearchGate, 2023). [title only] — [ResearchGate](https://www.researchgate.net/publication/373891538_Propeller_Characterization_Testing_of_a_Blue_Robotics_T200_Thruster)
- Data-driven thruster modelling and fault diagnosis with RNNs: arXiv 1807.04109. [title only] — [arXiv](https://arxiv.org/pdf/1807.04109)
- Community discussion of BlueROV2 Heavy 6-DOF model parameters; not peer-reviewed. — [Blue Robotics forum](https://discuss.bluerobotics.com/t/bluerov2-heavy-6-dof-model/13065); OsloMet BlueROV2 model repo — [GitHub](https://github.com/OsloMet-OceanLab/BlueROV2)

### Inferences
- The manufacturer's T200 curves are given at fixed supply voltages. The model aid therefore needs battery voltage as an input, or a voltage-scaled lookup. Otherwise the thrust gain drifts as the battery discharges, and that drift looks to the filter exactly like a slowly varying model bias. It is another reason to carry a thrust-gain state.
- BlueROV2 hydrodynamic coefficients should not be reused directly for the manta-ray AUV, which has a different hull and a flooded cavity. They only show method and plausible magnitude.

### Gaps
- No paper was found that uses a T200 thrust model **as a velocity aid inside a navigation filter**. T200 models appear only in control and simulation work.
- The T200 motor/ESC dynamic time constant was not verified from any source in this session.

---

### Consolidated list of open questions (potential research gaps)
1. Observability of INS + model velocity + magnetometer + intermittent surface GNSS, with **no depth sensor**. No analysis found.
2. Submerged AUV navigation with no pressure sensor (model-heave or ZUPT-style vertical aiding). No paper found.
3. Identifiability of added mass and other hydrodynamic parameters inside a navigation filter with no DVL or ADCP. No paper found.
4. NEES/NIS consistency evaluation of model-aided INS, and resurfacing-innovation consistency for surfacing AUVs. No paper found.
5. Correlation time and statistics of model-aid velocity error. No measurement found.
6. Model-aided navigation performance for hovering-capable 6-DOF or BlueROV-class vehicles. No numbers found.
7. T200 thrust model used as a navigation aid. Not found.
