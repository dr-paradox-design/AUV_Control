# Field Map and Lineage of Model-Aided AUV Navigation (from arXiv 2301.01114 through 2026)

> **Method caveat (read first):** In this environment the egress proxy blocked direct fetching of arxiv.org, doi.org, mdpi.com, ncbi/PMC, researchgate, dfki.de, crossref and semanticscholar. **Every finding below comes from web-search result snippets and abstracts, not full text.** Nothing here was read from the paper bodies. Treat all numbers as "abstract/snippet-level". Every paper listed came up in at least one search result with matching title, authors and venue; papers I could not confirm that way are not listed. The search tool's summariser once credited Hegrenæs's MA-INS work to Engelsman & Klein. I corrected that attribution: the HUGIN MA-INS work belongs to Hegrenæs & Hallingstad.

## Q1. arXiv 2301.01114: bibliographic facts, taxonomy, AUV section, model-aided papers cited

### Takeaway
"Information Aided Navigation: A Review" is by Daniel Engelsman and Itzik Klein (Univ. of Haifa), published in IEEE Transactions on Instrumentation and Measurement, vol. 72, 2023, pp. 1–18. It sorts aiding into three groups: **direct information aiding**, **indirect information aiding** and **model-based aiding**. In model-based aiding, a vehicle model runs in parallel with the INS, and the two feed each other. Within the underwater part, it describes a progression from Hegrenæs's kinetic-model MA-INS with sea-current estimation, through Lammas (2010) and Allotta (2016), to Arnold (2018).

### Cited Findings
- Title "Information Aided Navigation: A Review", authors Daniel Engelsman and Itzik Klein, IEEE Trans. Instrumentation and Measurement vol. 72, pp. 1–18, 2023; arXiv 2301.01114 — [arXiv](https://arxiv.org/abs/2301.01114)
- Problem framing: INS performance depends on a steady flow of external measurements to keep the filter updated and bound drift. Platforms in some environments cannot receive those measurements, so their solution drifts — [arXiv abstract](https://arxiv.org/abs/2301.01114)
- The three taxonomy groups — [arXiv abstract](https://arxiv.org/abs/2301.01114):
  - **Direct** information aiding: knowledge about the platform or its environment becomes pseudo-measurements, for example motion constraints.
  - **Indirect** information aiding: extra information is extracted from external sensors and imposed on the filter indirectly.
  - **Model-based** aiding: "a vehicle simulation runs in parallel to the INS model, enabling simultaneous fusion with the navigation solution". In the snippet's wording, both "feed each other reciprocally and enable state estimation of their errors".
- Underwater model-aided works the review cites, with the review's characterisation (search-snippet text attributed to arXiv 2301.01114) — [arXiv PDF](https://arxiv.org/pdf/2301.01114):
  - **Hegrenæs (2008)**: "proposed several model-aided implementations where not only the hydrodynamic model is addressed, but also real-time sea currents are mathematically formulated."
  - **Lammas (2010)**: "used a VD [vehicle-dynamics]-based model to extract kinematic states from the AUV's equations of motion whenever the acoustic sensors experience outage."
  - **Allotta (2016)**: "introduced an innovative navigation strategy based on kinematics derived from a Typhoon AUV hydrodynamic model and estimated by unscented KF."
  - **Arnold (2018)**: "used a hydrodynamic-based motion model for the FlatFish AUV to aid DVL measurements during drop outs caused by bottom locks."
- A companion book chapter by the same group exists: "Information-Aided Navigation: Theory and Applications" (Springer reference work, DOI prefix 10.1007/978-981-99-1650-4_55-1) — [Springer](https://link.springer.com/rwe/10.1007/978-981-99-1650-4_55-1)
- A related ResearchGate entry is titled "Information Aided **Inertial** Navigation: A Review". It may be a later or variant version — [ResearchGate](https://www.researchgate.net/publication/375535790_Information_Aided_Inertial_Navigation_A_Review)

### Inferences
- The review traces the underwater thread from kinematic dead reckoning, to a dynamic/kinetic model as the velocity aid (Hegrenæs, Lammas), to a model inside a nonlinear filter (Allotta UKF), to a model as a DVL-dropout bridge with model parameters as filter states (Arnold). In this map, "hybrid" means model plus INS plus intermittent DVL, USBL or depth.
- None of the four works it cites had the user's sensor setup. All had a depth sensor, and most had a DVL or acoustic positioning.

### Gaps
- I could not read the full text, so I could not confirm the section headings (for example whether the review has an explicit "kinematic → dynamic → hybrid" subsection), the full model-aided reference list, or any quantitative comparison table it may contain.
- I could not verify whether the review states explicit open problems for the underwater model-aided case.

## Q2. Other surveys to cross-check (2006–2026)

### Takeaway
The classic surveys are Kinsey, Eustice & Whitcomb (2006) and Paull et al. (2014). The most recent, Damari … Klein (arXiv, May 2026), focuses on AI-aided INS/DVL/camera fusion rather than DVL-free model-aiding. I found no 2020–2026 review dedicated specifically to DVL-free model-aided navigation.

### Cited Findings
- Kinsey, J. C., Eustice, R. M., Whitcomb, L. L. (2006), "A survey of underwater vehicle navigation: Recent advances and new challenges", invited paper at the 7th IFAC Conference on Manoeuvring and Control of Marine Craft (MCMC), Lisbon, Sept 2006 — [WHOI Kinsey publication list](https://www2.whoi.edu/site/jameskinsey/pubs/full-length-conference-publications/)
- Paull, L., Saeedi, S., Seto, M., Li, H., "AUV Navigation and Localization: A Review", IEEE J. Oceanic Engineering 39(1):131–149, Jan 2014. It reviews the state of the art and future research directions, motivated by rapid attenuation of GPS and RF underwater — [PDF copy](https://uml.iut.nsysu.edu.tw/hhchen/Handouts/2014%20AUV_Navigation_and_Localization_A_Review.pdf); [Semantic Scholar](https://www.semanticscholar.org/paper/AUV-Navigation-and-Localization:-A-Review-Paull-Saeedi/b141c78f429df09b532b8c996b321eae5983f27e)
- Damari, G., Yampolsky, Z., Cohen, N., Sahoo, A. K., Danial, J., Silva, F. O., Klein, I., "AI-Aided Advancements in Autonomous Underwater Vehicle Navigation", arXiv 2605.04672, submitted 6 May 2026. It covers AI-aided INS/DVL/camera fusion, learning for inertial dead reckoning, and adaptive fusion — [arXiv](https://arxiv.org/abs/2605.04672)
- Cohen, N. & Klein, I., "Inertial Navigation Meets Deep Learning: A Survey of Current Trends and Future Directions", arXiv 2307.00014. It is also listed on ScienceDirect (journal "Results in Engineering", PII S2590123024018085, 2024) — [arXiv](https://arxiv.org/abs/2307.00014)
- "Deep Learning-Based Inertial Navigation Technology for Autonomous Underwater Vehicle Long-Distance Navigation—A Review", Gyroscopy and Navigation (Springer), 2023, DOI 10.1134/S2075108723030070 — [Springer](https://link.springer.com/article/10.1134/S2075108723030070) (title only; authors not captured)
- "Cooperative Localization for Autonomous Underwater Vehicles — a comprehensive review", arXiv 2307.06189 — [emergentmind listing](https://www.emergentmind.com/papers/2307.06189) (title only)

### Inferences
- From abstracts alone, Paull 2014 and Kinsey 2006 treat dead reckoning and dynamic-model-based estimation as one branch next to acoustic and geophysical navigation. The 2023 Engelsman & Klein review is the first survey found that names model-based aiding as its own class.

### Gaps
- Leonard & Bahr's "Autonomous Underwater Vehicle Navigation" (Springer Handbook of Ocean Engineering, 2016) did not come up in my searches, so I did not verify it.
- I found no dedicated review of "DVL-denied model-aided navigation".

## Q3. Key model-aided / dynamic-model-aided works after Hegrenæs (with sensors used)

### Takeaway
After Hegrenæs, the lineage has four branches. Below, ✔ means the vehicle had the sensor and ✘ means it did not.
1. **Hydrodynamic model as velocity aid with sea-current or water-column estimation:** Lammas 2010; Randeni 2018 a/b; Arnold & Medagoda 2018.
2. **Model inside nonlinear filters:** Allotta UKF 2016.
3. **Thruster-driven small vehicles that recalibrate the model at the surface with GPS:** Ji et al. 2023, the closest to the user.
4. **Data-driven or hybrid velocity models:** Saksvik 2021 RNN; Lv 2024 OP-ELM; the Klein-group BeamsNet family and PiDR; a 2026 JMSE attitude-compensated MA-nav.

Almost all of these keep a depth/pressure sensor. Most use a DVL, either as the main sensor or for training and ground truth.

### Cited Findings
| Work | Contribution (one line) | DVL? | Depth? | Source |
|---|---|---|---|---|
| Lammas, Sammut & He (2010), "6-DoF navigation systems for AUVs" (book chapter in *Mobile Robots Navigation*) | Uses a vehicle-dynamics model to extract kinematic states from the equations of motion during acoustic-sensor outage | Acoustic aiding normally present (details unverified) | unverified | [ResearchGate entry](https://www.researchgate.net/publication/221908018_6-DoF_Navigation_Systems_for_Autonomous_Underwater_Vehicles); characterisation from [2301.01114](https://arxiv.org/pdf/2301.01114) |
| Allotta et al., "A new AUV navigation system exploiting unscented Kalman filter", Ocean Engineering (2016; ScienceDirect PII S0029801815007271) | UKF navigation on the Typhoon AUV using kinematics derived from its hydrodynamic model. Better than EKF in discontinuous or strongly nonlinear situations. Sea-tested at Biograd na Moru (BtS 2014, FP7 ARROWS) | Typhoon carries a DVL (unverified from abstract) | unverified | [ScienceDirect abstract page](https://www.sciencedirect.com/science/article/abs/pii/S0029801815007271) |
| Allotta group, "A forward-looking SONAR and dynamic model-based AUV navigation strategy: Preliminary validation with FeelHippo AUV", Ocean Engineering (PII S0029801819308741) | Combines FLS odometry with a dynamic model. FeelHippo is a small thruster-driven AUV | unverified | unverified | [ScienceDirect](https://www.sciencedirect.com/science/article/abs/pii/S0029801819308741) (title only) |
| Arnold, S. & Medagoda, L., "Robust Model-Aided Inertial Localization for Autonomous Underwater Vehicles", ICRA 2018, arXiv 1805.08011 | Manifold UKF. Drag and thrust model-aiding, with model-parameter errors carried as filter states (correlated errors). ADCP incorporation. Gyrocompassing with a tactical-grade IMU. Bridges DVL dropouts on the FlatFish AUV | ✔ (aided during dropouts) | ✔ (assumed; unverified) | [arXiv](https://arxiv.org/abs/1805.08011) |
| Randeni, S. A. T. et al., "Parameter identification of a nonlinear model: replicating the motion response of an AUV for dynamic environments", Nonlinear Dynamics 91(2):1229–1247, Jan 2018 | Recursive-least-squares identification of a Gavia-class AUV model. Inputs are **only propeller thrust, gyro measurements and hydrodynamic/hydrostatic/mass parameters**, and it outputs u, v, w. A calibration mission re-tunes the baseline model for a new environment | Used for calibration and truth | unverified | [Springer](https://link.springer.com/article/10.1007/s11071-017-3941-z) |
| Randeni, Rypkema, Fischell, Forrest, Benjamin, Schmidt, "Implementation of a Hydrodynamic Model-Based Navigation System for a Low-Cost AUV Fleet", IEEE AUV Symposium, Porto, Nov 2018 | Model-based navigation on a fleet of low-cost **Bluefin SandShark** AUVs (no DVL on SandShark, per the figure caption context) | ✘ (low-cost fleet; confirm) | unverified | [IEEE Xplore](https://ieeexplore.ieee.org/abstract/document/8729758/); [MIT DSpace](https://dspace.mit.edu/handle/1721.1/137998) |
| Randeni et al., water-column-current-aided model localisation (title as indexed: "Water column current aided localisation for significant horizontal trajectories with AUVs") | Non-acoustic estimate of water-column velocity from the AUV's motion response, combined with the model-aided solution | — | — | [ResearchGate entry](https://www.researchgate.net/publication/230642972_Water_column_current_aided_localisation_for_significant_horizontal_trajectories_with_Autonomous_Underwater_Vehicles) (title/abstract only; authorship of this specific entry unverified) |
| Saksvik, Alcocer & Hassani, "A Deep Learning Approach To Dead-Reckoning Navigation For AUVs With Limited Sensor Payloads", OCEANS 2021, arXiv 2110.00661 | RNN predicts horizontal relative velocities from **IMU, pressure sensor and control inputs**. DVL is used only as training ground truth. Tested on LRAUV data (Monterey Bay) and a simulated glider | Training only | ✔ | [arXiv](https://arxiv.org/abs/2110.00661); [IEEE](https://ieeexplore.ieee.org/document/9706096/) |
| Balasubramanian, Rajput, Hascaryo, Rastogi, Norris, "Comparison of Dynamic and Kinematic Model Driven EKFs for the Localization of AUVs", arXiv 2105.12309 | Simulation (UUV Simulator, RexROV, a **thruster-driven ROV**). A dynamic-model EKF predicts better than a kinematic one. The authors say it is not yet real-time ready | sim | sim | [arXiv](https://arxiv.org/abs/2105.12309) |
| Ji, D., Cheng, H., Zhou, S., Li, S., "Dynamic model based integrated navigation for a small and low cost autonomous surface/underwater vehicle", Ocean Engineering 276, 15 May 2023 | **Six-thruster** small S-ASUV. At the surface, an adaptive KF fuses GPS with the dynamic model. Underwater, a dynamic-model-aided INS. **Thrust-model coefficients are corrected online each time it surfaces.** Field-tested | ✘ (as described) | unverified | [SSRN preprint](https://papers.ssrn.com/sol3/papers.cfm?abstract_id=4251644); [ScienceDirect](https://www.sciencedirect.com/science/article/abs/pii/S0029801823004754) |
| Lv, P., Lv, J., Hong, Z., Xu, L., "An Integrated Navigation Method Aided by Position Correction Model and Velocity Model for AUVs", Sensors 24(16):5396, Aug 2024, DOI 10.3390/s24165396 | A velocity model (dynamic model plus OP-ELM, trained online) fills the gaps **between DVL updates**. A hybrid gated RNN (HGRNN) predicts position corrections | ✔ (intermittent) | unverified | [PubMed](https://pubmed.ncbi.nlm.nih.gov/39205090/) |
| "Attitude-Compensated and Acoustics-Calibrated Model-Aided Navigation Framework for AUVs", JMSE 14(7):612, 26 Mar 2026, DOI 10.3390/jmse14070612 | Corrects propeller-speed-to-velocity model errors. Uses **pressure-derived vertical velocity** for attitude-induced error, and refines the propeller-to-velocity map online with LBL fixes | ✘ (model replaces it) | ✔ | [MDPI](https://www.mdpi.com/2077-1312/14/7/612) (authors not captured) |
| Klein group, DVL-degradation learning: BeamsNet (arXiv 2206.13603), LiBeamsNet (arXiv 2210.11572), MissBeamNet (Yona & Klein, arXiv 2301.11597), ST-BeamsNet | 1D-CNN or set-transformer networks regress the DVL velocity from IMU data plus partial or past beams. ST-BeamsNet handles **complete DVL outage** using inertial data and past DVL velocities | ✔ (needs DVL history) | — | [BeamsNet](https://arxiv.org/pdf/2206.13603); [LiBeamsNet](https://arxiv.org/abs/2210.11572); [MissBeamNet](https://arxiv.org/pdf/2301.11597) |
| Klein group, "AUV Acceleration Prediction Using DVL and Deep Learning", arXiv 2503.16573; "INS/DVL Fusion with DVL Based Acceleration Measurements", arXiv 2308.11762 | DVL-derived acceleration as an extra aiding signal | ✔ | — | [2503.16573](https://arxiv.org/pdf/2503.16573); [2308.11762](https://arxiv.org/pdf/2308.11762) |
| "DVL-DeepONet: A Physics-Guided Operator Learning for Resilient Underwater Navigation", arXiv 2606.23502 (2026) | A physics-guided neural operator estimates the velocity vector under missing DVL beams | ✔ (partial) | — | [arXiv](https://arxiv.org/html/2606.23502) |
| Sahoo, A. K. & Klein, I., "PiDR: Physics-Informed Inertial Dead Reckoning for Autonomous Platforms", arXiv 2601.03040 (Jan 2026, rev. Jun 2026) | A PINN embeds the strapdown INS equations for **pure inertial** dead reckoning. Tested on a mobile robot and an AUV dataset | ✘ at inference | — | [arXiv](https://arxiv.org/abs/2601.03040) |
| "A Unified Neural-Aided Alignment and Calibration Method for AUVs", arXiv 2608.22496 (2026) | Neural-aided alignment and calibration | unverified | — | [arXiv](https://arxiv.org/pdf/2608.22496) (title only) |

### Inferences
- The field has moved in two directions since Arnold 2018. One keeps the physics model and fixes its main weakness, model-parameter error: parameters as filter states (Arnold), calibration missions (Randeni), recalibration at each surfacing (Ji 2023), and online LBL calibration (JMSE 2026). The other replaces the model with learned regressors (Saksvik, Lv, BeamsNet family, PiDR), but most of these still need a DVL for training or partial input.
- The Fossen-group line (NTNU) is mainly Hegrenæs, so it is outside my scope. I found no newer Fossen-authored model-aided INS paper in these searches.

### Gaps
- I could not confirm the authors of the JMSE 2026 paper or of the ST-BeamsNet publication venue.
- I could not confirm the sensor suites (depth sensor present?) for Allotta 2016, Ji 2023 or Randeni 2018 from abstracts.
- I did not find a "Tal" or "Lu" model-aided AUV paper. I could not verify the requested names and do not list them.

## Q4. Works on small, thruster-driven / BlueROV2-class platforms

### Takeaway
The closest prior art to the user's case is **Ji et al. 2023**: a six-thruster small surface/underwater vehicle with GPS at the surface, a dynamic-model-aided INS underwater, and thrust-model coefficients recalibrated each time it surfaces. BlueROV2 navigation papers found so far all use a DVL (Water Linked A50) or are not model-aided.

### Cited Findings
- **Ji et al. 2023**: an S-ASUV with six thrusters. At the surface, an adaptive KF combines GPS and the dynamic model. Underwater, a dynamic-model-aided INS. "Thrust model coefficients were corrected online by the S-ASUV surfacing." Field experiments with a prototype — [SSRN](https://papers.ssrn.com/sol3/papers.cfm?abstract_id=4251644)
- **Naik, Sreenivas, Nottage, Soylemezoglu (UIUC / US Army ERDC)**, "Underwater Dead Reckoning with Deployable Situation-Triggered Covariance Scheduling", arXiv 2607.10597 (2026, submitted to IEEE JOE). A situation-triggered calibrated adaptive robust EKF on a **BlueROV2**: a probabilistic trigger identifies the motion situation while one error-state filter runs. Sensor suite not captured — [arXiv](https://arxiv.org/abs/2607.10597)
- **Cascade IPG Observer for Underwater Robot State Estimation**, arXiv 2504.15235. A cascade of two nonlinear observers, the first quaternion-based, both using IPG gradient descent with IMU preintegration. Runs on a BlueROV2 with an ICM-20602 MEMS IMU (100 Hz), Water Linked DVL-A50 with AHRS (20 Hz), and u-blox M9N GPS (5 Hz). Field-tested in Chesapeake Bay and compared with EKF and InEKF — [arXiv](https://arxiv.org/html/2504.15235)
- **Balasubramanian et al. 2021**: dynamic- vs kinematic-model EKF on a simulated thruster-driven RexROV — [arXiv](https://arxiv.org/abs/2105.12309)
- **FeelHippo AUV** (Allotta group): a small thruster-driven AUV with FLS plus a dynamic-model navigation strategy — [ScienceDirect](https://www.sciencedirect.com/science/article/abs/pii/S0029801819308741)
- The BlueROV2 Heavy has 8 thrusters for 6-DOF motion. A Flinders University thesis (Wu, 2018) develops a thruster model and a 6-DOF dynamic model for it, a useful source of model parameters — [Flinders thesis PDF](https://flex.flinders.edu.au/file/27aa0064-9de2-441c-8a17-655405d5fc2e/1/ThesisWu2018.pdf)
- Community evidence that BlueROV2 navigation normally relies on the DVL A50 and struggles with heading drift — [Blue Robotics forum](https://discuss.bluerobotics.com/t/heading-drift-with-bluerov2-and-water-linked-dvl-a50-improve-the-compass-ekf-or-use-a-fog/23533)
- Randeni's SandShark-fleet work is on a small, low-cost vehicle, but it is a propeller-driven torpedo, not thruster-vectored — [IEEE](https://ieeexplore.ieee.org/abstract/document/8729758/)

### Inferences
- I found **no published work matching the user's exact setup**: MEMS GNSS-INS, no DVL, **no depth sensor**, an 8-thruster 6-DOF vehicle. Ji 2023 comes closest in architecture (thruster model plus surfacing recalibration with GPS). Whether Ji 2023 had a pressure sensor is unverified.
- With no depth sensor, the vertical channel is unobservable from external measurements. Every model-aided paper found keeps a pressure sensor, and the JMSE 2026 paper explicitly uses pressure-derived vertical velocity to correct attitude-induced model errors. This makes the user's case strictly harder than anything published.

### Gaps
- I could not confirm the Naik et al. 2026 BlueROV2 sensor suite or whether it uses a thruster model.
- I found no peer-reviewed BlueROV2 study using a thrust model as the only velocity aid.

## Q5. Reported results (drift, conditions)

### Takeaway
Reported model-aided drift ranges from about 1.5% of distance travelled (calibrated model, Gavia) to under 6% (high currents above 2 m/s, with water-column velocity prediction). Unaided INS or DVL water-track exceeds 30% in that same high-current setting. The 2026 deep-sea result is 509 m over 5 h outside LBL coverage. Numbers from different papers are not directly comparable because of different IMU grades and sensor suites.

### Cited Findings
- Randeni et al. (Nonlinear Dynamics 2018): the calibrated model improved velocity prediction by at least 50% over the baseline. Position was within about **1.5% of distance travelled** (Gavia AUV) — [Springer](https://link.springer.com/article/10.1007/s11071-017-3941-z)
- Model-aided localisation plus water-column velocity prediction: error **< 6% of distance travelled even in currents > 2 m/s**. Other non-bottom-tracking methods (DVL water-track, unaided INS) "could be above 30%" in those conditions — search snippet attributed to Randeni's model-aided / water-column work (exact paper not pinned; see [ResearchGate entry](https://www.researchgate.net/publication/230642972_Water_column_current_aided_localisation_for_significant_horizontal_trajectories_with_Autonomous_Underwater_Vehicles))
- JMSE 2026 attitude-compensated model-aided navigation: South China Sea, 2000 m depth, **509 m position error after 5 h beyond LBL coverage** (pressure sensor used; LBL calibration beforehand) — [MDPI](https://www.mdpi.com/2077-1312/14/7/612)
- PiDR: improvement over the baseline is reported as "more than 29%" for both the robot and AUV datasets in one listing, and "more than 55%" in another snippet. These are likely different arXiv versions. The figures are relative improvements, not absolute drift — [arXiv](https://arxiv.org/abs/2601.03040); conflicting snippet via [search on 2605.04672 page](https://arxiv.org/html/2605.04672)
- Naik et al. 2026 (BlueROV2): a paired bootstrap over 10 s segments gives a candidate-minus-baseline position-error difference of **−0.017 m** (95% CI [−0.024, −0.008] m). This is a short-horizon metric — [arXiv](https://arxiv.org/abs/2607.10597)
- Hegrenæs & Hallingstad MA-INS (HUGIN 4500, for context only; covered by the other researcher): about 6000 m travelled over the 3000 s evaluation, final position error about 0.02% DRMS of distance. This is a full-aiding/DVL-dropout scenario, not model-only — [navlab PDF](https://www.navlab.net/Publications/Model_Aided_INS_With_Sea_Current_Estimation_for_Robust_Underwater_Navigation.pdf)

### Inferences
- For a MEMS-only vehicle, the tactical-grade results (Hegrenæs, Arnold) are overly optimistic upper bounds. Randeni's 1.5–6% of distance travelled is the more relevant range, but it was achieved with a depth sensor and on a torpedo hull.

### Gaps
- I found no reported drift numbers for Ji 2023, Lv 2024, Allotta 2016 or Arnold 2018 in snippets.
- I found no results for a MEMS-only, no-depth configuration.

## Q6. Open problems stated in the literature

### Takeaway
The open problems the literature itself names are model-parameter uncertainty and changes with the environment, unknown sea or water currents, attitude-induced model errors, the mapping from propeller or thrust command to velocity, real-time feasibility, and dependence on a DVL for training or ground truth in learned approaches.

### Cited Findings
- Model-parameter error is correlated and must be modelled as filter states — Arnold & Medagoda — [arXiv](https://arxiv.org/abs/1805.08011)
- A baseline calm-water model fails in new environments and needs a calibration mission — Randeni et al. — [Springer](https://link.springer.com/article/10.1007/s11071-017-3941-z)
- Water-column current makes model-only velocity drift. Current estimation (Hegrenæs) or water-column prediction (Randeni) is needed — [2301.01114](https://arxiv.org/pdf/2301.01114); [Springer](https://link.springer.com/article/10.1007/s11071-017-3941-z)
- "Model-based velocity errors from attitude-induced deviations and uncertainties in propeller-to-velocity mapping" degrade model-aided precision — JMSE 2026 — [MDPI](https://www.mdpi.com/2077-1312/14/7/612)
- The thrust model drifts and has to be re-corrected at each surfacing — Ji et al. 2023 — [SSRN](https://papers.ssrn.com/sol3/papers.cfm?abstract_id=4251644)
- The dynamic-model EKF "needs improvements before it can be used in real time" — Balasubramanian et al. — [arXiv](https://arxiv.org/abs/2105.12309)
- External measurement loss leaves the INS solution drifting, which motivates information aiding in general — Engelsman & Klein — [arXiv](https://arxiv.org/abs/2301.01114)

### Inferences
- Applied to the user's vehicle, the top risks are: (1) no depth reference, so heave and vertical position have no external correction, and attitude-induced model error, which the 2026 paper fixes with pressure, cannot be corrected the same way; (2) T200 thrust curves vary with voltage and advance ratio, which is the same propeller-to-velocity mapping problem; (3) MEMS heading drift underwater, where the magnetometer may be disturbed by the thrusters. Surfacing recalibration of the model, as in Ji 2023, is the documented mitigation that fits the user's setup.

### Gaps
- The full-text "future work" sections of the 2023 and 2026 reviews could not be read.
