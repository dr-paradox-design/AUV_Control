# Hegrenæs / Hallingstad / Berglund: Model-Aided INS (MA-INS) for AUVs

> **Access caveat (read first).** I could not open any full text this session. The egress proxy blocked IEEE Xplore, navlab.net (FFI's NavLab site, which hosts free PDFs of all the core papers), mic-journal.no, ResearchGate, Semantic Scholar, Crossref, DFKI, NTNU Open and LiU. Only WebSearch result snippets and search-engine summaries came through. **Everything below comes from abstracts, repository metadata or search snippets. None of it comes from reading the papers.** The equations, state-vector lists and process-model details the user asked about **could not be verified**. Where I add something from background memory, it is labelled **[UNVERIFIED RECOLLECTION]** and must be checked against the PDFs. Free PDF URLs to read next are listed under each Gaps section.

## 1. Bibliographic details and the citation discrepancy

### Takeaway
The user's citation "Hegrenæs, Berglund & Hallingstad, JOE 2008/2009" conflates several papers. No search turned up a JOE paper by those three authors. The three-author paper is a **conference paper at ICRA 2008**. The JOE paper is **2011 and has two authors, Hegrenæs and Hallingstad**. Berglund's other co-authored paper is the **OCEANS'09 Europe paper on DVL water-track aiding**, which uses no vehicle model.

### Cited Findings
- **Core journal paper:** Ø. Hegrenæs and O. Hallingstad, "Model-Aided INS With Sea Current Estimation for Robust Underwater Navigation," *IEEE Journal of Oceanic Engineering*, vol. 36, no. 2, pp. 316–337, May 2011 — [WebSearch summary of ResearchGate/navlab listings](https://www.researchgate.net/publication/224238828_Model-Aided_INS_With_Sea_Current_Estimation_for_Robust_Underwater_Navigation). The DOI **10.1109/JOE.2010.2100470** comes only from a search-engine summary. I could not open IEEE Xplore to confirm it, so check it before citing. Free PDF: [navlab.net](https://www.navlab.net/Publications/Model_Aided_INS_With_Sea_Current_Estimation_for_Robust_Underwater_Navigation.pdf)
- **ICRA paper (the real "Hegrenæs, Berglund, Hallingstad"):** Ø. Hegrenæs, E. Berglund, O. Hallingstad, "Model-aided inertial navigation for underwater vehicles," *Proc. IEEE ICRA*, Pasadena, CA, May 2008, pp. 1069–1076 — [search summary; ResearchGate listing](https://www.researchgate.net/publication/224318397_Model-Aided_Inertial_Navigation_for_Underwater_Vehicles). Free PDF: [navlab.net](https://www.navlab.net/Publications/Model-Aided_Inertial_Navigation_for_Underwater_Vehicles.pdf)
- **Earliest MA-INS paper (journal, open access):** Ø. Hegrenæs, O. Hallingstad, K. Gade, "Towards Model-Aided Navigation of Underwater Vehicles," *Modeling, Identification and Control*, vol. 28, no. 4, pp. 113–123, 2007, DOI 10.4173/mic.2007.4.3. Affiliations: UNIK, FFI, NTNU — [MIC abstract page](https://www.mic-journal.no/ABS/MIC-2007-4-3.asp/); [navlab PDF](https://www.navlab.net/Publications/Towards_Model_Aided_Navigation_of_Underwater_Vehicles.pdf). It describes itself as the first report on implementing and experimentally evaluating model-aided INS for underwater vehicles.
- **DVL water-track paper (Berglund co-author, no vehicle model):** Ø. Hegrenæs and E. Berglund, "Doppler water-track aided inertial navigation for autonomous underwater vehicle," *OCEANS 2009-EUROPE*, May 2009 — [Semantic Scholar listing](https://www.semanticscholar.org/paper/Doppler-water-track-aided-inertial-navigation-for-Hegrenaes-Berglund/968e994cd85e524bb8a87a917b0c2021fcaa0a9d); [ResearchGate](https://www.researchgate.net/publication/224599991_Doppler_water-track_aided_inertial_navigation_for_autonomous_underwater_vehicle)
- **Model-identification precursor:** Ø. Hegrenæs, O. Hallingstad, B. Jalving, "Comparison of Mathematical Models for the HUGIN 4500 AUV Based on Experimental Data," *Proc. IEEE Int. Symp. Underwater Technology (UT'07)*, Tokyo, 2007. It compares several HUGIN 4500 models driven by **measured actuator inputs**, fitted to field data by least squares — [search summary / ResearchGate profile](https://www.researchgate.net/profile/Oyvind-Hegrenaes)
- **PhD thesis:** Ø. Hegrenæs, *Autonomous Navigation for Underwater Vehicles*, PhD dissertation, NTNU, 2010 — [NTNU Open handle 11250/260288](https://ntnuopen.ntnu.no/ntnu-xmlui/handle/11250/260288?show=full&locale-attribute=en). The thesis has two main topics: (i) using a kinetic vehicle model to provide velocity aiding to the INS; (ii) a DVL water-track aided INS, described as "the first in-depth discussion, derivation, and experimental evaluation" of that technology. In both, sea current is estimated in real time (from the NTNU Open abstract, via search snippet).
- A search result attributed "Inertial Navigation — Theory and Applications" (NTNU, Jan 2018) to Hegrenæs — [ResearchGate](https://www.researchgate.net/publication/323657391_Inertial_Navigation_-_Theory_and_Applications). **This attribution looks wrong.** That title matches Kenneth Gade's 2018 NTNU thesis. Hegrenæs's thesis is the 2010 one above. Treat the search summary as an error.

### Inferences
- Suggested citation mapping: the user's "JOE" → Hegrenæs & Hallingstad 2011 (JOE 36(2)); "Berglund 2008" → the ICRA 2008 conference paper; "2009" → most likely the OCEANS'09 water-track paper, which is a different technique (DVL water-track, not a vehicle model).
- The ICRA 2008 abstract wording in the search summary ("state-of-the-art MA-INS … with real-time sea current estimation") is nearly the same as the JOE 2011 abstract. JOE 2011 looks like the extended journal version of ICRA 2008.

### Gaps
- DOI not confirmed on IEEE Xplore (blocked).
- I could not confirm the venue city or page numbers of the OCEANS'09 paper. It is probably Bremen, but I could not verify that.

## 2. Filter architecture and how the vehicle model enters

### Takeaway
Abstract-level sources agree on the following. The kinetic vehicle model runs as a **separate model whose output is a model-based velocity**. That velocity is fed into the INS's Kalman filter as an **aiding measurement**, the same way a DVL velocity would be. Sea current is a **state in the Kalman filter**. I could not verify the exact error-state vector or the measurement equations.

### Cited Findings
- JOE 2011 abstract (via search summary): "the output from an experimentally validated kinetic vehicle model is integrated in the navigation system to provide velocity aiding for the INS", "together with real-time sea current estimation" — [ResearchGate abstract / search summary](https://www.researchgate.net/publication/224238828_Model-Aided_INS_With_Sea_Current_Estimation_for_Robust_Underwater_Navigation)
- MIC 2007 / thesis description (search summary): an "experimentally validated kinetic vehicle model for providing model-based velocity measurements", used for aiding the INS — [ResearchGate "Towards Model-Aided Navigation"](https://www.researchgate.net/publication/41719915_Towards_Model-Aided_Navigation_of_Underwater_Vehicles)
- A secondary source summarised in search results: "the current is included as a state in the Kalman filter" — [search summary of citing literature](https://www.researchgate.net/publication/224238828_Model-Aided_INS_With_Sea_Current_Estimation_for_Robust_Underwater_Navigation) (secondary, low confidence on wording)
- "Accurate knowledge of the vehicle dynamics is utilized for aiding the INS" — [MIC 2007 abstract](https://www.mic-journal.no/ABS/MIC-2007-4-3.asp/)

### Inferences
- **[UNVERIFIED RECOLLECTION]** The FFI/Kongsberg HUGIN navigation system (NavLab) uses an **error-state (indirect), feedback** Kalman filter. Its error states include position, velocity, attitude, and accelerometer and gyro biases. MA-INS added water-current states to this filter. I believe the vehicle model is integrated **outside** the KF. Its velocity relative to the water, rotated with the INS attitude and combined with the estimated current, is differenced against the INS velocity to form the measurement. If that is right, it is a "model as a pseudo-sensor" architecture, not "model inside the process model". Verify against JOE 2011 Sections II–IV.
- For the user's design, this means MA-INS can be added as a **measurement update** to an existing INS error-state filter (VN-200 output or a custom EKF), without replacing the INS mechanisation.

### Gaps
- I did not see the exact error-state vector, its dimension, the process and measurement equations, or the noise values. The full text was blocked.
- I could not confirm whether the model's own states (e.g. a model-propagated velocity) are augmented into the filter or reset by the filter.

## 3. The vehicle (kinetic) model: DOF, hydrodynamic terms, thrust, inputs

### Takeaway
The model is a **kinetic model of HUGIN 4500**, driven by **measured actuator inputs**. Its parameters came from semi-empirical relations, open-water tests and navigation data. I could not read which terms it contains, its DOF, or the form of the propeller model.

### Cited Findings
- Model parameters for the HUGIN 4500 were found "from semi-empirical relationships, open-water test, and from navigation data collected by the HUGIN 4500" — [search snippet, MIC 2007 paper](https://www.navlab.net/Publications/Towards_Model_Aided_Navigation_of_Underwater_Vehicles.pdf)
- UT'07 paper: several mathematical models compared, describing HUGIN 4500 response "as a function of measured actuator inputs", fitted by dedicated experiments plus least squares — [search summary](https://www.researchgate.net/profile/Oyvind-Hegrenaes)
- Related identification work by the same group: "A framework for obtaining steady-state maneuvering characteristics of underwater vehicles using sea-trial data" — [ResearchGate](https://www.researchgate.net/publication/224303034_A_framework_for_obtaining_steady-state_maneuvering_characteristics_of_underwater_vehicles_using_sea-trial_data). Also Fauske, Gustafsson & Hegrenæs, "Estimation of AUV dynamics for sensor fusion" (FUSION 2007) — [LiU report](https://people.isy.liu.se/rt/fredrik/reports/07FusionAUV.pdf). I did not read either.

### Inferences
- **[UNVERIFIED RECOLLECTION]** The model follows Fossen's structure: M ν̇_r + C(ν_r)ν_r + D(ν_r)ν_r + g(η) = τ, written in **relative (water-referenced) velocity ν_r**. Inputs are **propeller rpm** and **control-fin (rudder/elevator) deflections**. Thrust comes from a propeller rpm model, and current enters through ν_r = ν − ν_c. Confirm the DOF (I believe the full 6-DOF model, possibly decoupled) and the exact thrust law in JOE 2011 and UT'07.
- **Relevance to the manta AUV:** HUGIN is a torpedo-shaped vehicle with one propeller and fins, cruising fast (forward speed around 1.6–2 m/s, see Section 6). Fin lift and forward-speed damping make its surge and sway well conditioned. A slow, 8-thruster vehicle with no control surfaces has to rely entirely on the T200 thrust-curve model (rpm or PWM → force, battery-voltage dependent) and on the damping terms. These are usually less certain at low speed.

### Gaps
- I could not get the exact list of hydrodynamic coefficients, the propeller thrust equation (e.g. a quadratic in rpm, or an advance-ratio dependence), or the model DOF.

## 4. Sea current model and estimation

### Takeaway
Sea current is estimated in real time as Kalman-filter states. This holds for both MA-INS and the DVL water-track INS. I could not read the stochastic model (random walk or Gauss–Markov), the frame (NED) or whether a vertical component is included.

### Cited Findings
- "Together with real-time sea current estimation…" — [JOE 2011 abstract via search](https://www.researchgate.net/publication/224238828_Model-Aided_INS_With_Sea_Current_Estimation_for_Robust_Underwater_Navigation)
- Thesis: "As with model aiding, sea current estimation is done in real-time" for the water-track INS — [NTNU Open abstract via search](https://ntnuopen.ntnu.no/ntnu-xmlui/handle/11250/260288?show=full&locale-attribute=en)
- Current included as a state in the KF — [secondary, search summary](https://www.researchgate.net/publication/224238828_Model-Aided_INS_With_Sea_Current_Estimation_for_Robust_Underwater_Navigation)
- **Observability (one line, as instructed):** JOE 2011 contains an observability analysis of the current states. I could not read its conclusion. **[UNVERIFIED RECOLLECTION]** It concludes that current is observable when there is position aiding (USBL/GPS) or DVL bottom-track, and that without such aiding, current errors and model velocity errors cannot be told apart. Verify.

### Inferences
- **[UNVERIFIED RECOLLECTION]** I believe the current was modelled in the local-level/NED frame as a slowly varying 1st-order Gauss–Markov process, with horizontal components only. Verify.
- **Key implication for the user:** a model gives velocity **relative to water**. Without position fixes, the current is unobservable, so all current error turns directly into position drift. The user has GPS only at the surface. The current can be estimated during surfaced or GPS-aided segments, but it is then frozen or random-walked while submerged.

### Gaps
- Process model, noise spectral densities and frame are all unverified.

## 5. Handling model errors / uncertain hydrodynamic coefficients

### Takeaway
Abstracts give no information on how model uncertainty is handled. The papers stress that the model is "experimentally validated" and calibrated from sea-trial data before use. That suggests uncertainty was handled mainly by **offline identification plus measurement-noise tuning**, not online parameter estimation, but this is an inference.

### Cited Findings
- Model calibrated from semi-empirical relations, open-water tests and HUGIN navigation data — [MIC 2007 snippet](https://www.navlab.net/Publications/Towards_Model_Aided_Navigation_of_Underwater_Vehicles.pdf)
- Least-squares fitting to field data — [UT'07 summary](https://www.researchgate.net/profile/Oyvind-Hegrenaes)

### Inferences
- I found no evidence (in the material I could access) of online hydrodynamic-parameter states in the Hegrenæs MA-INS filter. Later work by other groups (e.g. the "Robust Model-Aided Inertial Localization" paper, arXiv 1805.08011, which I did not read) covers robustness to model error.

### Gaps
- Model-velocity measurement covariance values, any parameter-state augmentation, and whether errors are modelled as coloured noise are all unverified.

## 6. Results: vehicle, data, quantitative drift numbers

### Takeaway
All results are from **real field data from the HUGIN 4500 AUV**, post-processed and not run in real time on the vehicle per the abstracts. The one hard number I found, from the 2007 MIC paper, is for a segment with no position aiding: **the INS with no velocity aiding reached a maximum horizontal position-error norm close to 700 m, while MA-INS stayed at about 6 m maximum.** I could not access the JOE 2011 numbers for DVL removal or dropout.

### Cited Findings
- MIC 2007 (search snippet, close to verbatim): "For the part without position aiding, the traditional INS breaks down quickly with a maximum Euclidian norm of the position error close to 700 meters, while the model-aided INS continues to perform excellently with a maximum norm of the position error of only 6 meters." — [MIC 2007 PDF snippet](https://www.navlab.net/Publications/Towards_Model_Aided_Navigation_of_Underwater_Vehicles.pdf). **Conditions I could not verify:** segment length and duration, whether DVL or pressure aiding was still present, and what "traditional INS" was aided by. The large gap suggests that "traditional INS" here means an INS with no velocity aiding at all.
- JOE 2011: performance is evaluated on data from a field-deployed AUV. The scenarios include "removal or dropouts of USBL and DVL". The conclusion is that "with merely an addition of software and no added instrumentation, it is possible to significantly improve the precision and robustness of an INS". It claims the "first experimental evaluation and practical application of a MA-INS" — [abstract via search](https://www.researchgate.net/publication/224238828_Model-Aided_INS_With_Sea_Current_Estimation_for_Robust_Underwater_Navigation)
- A search snippet gave "more than 4% of the distance travelled (assuming a forward speed of 1.6 m/s)" for an unaided or unconstrained INS, and "around 1.5% of the distance travelled" for a calibrated model. **Attribution is unclear.** The snippet came with a group of results and probably belongs to a later low-cost-AUV paper (e.g. arXiv 1805.08011 or the "Hydrodynamic Model-Based Navigation System for a Low-Cost AUV Fleet" paper), not to Hegrenæs. **Do not cite these as Hegrenæs numbers.** — [search result set incl. arXiv 1805.08011](https://arxiv.org/pdf/1805.08011)

### Inferences
- **[UNVERIFIED RECOLLECTION]** JOE 2011 reports MA-INS drift without DVL on the order of a few tenths of a percent to about 1% of distance travelled over multi-hour HUGIN segments. This is from memory and not citable. Read Section VI of the JOE PDF for the exact figures.

### Gaps
- JOE 2011 and ICRA 2008 quantitative tables (drift in m and % of distance travelled, mission length and duration) were not accessible.

## 7. Assumptions, limitations, and other aiding sensors (depth sensor?)

### Takeaway
**Yes, the Hegrenæs MA-INS relied on additional aiding.** JOE 2011 explicitly lists **pressure (depth) readings**, **USBL acoustic positioning** and **DVL bottom-track** as aiding sources alongside the model. The scenarios removed DVL and/or USBL, but **the pressure sensor stayed in**. Their setup also used a high-grade IMU on a large torpedo AUV with fins. The user's setup has none of these: no depth sensor, a MEMS IMU, and a thruster-only manta. This is a major difference.

### Cited Findings
- JOE 2011 abstract: model velocity aiding plus "additional aiding sources including ultrashort base line (USBL) acoustic positioning, pressure readings, and measurements from a Doppler velocity log (DVL) with bottom track" — [abstract via search](https://www.researchgate.net/publication/224238828_Model-Aided_INS_With_Sea_Current_Estimation_for_Robust_Underwater_Navigation)
- Thesis goal: navigate "for considerable time without operator supervision … possibly with sparse external positioning and subject to velocity sensor failures or dropouts". This frames the model as a **backup for DVL dropouts**, not a sole sensor — [NTNU Open](https://ntnuopen.ntnu.no/ntnu-xmlui/handle/11250/260288?show=full&locale-attribute=en)
- The water-track paper also evaluated "both horizontal and vertical navigation performance" — [NTNU Open abstract via search](https://ntnuopen.ntnu.no/ntnu-xmlui/handle/11250/260288?show=full&locale-attribute=en)

### Inferences
- **For a VN-200-only vehicle:** without a pressure sensor, vertical position and vertical velocity are constrained only by the model's heave prediction. With a MEMS IMU, the vertical channel will diverge faster than anything shown in the Hegrenæs results. A cheap Bar30/Bar02 depth sensor would close the largest gap between the user's setup and the conditions in these papers. **[UNVERIFIED RECOLLECTION]** HUGIN's IMU was a navigation-grade ring-laser-gyro unit (Honeywell HG9900 class). MEMS gyros on the VN-200 will have much larger heading drift underwater, where there is no GNSS and the magnetometer may be disturbed by thruster currents. Model aiding constrains velocity, but heading error still turns into cross-track error.
- The MA-INS advantage depends on forward speed and on the model being well conditioned. A torpedo AUV at cruise speed is the favourable case.

### Gaps
- I did not read the stated limitations or future-work sections.

## 8. Follow-on and related papers by the same authors

### Takeaway
The lineage runs: UT'07 model comparison → MIC 2007 "Towards MA-INS" → ICRA 2008 MA-INS → OCEANS'09 DVL water-track INS → 2010 NTNU PhD → JOE 2011 MA-INS with current estimation. Related work came from FFI/LiU on AUV dynamics estimation.

### Cited Findings
- UT'07 HUGIN 4500 model comparison; MIC 2007; ICRA 2008; OCEANS'09-Europe; PhD 2010; JOE 2011 — sources as in Section 1.
- Fauske, Gustafsson, Hegrenæs, "Estimation of AUV dynamics for sensor fusion" (FUSION 2007) — [LiU report](https://people.isy.liu.se/rt/fredrik/reports/07FusionAUV.pdf); [ResearchGate](https://www.researchgate.net/profile/Oyvind-Hegrenaes/publication/4301323_Estimation_of_AUV_dynamics_for_sensor_fusion/links/5934647645851553b6e6a817/Estimation-of-AUV-dynamics-for-sensor-fusion.pdf?origin=scientificContributions)
- "A framework for obtaining steady-state maneuvering characteristics of underwater vehicles using sea-trial data" (Hegrenæs et al.) — [ResearchGate](https://www.researchgate.net/publication/224303034_A_framework_for_obtaining_steady-state_maneuvering_characteristics_of_underwater_vehicles_using_sea-trial_data)
- Related FFI/HUGIN aiding overview: "A Toolbox of Aiding Techniques for the HUGIN AUV Integrated Inertial Navigation System" — [navlab.net](https://www.navlab.net/Publications/A_Toolbox_of_Aiding_Techniques_for_the_HUGIN_AUV_Integrated_Inertial_Navigation_System.pdf)
- Hegrenæs later co-authored "Validation of a new generation DVL for underwater vehicle navigation" (Kongsberg/Nortek). This continues the work on DVL aiding, not on the vehicle model — [Nortek PDF](https://www.nortekgroup.com/assets/documents/Validation-of-a-New-Generation-DVL-for-Underwater-Vehicle-Navigation.pdf)

### Gaps
- I did not establish whether MA-INS became an operational HUGIN/NavLab feature after 2011.
- **What to read next (all free, but blocked here):** the navlab.net PDFs for JOE 2011, ICRA 2008 and MIC 2007; the NTNU Open thesis PDF. For the user's needs, JOE 2011 Sections I, the model section, the filter/measurement section and the results section answer everything left open above.
