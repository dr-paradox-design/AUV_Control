# Error-state (indirect) Kalman filter for quaternion strapdown INS: Solà (arXiv 1711.02508) and Fossen (Handbook, 2011 / 2021), with a notation map for a Fossen-notation AUV simulator

**How these notes were sourced, and their limits**
- The network policy blocked arxiv.org, iri.upc.edu, alphaxiv, semanticscholar, wiley.com, fossen.biz, scispace, e-bookshelf and Dropbox, so I could not open the arXiv abstract page or Fossen's book/slides directly.
- **Solà:** I read the full PDF from a public GitHub mirror ([TurtleZhong/msckf_mono copy](https://github.com/TurtleZhong/msckf_mono/blob/master/Quaternion%20kinematics%20for%20the%20error-state%20Kalman%20filter.pdf)). Its title page reads "Quaternion kinematics for the error-state Kalman filter, Joan Solà, October 12, 2017". The copy has someone's handwritten Chinese annotations, which I ignored. All equation numbers below come from that copy.
- **Lost signs in the extracted text:** pdftotext dropped minus signs and δ symbols. I restored the signs by re-checking each step of Solà's own derivation, e.g. (243)→(246), (255)→(258) and (290)→(293). Restored signs are written normally; anything I could not re-derive is flagged.
- **Fossen:** I could not read the book text. For Fossen I report only:
  - the table of contents as surfaced in search-result snippets;
  - the official FossenHandbook GitHub README;
  - the header comments and maths structure of the MSS toolbox INS functions, read from a clone of [cybergalactic/MSS](https://github.com/cybergalactic/MSS) at commit ff0ee4c, 2026-10-03.
- I describe the MSS functions only as pointers to what they implement; no code is copied. Anything from memory is marked **[unverified]**.

---

## Q1. Solà: arXiv id, version history, section structure

### Takeaway
"Quaternion kinematics for the error-state Kalman filter" by Joan Solà is arXiv:1711.02508 (cs.RO). Search metadata says it was submitted 3 Nov 2017, and the PDF I read is dated 12 Oct 2017. The ESKF material is in §5 (error-state kinematics), §6 (correction/injection/reset), §7 (global-angular-error variant) and Appendices B–E (integration and noise).

### Cited Findings
- **Identifier and date:** arXiv identifier 1711.02508, submitted 3 November 2017 (search-engine summary of the arXiv listing; I could not open the page itself) — [arXiv abs](https://arxiv.org/abs/1711.02508).
- **Abstract wording:** the abstract describes "an exhaustive revision of concepts and formulas related to quaternions and rotations in 3D space, and their proper use in estimation engines such as the error-state Kalman filter … precise formulations for error-state Kalman filters suited for real applications using integration of signals from an inertial measurement unit (IMU)" — [arXiv abs, via search snippet](https://arxiv.org/abs/1711.02508).
- **Earlier version:** an earlier version circulated as the IRI technical report "IRI-TR-16-02, Quaternion kinematics for the error-state KF", dated September 12, 2016. It was also mirrored for ETH's Robot Dynamics 2016 course — [IRI TR (search listing)](http://www.iri.upc.edu/files/scidoc/1773-Quaternion-kinematics-for-the-error-state-Kalman-filter.pdf); [ETH mirror (search listing)](https://ethz.ch/content/dam/ethz/special-interest/mavt/robotics-n-intelligent-systems/rsl-dam/documents/RobotDynamics2016/QuaternionKinematicsSolaForETH.pdf). Solà's homepage also hosts a live copy, `kinematics.pdf` — [IRI personal page (search listing)](http://www.iri.upc.edu/people/jsola/JoanSola/objectes/notes/kinematics.pdf).
- **Section structure** (from the TOC of the 12 Oct 2017 PDF) — [PDF copy](https://github.com/TurtleZhong/msckf_mono/blob/master/Quaternion%20kinematics%20for%20the%20error-state%20Kalman%20filter.pdf):
  - **1 Quaternion definition and properties.**
  - **2 Rotations and cross-relations:** SO(3), exp/log maps, Rodrigues, rotation matrix ↔ quaternion, composition, SLERP, isoclinic rotations.
  - **3 Quaternion conventions. My choice:** §3.1 Quaternion flavors, §3.1.1–3.1.4.
  - **4 Perturbations, derivatives and integrals:**
    - §4.1 ⊕/⊖ operators;
    - §4.2 four derivative definitions;
    - §4.3 Jacobians, including the right Jacobian of SO(3);
    - §4.4 Perturbations: §4.4.1 Local, §4.4.2 Global;
    - §4.5 Time derivatives;
    - §4.6 Time-integration of rotation rates (zeroth- and first-order).
  - **5 Error-state kinematics for IMU-driven systems** (p.50):
    - §5.1 Motivation; §5.2 The ESKF explained;
    - §5.3 Continuous time: §5.3.1 true, §5.3.2 nominal, §5.3.3 error;
    - §5.4 Discrete time: §5.4.1 nominal, §5.4.2 error, §5.4.3 Jacobian and perturbation matrices.
  - **6 Fusing IMU with complementary sensory data** (p.60):
    - §6.1 Observation via filter correction; §6.1.1 Jacobian computation;
    - §6.2 Injection;
    - §6.3 ESKF reset; §6.3.1 Jacobian of reset.
  - **7 The ESKF using global angular errors** (p.64): §7.1–7.3, mirroring §5–6.
  - **Appendices:**
    - App. A Runge-Kutta;
    - App. B Closed-form integration (B.1 angular error, B.2 simplified IMU, B.3 full IMU);
    - App. C Truncated series (C.1 system-wise, C.2 block-wise);
    - App. D Transition matrix via RK;
    - App. E Integration of random noise and perturbations (E.1 impulses, E.2 Full IMU example, E.2.1).

### Inferences
- §5–7 equation numbering runs (229)–(322) and App. E runs (424)–(458) in this copy. If the user works from a later arXiv/IRI revision, equation numbers may shift. Match by section title.

### Gaps
- **Version history unverified:** I could not open the arXiv abstract page (blocked), so the full list (v1 only, or v1/v2…, with dates) is unverified.
- **Unknown changes in the living copy:** Solà is known to keep updating `kinematics.pdf` on his site **[unverified]**. Any post-2017 corrections (e.g. to the noise units in (261)–(264)) could not be checked.

---

## Q2. Solà: quaternion convention (Hamilton vs JPL; local vs global perturbation; pitfalls)

### Takeaway
Solà uses the **Hamilton** quaternion:
- scalar first, q = [q_w, q_v];
- ij = k, so right-handed;
- passive;
- q = q_GL, so x_G = q⊗x_L⊗q*.

Perturbations default to **local** (right-multiplied): q_t = q⊗δq. JPL differs on order, algebra/handedness and direction (q_LG). Solà warns that the formulas are not interchangeable even when the numerical quaternion values coincide.

### Cited Findings
All findings in this section are from the [PDF copy](https://github.com/TurtleZhong/msckf_mono/blob/master/Quaternion%20kinematics%20for%20the%20error-state%20Kalman%20filter.pdf).
- **Four binary choices** fix a quaternion convention (§3.1, eqs. 140–143):
  - component order: real first or last;
  - algebra: ij = −ji = k vs ji = −ij = k, i.e. right- vs left-handed;
  - function: passive vs active;
  - direction: local-to-global vs global-to-local.

  These give 12 combinations. "The formulas are thus not compatible, and we need to make a clear choice from the very start."
- **Table 2, Hamilton vs JPL:**

  | Choice | Hamilton | JPL |
  |---|---|---|
  | Order | (q_w, q_v) | (q_v, q_w) |
  | Algebra | ij = k | ij = −k |
  | Handedness | Right | Left |
  | Function | Passive | Passive |
  | Right-to-left products mean | Local-to-Global | Global-to-Local |
  | Default q | q_GL | q_LG |
  | Default operation | x_G = q⊗x_L⊗q* | x_L = q⊗x_G⊗q* |
- **Solà's choice:** "My choice … is to take the Hamilton convention, which is right-handed and coincides with … Eigen, ROS, Google Ceres". JPL is used by Trawny & Roumeliotis (2005) and Li & Mourikis (§3.1).
- **Pitfalls Solà names:**
  - Changing component order requires swapping rows/columns of every 4×4 or 4×3 quaternion matrix, which is "prone to error" (§3.1.1).
  - Left- and right-handed quaternions satisfy q_left = q*_right (eq. 146).
  - q_JPL ≜ q_LG,left = q*_LG,right = q_GL,right ≜ q_Hamilton (eq. 156). The values are equal but the quaternions "mean and represent different things", "the source of great confusion" (§3.1.4).
  - Eigen stores the real part last but is still Hamilton (§3.1.1).
- **Active vs passive and DCM:** q_active = q*_passive and R_active = Rᵀ_passive. The direction cosine matrix C ≡ R_passive (eqs. 147–150).
- **Local perturbation (§4.4.1):** q̃ = q⊗δq_L, R̃ = R·δR_L (eq. 189), with δq_L = Exp(δθ_L) (eq. 190). Small-angle forms: δq_L ≈ [1, ½δθ_L], δR_L ≈ I + [δθ_L]× (eq. 192).
- **Global perturbation (§4.4.2):** q̃ = Exp(δθ_G)⊗q, R̃ = Exp(δθ_G)·R (eq. 193). The global perturbation lies in the tangent space at the origin.
- **ESKF defaults (§5.3):**
  - angular rates ω are local, so gyro output ω_m is used directly;
  - the angular error δθ is local, "the classical approach";
  - Solà adds: "There exists evidence (Li and Mourikis, 2012) that a globally-defined angular error has better properties", which is treated in §7.

### Inferences
- **Fossen/MSS uses the same convention as Solà** (see Q7 for sources):
  - `Rquat(q)` builds R = I + 2ηS(ε) + 2S(ε)², scalar-first q = [η, ε];
  - `quatprod` is the Hamilton product;
  - `Rquat` maps BODY→NED, i.e. R^n_b = Solà's R_GL with G = NED and L = BODY.

  So Solà's Hamilton formulas apply directly with q ↔ [η; ε₁; ε₂; ε₃]. A JPL-based MSCKF/VIO code would need conversion.

### Gaps
- None material for this question.

---

## Q3. Solà: nominal/error states, continuous-time error dynamics, discrete F_x, F_i, Q_i (local-angular-error ESKF)

### Takeaway
- **State sizes:** true and nominal states are 19-D (p, v, q, a_b, ω_b, g); the error state is 18-D (δp, δv, δθ, δa_b, δω_b, δg).
- **Error dynamics:** (237) is linear in δx, with Jacobians built from the nominal state.
- **Discrete form:** the Euler transition matrix F_x is (269). F_i maps four impulse vectors; Q_i = diag(V_i, Θ_i, A_i, Ω_i) is given by (261)–(264).
- **Prediction:** the error mean stays zero and only P is propagated.

### Cited Findings
All findings in this section are from the [PDF copy](https://github.com/TurtleZhong/msckf_mono/blob/master/Quaternion%20kinematics%20for%20the%20error-state%20Kalman%20filter.pdf).

**Composition (Table 3):**
- p_t = p + δp;
- v_t = v + δv;
- q_t = q⊗δq with δq = e^{δθ/2};
- R_t = R·δR with δR = e^{[δθ]×};
- a_bt = a_b + δa_b;
- ω_bt = ω_b + δω_b;
- g_t = g + δg.
- Bias random-walk noises are a_w and ω_w; measurement noises are a_n and ω_n.

**IMU model:**
- a_m = R_tᵀ(a_t − g_t) + a_bt + a_n (230); ω_m = ω_t + ω_bt + ω_n (231).
- Inverting: a_t = R_t(a_m − a_bt − a_n) + g_t (232); ω_t = ω_m − ω_bt − ω_n (233).
- Footnote 23: Earth rate is neglected, otherwise ω_m = ω_t + R_tᵀω_E + ω_bt + ω_n. ω_E ≈ 15°/h ≈ 7.3·10⁻⁵ rad/s and "should not be neglected" with high-end IMUs.

**True kinematics (234a–f):**
- ṗ_t = v_t;
- v̇_t = R_t(a_m − a_bt − a_n) + g_t;
- q̇_t = ½ q_t⊗(ω_m − ω_bt − ω_n);
- ȧ_bt = a_w; ω̇_bt = ω_w; ġ_t = 0.
- State and noise vectors (235): x_t = [p v q a_b ω_b g], u = [a_m − a_n; ω_m − ω_n], w = [a_w; ω_w].

**Treatment of gravity:**
- Solà estimates g, expressed in the initial frame q₀ = (1,0,0,0), to improve linearity.
- He notes "the reader is free to remove all equations related to gravity … and adopt a more classical approach of considering g ≜ (0,0,−9.8xx) … and an uncertain initial orientation q₀" (§5.3.1). The sign of the z-component was lost in extraction; with his a_m = Rᵀ(a − g) model, g is the gravity acceleration vector.

**Nominal kinematics (236a–f):**
- ṗ = v;
- v̇ = R(a_m − a_b) + g;
- q̇ = ½ q⊗(ω_m − ω_b);
- ȧ_b = 0; ω̇_b = 0; ġ = 0.

**Error-state kinematics, local δθ (237a–f):**
- δṗ = δv
- δv̇ = −R[a_m − a_b]× δθ − R δa_b + δg − R a_n
- δθ̇ = −[ω_m − ω_b]× δθ − δω_b − ω_n
- δȧ_b = a_w; δω̇_b = ω_w; δġ = 0.

**How (237b) and (237c) are derived:**
- **Velocity error:** uses R_t = R(I + [δθ]×) + O(‖δθ‖²) (238). It assumes isotropic white accel noise, E[a_n a_nᵀ] = σ_a² I (247), which allows the redefinition a_n ← R a_n "with absolutely no consequences" (248). The result is δv̇ = −R[a_m − a_b]×δθ − Rδa_b + δg − a_n (249).
- **Orientation error:** (250)–(258), giving δθ̇ = −[ω]×δθ + δω (257) with ω = ω_m − ω_b (252) and δω = −δω_b − ω_n (253).
- Footnote 24: isotropy "cannot be made in cases where the three XYZ accelerometers are not identical".

**Discrete nominal update (259):**
- p ← p + vΔt + ½(R(a_m − a_b) + g)Δt²
- v ← v + (R(a_m − a_b) + g)Δt
- q ← q⊗q{(ω_m − ω_b)Δt}
- the biases and g are held constant.
- Solà notes "more precise integration" is available in the appendices.

**Discrete error update (260):**
- δp ← δp + δvΔt
- δv ← δv + (−R[a_m − a_b]×δθ − Rδa_b + δg)Δt + v_i
- δθ ← Rᵀ{(ω_m − ω_b)Δt}δθ − δω_bΔt + θ_i
- δa_b ← δa_b + a_i; δω_b ← δω_b + ω_i; δg ← δg.

**Impulse covariances (261–264):**
- V_i = σ²_ãn Δt² I [m²/s²]
- Θ_i = σ²_ω̃n Δt² I [rad²]
- A_i = σ²_aw Δt I [m²/s⁴]
- Ω_i = σ²_ωw Δt I [rad²/s²]
- Units as printed: σ_ãn [m/s²], σ_ω̃n [rad/s], σ_aw [m/s²√s], σ_ωw [rad/s√s]. These are "to be determined from the information in the IMU datasheet, or from experimental measurements".

**Compact form (265–268):**
- x = [p v q a_b ω_b g], δx = [δp δv δθ δa_b δω_b δg], u_m = [a_m; ω_m], i = [v_i θ_i a_i ω_i].
- δx ← f(x, δx, u_m, i) = F_x(x, u_m)·δx + F_i·i (266)
- Prediction: δx̂ ← F_x δx̂ (267); P ← F_x P F_xᵀ + F_i Q_i F_iᵀ (268).

**Transition matrix (269), Euler form.** Block rows/cols are [δp δv δθ δa_b δω_b δg]:
```
F_x = | I  IΔt  0                          0      0     0   |
      | 0  I    −R[a_m−a_b]×Δt             −RΔt   0     IΔt |
      | 0  0    Rᵀ{(ω_m−ω_b)Δt}            0     −IΔt   0   |
      | 0  0    0                          I      0     0   |
      | 0  0    0                          0      I     0   |
      | 0  0    0                          0      0     I   |
```

**Noise matrices (270):**
```
F_i = | 0 0 0 0 |      Q_i = diag(V_i, Θ_i, A_i, Ω_i)
      | I 0 0 0 |
      | 0 I 0 0 |
      | 0 0 I 0 |
      | 0 0 0 I |
      | 0 0 0 0 |
```

**Solà's implementation notes (§5.4.3):**
- F_x "can be approximated to different levels of precision"; (269) is "one of its simplest forms (the Euler form)".
- Line (267) "always returns zero. You should of course skip line (267) in your code … but … comment it out".
- "you should NOT skip the covariance prediction (268)!!"

**Noise integration theory (App. E):**
- Continuous model δẋ = Aδx + Bũ + Cw (427), where ũ is sampled control (IMU) noise and w is the unsampled perturbation (bias random walk).
- Discretised: F_x = Φ = e^{AΔt}, F_u = BΔt, F_w = C, U = U^c, W = W^c Δt (Table 5, eqs. 434–437).
- So P_{n+1} = e^{AΔt} P_n (e^{AΔt})ᵀ + Δt² B U^c Bᵀ + Δt C W^c Cᵀ (439). "The dynamic error term is exponential, the measurement error term is quadratic, and the perturbation error term is linear" in Δt.
- Q = Δt² B U^c Bᵀ + Δt C W^c Cᵀ (443).
- For the full IMU: B has blocks −R (on δv) and −I (on δθ); C has I on δa_b and I on δω_b (450).
- U^c = diag(σ_ã² I, σ_ω̃² I), W^c = diag(σ_aw² I, σ_ωw² I) (452). Q_i is as in (456).
- F_i is "trivial" because isotropy gives (−R)σ²I(−R)ᵀ = σ²I. "This is not possible when considering non-isotropic IMUs, where a proper Jacobian F_i = [B C] should be used together with a proper specification of Q_i" (E.2.1).

### Inferences
**1. Mapping datasheet noise densities into Q_i** (my inference, not printed by Solà):
- In App. E the IMU noise is a *sampled* control noise with per-sample std σ_ã [m/s²] (U = U^c, not multiplied by Δt).
- For a datasheet velocity/angle random walk density N_a [m/s²/√Hz] or N_g [rad/s/√Hz] sampled at 1/Δt, the usual conversion is σ_ã² ≈ N_a²/Δt. This gives V_i = σ_ã²Δt² = N_a²Δt and Θ_i = N_g²Δt.
- Bias random-walk densities σ_aw (units m/s²/√s, equivalently m/s³/√Hz) enter as A_i = σ_aw²Δt; likewise Ω_i = σ_ωw²Δt.
- The VN-200 datasheet values should be converted this way. Check the convention against VectorNav's stated units; this was not researched here.

**2. Gravity and initial attitude for the VN-200 AUV case:**
- With an absolute magnetometer/accelerometer-levelled start, it is simpler to drop δg (17-state error) and fix g = [0, 0, +g(μ)] in NED, as Fossen/MSS does.
- Solà explicitly allows this "classical approach".

**3. Exact discretisation:** F_x can be computed as expm(AΔt), as MSS does, instead of the Euler form (269). Solà endorses this via Table 5.

### Gaps
- The printed units of σ_aw and σ_ωw in (261)–(264) appear in the PDF as "[m/s²√s]" and "[rad/s√s]". It is not clear whether later revisions corrected these to the more conventional random-walk density units. Not verifiable here.

---

## Q4. Solà: measurement update, H = H_x X_δx, injection, reset G; commonly omitted steps

### Takeaway
1. **Correct:** use H = H_x·X_δx, where X_δx is identity except for the 4×3 quaternion block Q_δθ.
2. **Inject:** apply the update multiplicatively to q.
3. **Reset:** set δx̂ to 0 and P ← GPGᵀ, with G = blkdiag(I₆, I − [½δθ̂]×, I₉).

Most implementations set G = I. Solà says the full G "should produce more precise results, which might be of interest for reducing long-term error drift". He also flags the simple (I − KH)P covariance update as numerically poor.

### Cited Findings
All findings in this section are from the [PDF copy](https://github.com/TurtleZhong/msckf_mono/blob/master/Quaternion%20kinematics%20for%20the%20error-state%20Kalman%20filter.pdf).

**The three steps (§6):** "1. observation of the error-state via filter correction, 2. injection of the observed errors into the nominal state, and 3. reset of the error-state."

**Correction (271)–(276):**
- Measurement model: y = h(x_t) + v, v ~ N{0, V}.
- K = PHᵀ(HPHᵀ + V)⁻¹ (273); δx̂ ← K(y − h(x̂_t)) (274); P ← (I − KH)P (275).
- H ≡ ∂h/∂δx evaluated at x (276), because x̂_t = x while δx̂ = 0 before observation.
- Footnote 26 calls (275) the simplest form, "known to have poor numerical stability", and recommends either:
  - the symmetric form P ← P − K(HPHᵀ + V)Kᵀ, or
  - the Joseph form P ← (I − KH)P(I − KH)ᵀ + KVKᵀ.

**Chain rule (277)–(280):**
- H = (∂h/∂x_t)|_x · (∂x_t/∂δx)|_x = H_x X_δx (277).
- H_x is "the Jacobian one would use in a regular EKF" and is sensor-specific (not given).
- X_δx = blkdiag(I₆, Q_δθ, I₉) (279), of size 19×18, with Q_δθ = ∂(q⊗δq)/∂δθ = [q]_L · ½[0 0 0; I₃] (row of zeros on top).
- Expanded (280), with rows (w, x, y, z):
```
Q_δθ = ½ | −q_x  −q_y  −q_z |
         |  q_w  −q_z   q_y |
         |  q_z   q_w  −q_x |
         | −q_y   q_x   q_w |
```

**Injection (281)–(282):**
- x ← x ⊕ δx̂.
- p ← p + δp̂; v ← v + δv̂; q ← q⊗q{δθ̂} (282c); a_b ← a_b + δâ_b; ω_b ← ω_b + δω̂_b; g ← g + δĝ.

**Reset (283)–(287):**
- δx ← g(δx) = δx ⊖ δx̂ (283); δx̂ ← 0 (284); P ← G P Gᵀ (285); G ≜ ∂g/∂δx|_{δx̂} (286).
- G = blkdiag(I₆, I − [½δθ̂]×, I₉) (287).
- "In major cases, the error term δθ̂ can be neglected, leading simply to a Jacobian G = I₁₈, and thus to a trivial error reset. This is what most implementations of the ESKF do. The expression here provided should produce more precise results, which might be of interest for reducing long-term error drift in odometry systems."

**Reset derivation (§6.3.1):**
- The true orientation is unchanged on reset: q⁺⊗δq⁺ = q⊗δq (288), and q⁺ = q⊗δq̂ (289).
- Therefore δq⁺ = δq̂*⊗δq = [δq̂*]_L·δq (290).
- This yields δθ⁺ = −δθ̂ + (I − [½δθ̂]×)δθ + O(‖δθ‖²) (292b) and ∂δθ⁺/∂δθ = I − [½δθ̂]× (293).

### Inferences
**Steps commonly omitted, and the consequence by Solà's account:**
- **(a) Reset Jacobian:** G = I is the usual simplification. The cost is a slightly mis-rotated attitude covariance after large corrections, and Solà ties it to long-term drift.
- **(b) Covariance update form:** using P ← (I − KH)P instead of Joseph risks loss of symmetry and positive-definiteness. MSS uses the Joseph form (see Q7).
- **(c) Covariance prediction:** this must *not* be skipped.
- **(d) Mean propagation (267):** this can be skipped because it is identically zero.

**Worked Jacobian for the user's model-based velocity aid** (my derivation, not in Solà):
- Suppose the hydrodynamic/thrust model provides body-frame velocity relative to water, ν_r = [u_r v_r w_r]. Take the filter velocity v in NED and a known (or zero) current v_c^n.
- Measurement: h = R_tᵀ(v_t − v_c).
- With the local error, R_t = R(I + [δθ]×), so h ≈ Rᵀ(v − v_c) + Rᵀδv + [Rᵀ(v − v_c)]× δθ.
- Hence H blocks: δv → Rᵀ, δθ → [Rᵀ(v − v_c)]×, all others → 0.
- The δθ block is the same structure as MSS's magnetometer block S(Rᵀm_ref) (Q7).
- If H is built via Solà's chain rule instead, H_x w.r.t. q times Q_δθ should give the same result. That equality is a useful unit test.

### Gaps
- Solà gives no sensor-specific H_x. GNSS position/velocity, magnetometer and model-velocity Jacobians must be derived by the user (as above) or taken from Fossen/MSS.

---

## Q5. Solà: global vs local angular-error variants

### Takeaway
The global variant (§7) keeps local body rates ω but defines q_t = q{δθ}⊗q. This changes exactly the entries listed in Table 4:
- the δv/δθ block;
- the δθ/δθ block, which becomes I;
- the δθ/δω_b block, which becomes −RΔt;
- Q_δθ, which uses [q]_R;
- injection, which becomes left-multiplication;
- the reset Jacobian, which becomes I + [½δθ̂]×.

### Cited Findings
All findings in this section are from the [PDF copy](https://github.com/TurtleZhong/msckf_mono/blob/master/Quaternion%20kinematics%20for%20the%20error-state%20Kalman%20filter.pdf).
- **Definition:** q_t = δq⊗q = q{δθ}⊗q. "We keep the local definition of the angular rates vector ω … as the measure of the angular rates provided by the gyrometers is in body frame" (§7 intro).
- **Continuous error dynamics (294):**
  - δv̇ = −[R(a_m − a_b)]× δθ − R δa_b + δg − R a_n
  - δθ̇ = −R δω_b − R ω_n
  - the rest are unchanged.
- **Derivation:** from R_t = (I + [δθ]×)R (295) and δθ̇ = ω_G = R δω (307).
- **Euler transition matrix (310)**, rows δv and δθ:
  - δv row: [0, I, −[R(a_m − a_b)]×Δt, −RΔt, 0, IΔt];
  - δθ row: [0, 0, I, 0, −RΔt, 0].
- **Noise:** F_i and Q_i are unchanged under isotropic noise (311).
- **Q_δθ (global)** = [q]_R·½[0;I₃] (312), expanded:
```
Q_δθ = ½ | −q_x  −q_y  −q_z |
         |  q_w   q_z  −q_y |
         | −q_z   q_w   q_x |
         |  q_y  −q_x   q_w |
```
- **Injection:** q ← q{δθ̂}⊗q (313c).
- **Reset:** G = blkdiag(I₆, I + [½δθ̂]×, I₉) (316)/(322).
- **Table 4 summary**, local vs global:

  | Item | Local | Global |
  |---|---|---|
  | ∂δv⁺/∂δθ | −R[a_m − a_b]×Δt | −[R(a_m − a_b)]×Δt |
  | ∂δθ⁺/∂δθ | Rᵀ{(ω_m − ω_b)Δt} | I |
  | ∂δθ⁺/∂δω_b | −IΔt | −RΔt |
  | Q_δθ | uses [q]_L | uses [q]_R |
  | Injection | q⊗q{δθ̂} | q{δθ̂}⊗q |
  | Reset | I − [½δθ̂]× | I + [½δθ̂]× |
- **Why consider it:** Solà cites Li & Mourikis (2012) as evidence that the globally-defined error "has better properties" (§5.3).

### Inferences
- For an NED-frame AUV filter, the global error δθ is expressed in NED. Its third component is then directly the heading error, which makes interpreting yaw covariance and GNSS-heading/magnetometer observability more intuitive.
- Fossen/MSS uses the **local** (body) error (see Q7), so matching MSS for validation favours the local variant.

### Gaps
- Solà gives no quantitative comparison of the two variants. The claim of "better properties" rests on the cited Li & Mourikis work, which was not reviewed here.

---

## Q6. Fossen: chapter titles/numbers and coverage (2011 vs 2021), observers covered

### Takeaway
- **2011 edition:** the navigation/observer material is Chapter 11, "Sensor and Navigation Systems", including Kalman filter design (§11.3) and nonlinear passive observers (§11.4). This comes from a search snippet; the full TOC is not verified.
- **2nd edition (2021):** the material is split into Chapter 13, "Model-Based Navigation Systems", and Chapter 14, "Inertial Navigation Systems". Ch. 14 contains:
  - IMU models;
  - attitude estimation, including a nonlinear attitude observer using reference vectors;
  - direct filters for aided INS;
  - indirect filters for aided INS: ESKF with attitude measurements and an error-state EKF with attitude estimation.
- **3rd edition:** MSS headers now reference a forthcoming 3rd edition (2027).

### Cited Findings
- **2nd-edition Chapter 14 TOC** — [search snippet of Wiley/e-bookshelf sample](https://content.e-bookshelf.de/media/reading/L-16324268-da82399760.pdf); I could not open the PDF itself, which was blocked:
  - Chapter 14, "Inertial Navigation Systems", pp. 443–492.
  - 14.1 Inertial Measurement Unit (444): 14.1.1 Attitude Rate Sensors (446); 14.1.2 Accelerometers (446); 14.1.3 Magnetometer (449).
  - 14.2 Attitude Estimation (451): 14.2.1 Static Mapping from Specific Force to Roll and Pitch Angles (451); 14.2.2 VRU Transformations (452); 14.2.3 Nonlinear Attitude Observer using Reference Vectors (453).
  - 14.3 Direct Filters for Aided INS (457): 14.3.1 Fixed-gain Observer using Attitude Measurements (458); 14.3.2 Direct Kalman Filter using Attitude Measurements (462); 14.3.3 Direct Kalman Filter with Attitude Estimation (465).
  - 14.4 Indirect Filters for Aided INS (467): 14.4.1 Introductory Example (469); 14.4.2 Error-state Kalman Filter using Attitude Measurements (472); 14.4.3 Error-state Extended Kalman Filter with Attitude Estimation (480).
- **2nd-edition chapter list and slides** — [FossenHandbook README](https://github.com/cybergalactic/FossenHandbook):
  - Part II = Ch. 11 Introduction to Part II; 12 Guidance Systems; 13 Model-Based Navigation Systems; 14 Inertial Navigation Systems; 15 Motion Control Systems; 16 Advanced Motion Control Systems.
  - Appendices A–D: Nonlinear Stability Theory; Numerical Methods; Model Transformations; Non-dimensional EoM.
  - Wiley, 2nd ed., April 2021, ISBN 978-1-119-57505-4. The textbook is used in NTNU TTK4190.
  - Slides (PDF): Ch13 last modified 2022-11-04 (Google Drive); Ch14 last modified 2025-10-12 (Dropbox). Both download hosts were blocked for me.
  - The README marks the Ch5/Ch6 slides as "3rd" edition (modified 2026-08-27).
- **2nd edition replaces 2011:** "The 2nd edition replaces the 2011 version" — [MSS documentation/Textbooks.pdf](https://github.com/cybergalactic/MSS/blob/master/documentation/Textbooks.pdf).
- **2011 edition** — [search snippet, scispace mirror](https://scispace.com/pdf/handbook-of-marine-craft-hydrodynamics-and-motion-control-2qu7epb0pt.pdf); the page itself was blocked:
  - Chapter 11 is "Sensor and Navigation Systems".
  - §11.3 Kalman Filter Design includes discrete-time KF, continuous-time KF, EKF, and a corrector–predictor representation for nonlinear observers.
  - §11.4 covers Nonlinear Passive Observer Designs.
- **Nonlinear attitude observer cited in MSS** — [MSS quatObserver.m header](https://github.com/cybergalactic/MSS/blob/master/INS/functions/quatObserver.m):
  - Implements the quaternion nonlinear observer of Mahony, Hamel & Pflimlin (2008) and Grip et al. (2013): H. F. Grip, T. I. Fossen, T. A. Johansen, A. Saberi, "Nonlinear Observer for GNSS-Aided Inertial Navigation with Quaternion-Based Attitude Estimation", ACC 2013, pp. 272–279, doi 10.1109/ACC.2013.6579849.
  - The injection term uses two reference vectors, σ = k₁v₁×Rᵀv₀₁ + k₂v₂×Rᵀv₀₂.
  - Continuous form: q̇ = T(ω_imu − b̂_ars + σ)q and ḃ_ars = −K_I σ. It is discretised with the matrix exponential plus normalisation.
  - The header claims USGES (uniform semiglobal exponential stability).
- **Forthcoming 3rd edition** — [MSS ins_mekf.m header](https://github.com/cybergalactic/MSS/blob/master/INS/functions/ins_mekf.m): MSS INS function headers now cite "T. I. Fossen (2027) … 3rd edition".

### Inferences
- **2011 vs 2021, the main change (partly inferred):** in 2011 sensors, KF and observers sat in one Chapter 11. In 2021 they are split into model-based navigation (Ch. 13: KF/observers with vessel models, wave filtering) and INS (Ch. 14). Ch. 14 gives explicit direct-vs-indirect (error-state) aided-INS treatment, with an Euler-angle ESKF and a quaternion MEKF-style error-state EKF.
- **Relevance to this project:**
  - For **model-aided velocity**, Ch. 13 (model-based observers) is the relevant Fossen material.
  - For **IMU-driven ESKF**, Ch. 14.4 is.
  - The user's design combines both.

### Gaps
- **Full TOCs not verified:** I could not verify the full 2011 Chapter 11 TOC, e.g. whether it has sections titled "Integration Filters for IMU and GNSS" or "Attitude Observers" **[unverified, from memory: I believe the 2011 Ch. 11 included §11.5 "Integration Filters for IMU and Global Navigation Satellite Systems" and §11.6 attitude observers]**. Nor could I verify the full Ch. 13 TOC of the 2nd edition.
- **Grip et al. 2015:** a journal version, "Nonlinear observer for GNSS-aided inertial navigation with quaternion-based attitude estimation" (IEEE TCST, 2015), and the "Grip/Fossen/Johansen/Saberi" family **[unverified for this note]**. Only the ACC 2013 paper is verified, via the MSS header.
- **No book text:** I read no text, equations or equation numbers from either edition of the book. Everything about Fossen's notation below is inferred from MSS code structure, not the book.

---

## Q7. Fossen notation for INS/ESKF (states, frames, specific force, biases) and marine measurement models

### Takeaway
MSS's quaternion ESKF (`ins_mekf`, `ins_mekf_psi`) uses these conventions:
- **Nominal state:** x_ins = [p^n; v^n; b_acc; q; b_ars] in NED (16-D, optionally +1 sea-level integral).
- **Error state:** δx = [δp; δv; δb_acc; δa; δb_ars] (15-D), where δa is 2× the Gibbs vector (a local, body-frame attitude error).
- **Bias model:** first-order Gauss–Markov with time constants T_acc and T_ars.
- **Gravity:** WGS-84 g(μ) as g^n = [0, 0, g]ᵀ.
- **Loop order:** corrector-then-predictor, Joseph update, multiplicative injection q ← q⊗δq, no reset Jacobian.

Marine aids in MSS are:
- GNSS position;
- optional NED velocity;
- magnetometer reference vector or compass yaw;
- gravity reference vector when accelerations are small;
- an average-sea-level pseudo-measurement;
- a separate pressure-aided heave ESKF.

### Cited Findings
- **Error-state structure in `ins_mekf.m`** — [MSS ins_mekf.m](https://github.com/cybergalactic/MSS/blob/master/INS/functions/ins_mekf.m). Header and structure as read:
  - ESKF "for Inertial Navigation Systems (INS) that are aided by magnetometer and positional data"; attitude parameterised "using the 4-parameter unit quaternion … and the Gibbs vector in the Multiplicative Error State Kalman Filter (MEKF) formulation".
  - 15-dimensional error state. Inputs include latitude μ (for gravity), h (sample time), Q_d, R_d, T_acc, T_ars, f_imu, w_imu, m_imu, m_ref (the NED magnetic reference), y_pos and optional y_vel.
  - "The IMU axes are assumed to be oriented forward-starboard-down."
- **Continuous error-dynamics matrix A in `ins_mekf.m`**, as I read it. Block order [δp, δv, δb_acc, δa, δb_ars]; R = R(q) (BODY→NED); f = f_imu − b̂_acc; ω = ω_imu − b̂_ars; S(·) = skew matrix:
  - δṗ = δv
  - δv̇ = −R δb_acc − R S(f) δa
  - δḃ_acc = −(1/T_acc) δb_acc
  - δȧ = −S(ω) δa − δb_ars
  - δḃ_ars = −(1/T_ars) δb_ars
- **Discretisation and noise in `ins_mekf.m`:**
  - A_d = expm(A h).
  - The noise input matrix (times h) maps four noise vectors: velocity noise via −R, acc-bias noise via I, gyro noise via −I, gyro-bias noise via I.
  - P_prd = A_d P̂ A_dᵀ + E_d Q_d E_dᵀ, with E_d = h·E.
- **Nominal propagation in `ins_mekf.m`:**
  - a = R f + g^n;
  - p ← p + h v + ½h² a;
  - v ← v + h a, described as "exact discretization";
  - q ← expm(T(ω) h) q, then normalised, where T(ω) = ½[0 −ωᵀ; ω −S(ω)], i.e. q̇ = ½ q⊗[0;ω].
- **Correction in `ins_mekf.m`:**
  - K = P_prd Cᵀ(C P_prd Cᵀ + R_d)⁻¹;
  - Joseph form P̂ = (I − KC)P_prd(I − KC)ᵀ + K R_d Kᵀ;
  - additive reset of p, v, b_acc, b_ars.
- **Attitude injection in `ins_mekf.m`:**
  - δq̂ = [2; δâ]/√(4 + δâᵀδâ) (the "2 × Gibbs vector to error quaternion" conversion);
  - q ← q⊗δq̂, then normalisation.
  - No covariance reset Jacobian is applied (G = I implicitly).
- **Measurement rows (C_d) in `ins_mekf.m`:**
  - position: [I 0 0 0 0] with innovation y_pos − p̂;
  - NED velocity: [0 I 0 0 0];
  - magnetometer: [0 0 0 S(Rᵀ m̄_ref) 0], innovation m̄_imu − Rᵀ m̄_ref, using normalised vectors;
  - gravity reference "when translational acceleration … is negligible": [0 0 0 S(Rᵀ v₀₁) 0] with v₀₁ = [0, 0, −1] NED, innovation f̄ − Rᵀv₀₁;
  - sea-level pseudo-measurement: an integral-of-z state (∫z_n = 0) appended as a 16th error state.
- **Quaternion convention** — [MSS GNC Rquat.m, Tquat.m, quatprod.m](https://github.com/cybergalactic/MSS) (found by repository search):
  - q = [η ε₁ ε₂ ε₃], scalar-first;
  - R(q) = I + 2ηS(ε) + 2S(ε)²;
  - quatprod gives [η₁η₂ − ε₁ᵀε₂; η₂ε₁ + η₁ε₂ + ε₁×ε₂], i.e. the Hamilton product.
- **Gravity model** — [MSS gravity.m](https://github.com/cybergalactic/MSS/blob/master/INS/functions/gravity.m): g(μ) from WGS-84, the Somigliana form: 9.7803253359(1 + 0.0019318504 sin²μ)/√(1 − 0.006694384442 sin²μ).
- **Euler-angle ESKF, `ins_euler`** — [MSS ins_euler.m](https://github.com/cybergalactic/MSS/blob/master/INS/functions/ins_euler.m):
  - error state δx = [δp; δv; δb_acc; δθ; δb_ars];
  - aided by compass and position, optional velocity; "singular for θ = ±90°";
  - a 2026-07-15 revision added "consistent additive Euler-angle error dynamics and reference-vector measurement Jacobian".
- **Heave ESKF, `ins_heave`** — [MSS ins_heave.m](https://github.com/cybergalactic/MSS/blob/master/INS/functions/ins_heave.m):
  - ESKF in heave aided by pressure, p = p₀ + ρ g z;
  - states: down position, down velocity, accelerometer bias.
- **Other MSS INS entry points** — [MSS Quick Reference](https://github.com/cybergalactic/MSS/blob/master/MSS%20Quick%20Reference.md):
  - `ins_ahrs` (position + AHRS attitude);
  - `quatMEKF` (MEKF attitude observer for 9-DOF IMU);
  - `insSignal` (test signal generator);
  - `magneticField` (NED reference vectors for cities);
  - demo scripts `SIMaidedINSquat`, `SIMaidedINSeuler`, `SIMaidedINSheave`, `SIMquatMEKF`, `SIMquatObserver`;
  - examples `exINS_MEKF`, `exINS_Euler`, `exINS_AHRS`, `exINSwaveFilter`.
- **Typical rates** — [MSS SIMaidedINSquat.m header](https://github.com/cybergalactic/MSS/blob/master/INS/SIMaidedINSquat.m): IMU f_fast ~1000 Hz, magnetometer/compass ~100 Hz, position f_slow ~5 Hz. The ESKF runs "as a corrector (with new measurements) or as a predictor (without new measurements)".

### Inferences
**Book-level notation** (inferred from MSS variable names; **[unverified against book text]**):
- p^n, v^n in NED;
- f^b_imu, ω^b_imu;
- b^b_acc, b^b_ars ("ARS" = attitude rate sensor);
- R^n_b(q) with q = [η, εᵀ]ᵀ;
- g^n;
- Gauss–Markov biases with T_acc, T_ars;
- discrete noise matrices Q_d, R_d;
- measurement matrix C_d;
- sample time h;
- the "corrector–predictor" loop.

**Differences in MSS from Solà's ESKF:**
- **(i) Bias model:** Gauss–Markov (−1/T) instead of pure random walk. Set T → ∞ to recover Solà.
- **(ii) Gravity:** fixed WGS-84 g(μ), not estimated, so 15 states vs Solà's 18.
- **(iii) Attitude error:** δa = 2×Gibbs, which agrees with δθ to first order. The injection quaternion [2; δa]/√(4 + |δa|²) is exact for the Gibbs parameterisation, whereas Solà uses Exp(δθ).
- **(iv) State ordering:** biases are interleaved, [δp δv δb_acc δa δb_ars] vs Solà's [δp δv δθ δa_b δω_b δg].
- **(v) Velocity noise:** in MSS the velocity-noise column is −R, whereas Solà simplifies it to I by isotropy (equivalent for isotropic noise).
- **(vi) Process-noise scaling:** E_d = hE, so the process-noise term scales as h² Q_d. That matches Solà's sampled-control-noise scaling (Δt²BUBᵀ) but is applied also to the bias-driving noise, which Solà scales by Δt. The user should check how Q_d is defined in SIMaidedINSquat before reusing tuning values.
- **(vii) Loop order:** MSS corrects before predicting (corrector–predictor), while Solà presents predict-then-correct. These are equivalent per step.
- **(viii) Model blocks agree:** the A-matrix blocks −R S(f) (δv/δa) and −S(ω) (δa/δa) equal Solà's local-error continuous-time (237b,c) with a_m − a_b ≡ f and ω_m − ω_b ≡ ω.

**For the VN-200, no-DVL, no-depth AUV:**
- GNSS is only available at the surface.
- The MSS `pseudoFlag` sea-level integral, the magnetometer reference-vector update and the gravity reference vector (when quasi-static) are the directly reusable marine aids.
- The thrust/hydrodynamic model velocity (relative to water) is not in MSS. It would be added as a body-frame velocity row with H blocks Rᵀ (δv) and S(Rᵀ(v^n − v_c^n)) (δa) (see Q4).
- Without depth or DVL, vertical position/velocity are weakly observable underwater. Expect reliance on the model's w-component and on surfacing GNSS. This is an inference; quantify it with an observability test.

### Gaps
- The book's own equation numbers for Fossen's ESKF (Ch. 14.4.2–14.4.3) could not be read.
- Whether the book defines the attitude error as 2×Gibbs (as MSS does) or as a rotation vector is unverified from book text.
- The book's marine-specific measurement models in Ch. 13 (model-aided velocity, wave filtering) were not read.

---

## Q8. Notation map: Solà ↔ Fossen (book/MSS), with convention flags

### Takeaway
Solà (Hamilton, q_GL, local error) and Fossen/MSS (scalar-first Hamilton, R^n_b, local 2×Gibbs error) are compatible. G = NED, L = BODY.

The things to watch are:
- symbol clashes: Solà's g is gravity *and* the reset function; Fossen's η is the 6-DOF pose but also the quaternion scalar part;
- different state ordering;
- bias models (random walk vs Gauss–Markov);
- the gravity sign, which depends on NED vs ENU;
- body velocity ν (Fossen kinetics) vs NED velocity v^n (the INS state).

### Cited Findings
The table below is assembled from Solà Table 3 / §5–7 ([PDF copy](https://github.com/TurtleZhong/msckf_mono/blob/master/Quaternion%20kinematics%20for%20the%20error-state%20Kalman%20filter.pdf)) and MSS INS/GNC functions ([MSS](https://github.com/cybergalactic/MSS)). Fossen-book symbol choices are **[unverified vs book text]** unless they appear in MSS.

| Quantity | Solà | Fossen (MSS / book-style) | Flag |
|---|---|---|---|
| Global/local frames | G (world), L (body) | {n} NED, {b} BODY (forward-starboard-down IMU axes in MSS) | Solà's frame is generic. Choose G = NED to match Fossen. Do **not** use ENU, or the gravity sign and heading conventions change. |
| Quaternion | q = [q_w, q_x, q_y, q_z], Hamilton, q_GL | q = [η, ε₁, ε₂, ε₃], Hamilton product (quatprod) | Same values. Fossen's η (quaternion scalar part) clashes with η = [n e d φ θ ψ]. |
| Rotation matrix | R = R{q}, x_G = R x_L | R(q) = R^n_b = I + 2ηS(ε) + 2S(ε)² (Rquat) | Identical meaning (body→NED). |
| Skew matrix | [a]× | S(a) (Smtrx) | Identical. |
| Position | p | p^n = [n, e, d] (first 3 of η) | Same in NED. |
| Velocity (INS state) | v (in G) | v^n (NED) | Fossen's kinetic ν₁ = [u v w] is BODY. v^n = R(Θ_nb) ν₁. Do not put ν in the INS state without this rotation. |
| Angular rate | ω (local/body) = ω_m − ω_b | ω^b_nb = ν₂ = [p q r] ≈ ω_imu − b_ars | Same frame (body). Earth rate neglected in both. |
| Accelerometer output | a_m = Rᵀ(a − g) + a_b + a_n (specific force) | f_imu (specific force, body) | Same quantity (specific force). |
| Accel bias | a_b, random walk ȧ_b = a_w | b_acc, Gauss–Markov ḃ = −b/T_acc + w | Set T_acc → ∞ to match Solà. |
| Gyro bias | ω_b, random walk ω̇_b = ω_w | b_ars, Gauss–Markov with T_ars | As above. |
| Gravity | g (estimated, in frame q₀), or fixed (0,0,±g) | g^n = [0, 0, g(μ)]ᵀ (WGS-84, fixed) | In NED g points +z (down). Nominal v̇ = R(a_m − a_b) + g ↔ v̇^n = R f + g^n: same. |
| Attitude error | δθ (local rotation vector), δq ≈ [1, ½δθ] | δa = 2×Gibbs vector, δq = [2; δa]/√(4 + δaᵀδa) | Equal to first order; local (right-multiplied) in both. |
| Error state | δx = [δp δv δθ δa_b δω_b δg] (18) | δx = [δp δv δb_acc δa δb_ars] (15) | Different ordering and size. Permute when comparing F_x/A. |
| Error dynamics δv | −R[a_m − a_b]×δθ − Rδa_b + δg − Ra_n | −R S(f)δa − Rδb_acc + noise | Same. |
| Error dynamics δθ | −[ω_m − ω_b]×δθ − δω_b − ω_n | −S(ω)δa − δb_ars + noise | Same. |
| Transition matrix | F_x (Euler, eq. 269) or e^{AΔt} | A_d = expm(A h) | Same concept. |
| Noise Jacobian / covariance | F_i, Q_i = diag(V_i, Θ_i, A_i, Ω_i) | E_d = hE, Q_d | Δt scaling differs for bias noise (see Q7). |
| Measurement Jacobian | H = H_x X_δx | C_d (built directly w.r.t. error state) | Fossen skips the chain rule by writing C_d w.r.t. δx directly. |
| Measurement noise | V | R_d | **Symbol clash:** Fossen's R_d is noise covariance; Solà's R is rotation. |
| Innovation | y − h(x̂_t) | δy = y − ŷ | Same. |
| Covariance update | (I − KH)P (Joseph recommended, fn. 26) | Joseph form | – |
| Injection | q ← q⊗q{δθ̂} | q ← q⊗δq̂ (from δâ) | Same side (right/local). |
| Reset | P ← GPGᵀ, G = blkdiag(I₆, I − [½δθ̂]×, I₉) | none (G = I) | MSS omits it, as Solà says most implementations do. |
| Time step | Δt | h | – |
| Earth rate | neglected (fn. 23) | neglected in the MSS INS | Fine for a MEMS VN-200. |

### Inferences
**Sign and frame checks to unit-test in MATLAB:**
1. **Stationary, level, NED:** f_imu should read ≈ [0, 0, −g]. Then R f + g^n = 0. This is the same check as MSS's gravity reference v₀₁ = [0, 0, −1].
2. **Rotation equivalence:** R(q) from the quaternion must equal Fossen's Rzyx(φ, θ, ψ) for the same attitude. This guarantees q ↔ Θ = [φ θ ψ] consistency with the simulator's η.
3. **Jacobian check:** numerically perturb q⊗Exp(δθ) and compare against the analytic H blocks (Q_δθ or S(Rᵀ·)).

**State definitions and remaining choices:**
- Fossen's 6-DOF simulator states (η, ν) relate to the INS states by:
  - p^n = η₁:₃;
  - Θ = η₄:₆ ↔ q;
  - v^n = R(q) ν₁:₃;
  - ω = ν₄:₆.
- The ESKF therefore never estimates ν directly. Body velocity is an output: ν̂₁ = Rᵀ v̂^n.
- **Velocity aid:** the model-based velocity aid should be compared against ν_r = ν₁ − Rᵀ v_c^n. If ocean current is unknown, augment the error state with a (slowly varying) current v_c^n. This is a common approach in model-aided AUV navigation (inference; not from these two sources).

### Gaps
- No access to Fossen's book text, so book symbols for the error-state (e.g. δx, δa, b^b_acc, Q_d vs Q) and their equation numbers are unverified.
- Not covered by either source as read:
  - VN-200-specific noise parameters;
  - whether VectorNav reports specific force with the NED sign convention used here;
  - its magnetometer frame.

  Check against the VectorNav documentation.
