# Plant model (MATLAB) — reimplementation from the mid-term report

6-DOF AUV plant with quaternion attitude, Fossen-form dynamics in the water-relative velocity,
thrust allocation and lag, and an ocean-current module with a depth gradient.
Plain MATLAB, no toolbox. Octave is **not** tested (not installed on the machine used).

**Status, read this first.** This code was written in one session from the equations of
`AUV_Midterm_Report.pdf` (eqs. 2.3–3.22, 5.3–5.5). The report's own plant (30 files) is not in
this repository and was never seen. So this is an independent reimplementation, and its passing
tests say nothing about the report's code. **Every parameter is ASSUMED or recalled and none is a
measurement of the manta-ray vehicle** (see `config/params_plant.m`; each line carries a source
tag). The thruster geometry is a placeholder with the right structure only.

## Run

```matlab
cd plant_m/config
setup_plant_paths
run_all_tests        % about 1 minute; writes results/test_results.txt and .mat
```

or `matlab -batch "addpath('plant_m/config'); setup_plant_paths; run_all_tests"` from the project root.

## Layout

| Folder | Content |
|---|---|
| `config/` | `params_plant` (all parameters), `thruster_geometry`, `update_params`, `setup_plant_paths` |
| `model/` | `rigid_body` (3.8, 3.9), `added_mass` (3.10, 3.11), `damping` (3.12), `restoring` and `restoring_euler` (3.13), `body_dynamics` (3.14–3.16 incl. M_A·ν̇_c), `plant_rhs` (quaternion), `euler_plant_rhs` (cross-check), `current_profile` (3.20–3.22), `thrust_matrix`, `thrust_alloc` (3.17, 3.18) |
| `sim/` | `simulate` (RK4, renormalise after each full step), `simulate_euler` |
| `lib/` | quaternion and Euler helpers, `rk4` |
| `tests/` | eight suites; each failure control feeds the same check a deliberately wrong input |
| `results/` | `test_results.txt` (one line per check) |

## What is tested, and what is not

Tolerances are fixed in the test files before a run. A *check* passes when its error is below the
tolerance; a *control* passes only when the check rejects the wrong input.

- Structure: M symmetric positive definite, C skew-symmetric, νᵀCν = 0, C_RB against an
  independent Newton–Euler computation, C_A against the entries printed in the report, rank B = 6.
- Closed forms (rg = rb = 0, one term on): linear surge and heave decay, thrust with quadratic
  drag (tanh), terminal speed, undamped roll period, and that no other state moves.
- Conservation: energy, linear and angular momentum of a free body; passivity with damping.
- Invariance: yaw rotation; Galilean shift in a uniform current during a turning manoeuvre; and
  ν̇_c against a finite difference through an Ekman profile. Dropping the M_A·ν̇_c term is the
  failure control and fails the Galilean check, which is the error the report found.
- Cross-check against an Euler-angle plant. **Both plants share `body_dynamics.m`**, so this tests
  the rotation matrix, kinematic matrix and restoring force only, not the dynamics.
- Numerics: observed RK4 order, quaternion norm.

**Not implemented or not tested:** the controller and the 50 Hz / 500 Hz multi-rate loop, the
benchmark against published BlueROV2 data, real thruster geometry, anything about the vehicle's
shell (cavity water, lift, direction-dependent damping). The sign convention and form of the
equations come from the report and were not checked against Fossen (2011).

## Known limitation found by the tests

With the quadratic damping `|ν|ν`, the right-hand side has a kink wherever a velocity component
changes sign, and RK4 then does not show clean fourth-order convergence (observed 4.58, 2.29, 3.83,
2.94 on the test manoeuvre). On a smooth right-hand side (linear damping) it is 3.94 and 3.97. The
order check therefore runs on the smooth case; the quadratic-damping orders are recorded as `info`
rows. The test was changed after its first version failed; the pass window was not.
