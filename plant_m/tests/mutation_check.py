"""Mutation check for plant_m: break the model on purpose in a scratch copy and see whether run_all_tests notices.
Usage: python mutation_check.py <repo root> <out.txt>   (needs MATLAB; edit MATLAB below if installed elsewhere)
"""
import shutil, subprocess, sys
from pathlib import Path

repo, outp = Path(sys.argv[1]), Path(sys.argv[2])
import tempfile; work = Path(tempfile.gettempdir()) / "plant_mutation"
MATLAB = r"C:\Program Files\MATLAB\R2025b\bin\matlab.exe"

MUT = [
    ("baseline (no change)", None, None, None),
    ("C_RB: sign of m S(S(w) rg) in the off-diagonal block", "model/rigid_body.m", "X = -m * skewm(v) - m * skewm(skewm(w) * rg);", "X = -m * skewm(v) + m * skewm(skewm(w) * rg);"),
    ("C_RB: sign of the m S(S(v) rg) term", "model/rigid_body.m", "m * skewm(skewm(v) * rg) - skewm(p.Io * w)", "-m * skewm(skewm(v) * rg) - skewm(p.Io * w)"),
    ("C_A: one block sign flipped", "model/added_mass.m", "CA = [zeros(3), -skewm(a);", "CA = [zeros(3), skewm(a);"),
    ("dynamics: M_A nu_c_dot term removed", "model/body_dynamics.m", " - g + MA * nuc_dot);", " - g);"),
    ("dynamics: nu_c_dot loses the -w x (R' v_c) term (the report's bug)", "model/body_dynamics.m", "[-cross(w, R' * vcn) + R' * (dvcdz * zdot); zeros(3, 1)]", "[R' * (dvcdz * zdot); zeros(3, 1)]"),
    ("dynamics: nu_c_dot loses the depth-gradient chain-rule term", "model/body_dynamics.m", "[-cross(w, R' * vcn) + R' * (dvcdz * zdot); zeros(3, 1)]", "[-cross(w, R' * vcn); zeros(3, 1)]"),
    ("dynamics: damping on absolute velocity, not water-relative", "model/body_dynamics.m", "- D * nur - g", "- D * nu - g"),
    ("dynamics: C_RB applied to nu_r instead of nu", "model/body_dynamics.m", "tau(:) - CRB * nu - CA * nur", "tau(:) - CRB * nur - CA * nur"),
    ("dynamics: C_A applied to nu instead of nu_r", "model/body_dynamics.m", "- CA * nur - D", "- CA * nu - D"),
    ("restoring force sign flipped", "model/restoring.m", "g  = -[fg + fb;", "g  = [fg + fb;"),
    ("quaternion kinematics: sign in T(q)", "lib/quat_T.m", "T = [-e(:)'; eta * eye(3) + skewm(e)];", "T = [-e(:)'; eta * eye(3) - skewm(e)];"),
    ("position rate uses R' instead of R", "model/plant_rhs.m", "xd = [R * nu(1:3);", "xd = [R' * nu(1:3);"),
    ("Ekman gradient: sign of the i k_v term", "model/current_profile.m", "dW = (-env.kd + 1i * env.kv) * (W - env.Winf);", "dW = (-env.kd - 1i * env.kv) * (W - env.Winf);"),
    ("thruster matrix: moment is e x r instead of r x e", "model/thrust_matrix.m", "cross(p.thr.r(:, i), p.thr.e(:, i))", "cross(p.thr.e(:, i), p.thr.r(:, i))"),
    ("no quaternion renormalisation after the step", "sim/simulate.m", "    xn(4:7) = xn(4:7) / norm(xn(4:7));\n", "\n"),
    ("quadratic damping loses the absolute value", "model/damping.m", "p.Dq(:) .* abs(nur(:))", "p.Dq(:) .* nur(:)"),
    ("Euler restoring force: one component sign flipped", "model/restoring_euler.m", "g = [ (W - B) * st;", "g = [-(W - B) * st;"),
    ("added mass: surge entry too small by 10%", "config/params_plant.m", "p.A  = [5.5 12.7", "p.A  = [4.95 12.7"),
]

lines = []
caught = tried = 0
for label, fname, old, new in MUT:
    if work.exists():
        shutil.rmtree(work)
    shutil.copytree(repo / "plant_m", work / "plant_m", ignore=shutil.ignore_patterns("results"))
    if fname:
        f = work / "plant_m" / fname
        src = f.read_text(encoding="utf-8")
        assert src.count(old) == 1, f"pattern not unique ({src.count(old)}): {label}"
        f.write_text(src.replace(old, new), encoding="utf-8")
    cfg = (work / "plant_m" / "config").as_posix()
    try:
        r = subprocess.run([MATLAB, "-batch", f"addpath('{cfg}'); setup_plant_paths; run_all_tests;"],
                           capture_output=True, text=True, timeout=600)
        out, err_tail = r.stdout.splitlines(), r.stderr[-300:]
    except subprocess.TimeoutExpired:
        out, err_tail = [], "TIMEOUT after 600 s"
        subprocess.run(["taskkill", "/F", "/IM", "MATLAB.exe"], capture_output=True)
    fails, suite = [], None
    for l in out:
        if l.startswith("test_"):
            suite = l.strip()
        if l.lstrip().startswith("FAIL"):
            fails.append(f"{suite}: {l.strip()[5:].split('  err')[0].strip()}")
    summ = [l for l in out if l.startswith("checks passed")]
    if fname:
        tried += 1
        caught += bool(fails)
        verdict = "CAUGHT" if fails else "MISSED"
    else:
        verdict = "BASELINE OK" if not fails else "BASELINE FAILS"
    lines.append(f"{verdict} | {label}")
    for l in fails[:4]:
        lines.append("         " + l[:170])
    if not out:
        lines.append("         (no output) " + err_tail)
    outp.write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(lines[-1 - min(len(fails), 4) if fails else -1], flush=True)

lines.append(f"\nMutation: {caught} caught of {tried} tried")
outp.write_text("\n".join(lines) + "\n", encoding="utf-8")
print(lines[-1], flush=True)

