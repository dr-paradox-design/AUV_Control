"""Per-run data locations and per-run sensor geometry.

Set REAL_DATA_RUN=<name> to work on real_data/data/<name>/ and real_data/results/<name>/.
Unset, the scripts use the original layout (real_data/data/, real_data/results/), which holds the
2026-11-05 log, so its results stay reproducible.
"""
import json
import os
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation as Rot

HERE = Path(__file__).resolve().parent
RUN = os.environ.get('REAL_DATA_RUN') or None
DATA = HERE / 'data' / RUN if RUN else HERE / 'data'
RESULTS = HERE / 'results' / RUN if RUN else HERE / 'results'
EXTRACTED = DATA / 'extracted'
RUNS = DATA / 'runs'

_DEFAULT_TF = {   # 2026-11-05 log, base_link -> child (FLU), used when a run has no tf_static.json
    'dvl_link': {'t': [-0.180, 0.0, -0.109], 'q': [0.0, 0.0, 0.0, 1.0]},
    'pressure_link': {'t': [-0.400, 0.0, 0.0], 'q': [0.0, 0.0, 0.0, 1.0]},
}


OVERRIDES = json.loads((HERE / 'mount_overrides.json').read_text()).get(RUN or '', {})


def static_tf(child):
    """(translation, rotation matrix) of base_link -> child from this run's /tf_static, in FLU.
    A run listed in mount_overrides.json gets its corrected mounting yaw instead (lever arm kept)."""
    f = EXTRACTED / 'tf_static.json'
    tfs = json.loads(f.read_text()) if f.exists() else {}
    tf = tfs.get(child) or _DEFAULT_TF[child]
    R = Rot.from_quat(tf['q']).as_matrix()
    if child in OVERRIDES:
        R = np.array(OVERRIDES[child]['R_base_dvl'], dtype=float)
        assert np.allclose(R @ R.T, np.eye(3), atol=1e-9) and np.isclose(np.linalg.det(R), 1.0), 'not a rotation'
        assert np.allclose(R, Rot.from_euler('z', OVERRIDES[child]['yaw_deg'], degrees=True).as_matrix(), atol=1e-9)
    return np.array(tf['t']), R
