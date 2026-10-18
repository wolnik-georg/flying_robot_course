"""Access to the authors' learned-FBL bundle (Hsieh et al. 2026), READ-ONLY.

The bundle stays where it was received (default ~/Desktop/learned_fbl_core, override with FBL_BUNDLE). We never edit it and never copy
its code into this repo: we import it and only override the module-level vehicle constants it reads (`quad_params` in
KNODE/Training/Velocity_Field/force_batch_fo.py). Integrity: `bundle_hash()` is the sha256 over the sorted (relative path, file sha256)
list of all non-pyc files; the value recorded on first use is in experiments/fbl/README.md.
"""
from __future__ import annotations

import hashlib
import os
import sys
from pathlib import Path

BUNDLE = Path(os.environ.get("FBL_BUNDLE", "~/Desktop/learned_fbl_core")).expanduser()


def bundle_hash() -> str:
    h = hashlib.sha256()
    for p in sorted(BUNDLE.rglob("*")):
        if p.is_file() and "__pycache__" not in p.parts and p.suffix != ".pyc":
            h.update(str(p.relative_to(BUNDLE)).encode())
            h.update(hashlib.sha256(p.read_bytes()).digest())
    return h.hexdigest()


def load():
    """Import the bundle modules; returns a simple namespace (torch must be available)."""
    if str(BUNDLE) not in sys.path:
        sys.path.insert(0, str(BUNDLE))
    from types import SimpleNamespace

    import KNODE.Training.Velocity_Field.force_batch_fo as force  # noqa: E402
    import KNODE.Training.nominal_dynamics as nominal  # noqa: E402
    import KNODE.Training.downwash_residual as dw  # noqa: E402
    import KNODE.Training.nn_models_full_only_zr as nnm  # noqa: E402
    return SimpleNamespace(force=force, nominal=nominal, dw=dw, nnm=nnm, path=BUNDLE)


def set_vehicle(b, mass: float, arm_length: float) -> None:
    """Override the vehicle constants the ROM reads (module-level dict). The propeller radius (0.05/2 m) is hard-coded in the bundle."""
    b.force.quad_params["mass"] = float(mass)
    b.force.quad_params["arm_length"] = float(arm_length)
