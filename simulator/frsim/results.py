"""Where results go and what is stored with them.

Every script writes one folder `simulator/results/<date>_<time>_<script>_<controllers>_<scenario>[_<variant>]/` with
`summary.json` (everything needed to rerun: command, gains, plant, firmware build, versions, results), `summary.md`
(the same in readable form) and the plots / CSV files. `--out DIR` chooses the folder, `--no-save` writes nothing.
"""

import hashlib
import json
import shlex
import subprocess
import sys
import time
from dataclasses import asdict
from pathlib import Path

from . import config, firmware


def output_dir(script, controllers, scenario, variant=None, out=None, save=True):
    """Create and return the output folder (None when save is False). Automatic folders never overwrite an old one."""
    if not save:
        return None
    if out:
        folder = Path(out)
        folder.mkdir(parents=True, exist_ok=True)
        return folder
    name = '%s_%s_%s_%s' % (time.strftime('%Y-%m-%d_%H%M%S'), script, '-'.join(controllers), scenario)
    if variant:
        name += '_%s' % variant
    folder = config.RESULTS_DIR / name
    suffix = 1
    while folder.exists():
        suffix += 1
        folder = config.RESULTS_DIR / ('%s_%d' % (name, suffix))
    folder.mkdir(parents=True)
    return folder


def _git_commit():
    """Last commit that touched simulator/ (the commit of this package), or 'unknown'."""
    try:
        done = subprocess.run(
            ['git', '-C', str(config.SIMULATOR_DIR), 'log', '-1', '--format=%H', '--', '.'],
            capture_output=True,
            text=True,
            timeout=10,
        )
        commit = done.stdout.strip()
        return commit if done.returncode == 0 and commit else 'unknown (simulator/ not committed)'
    except (OSError, subprocess.SubprocessError):
        return 'unknown'


def _versions():
    """Python, numpy and scipy versions."""
    import numpy
    import scipy

    return {'python': sys.version.split()[0], 'numpy': numpy.__version__, 'scipy': scipy.__version__}


def _patch_hashes():
    """Short sha256 of every firmware patch file (identifies the firmware source)."""
    return {p.name: hashlib.sha256(p.read_bytes()).hexdigest()[:16] for p in sorted(config.PATCH_DIR.glob('*.patch'))}


def run_info(variant=None):
    """Everything about the run that is not a result: command, time, plant, firmware build, software versions."""
    info = {
        'command': ' '.join(shlex.quote(a) for a in ['python3'] + sys.argv),
        'date': time.strftime('%Y-%m-%d %H:%M:%S'),
        'plant': asdict(config.CALIBRATED_PLANT),
        'firmware_build': dict(firmware.build_info(), patches_sha256=_patch_hashes()),
        'frsim_commit': _git_commit(),
        'versions': _versions(),
    }
    if variant is not None:
        info['variant'] = asdict(variant)
    return info


def write_summary(folder, summary, markdown):
    """Write summary.json and summary.md into the folder (nothing when folder is None)."""
    if folder is None:
        return
    with open(Path(folder) / 'summary.json', 'w') as f:
        json.dump(summary, f, indent=1)
    with open(Path(folder) / 'summary.md', 'w') as f:
        f.write(markdown)
