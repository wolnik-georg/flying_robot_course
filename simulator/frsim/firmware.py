"""Load the compiled firmware bindings (module `cffirmware`, built by firmware/build_bindings.sh).

Importing the bindings needs their directory on sys.path; this is the only place where that happens. `load()` is cached, so every
process loads the extension once.
"""
import importlib
import os
import sys

from . import config

_module = None
VERBOSE_ENV = 'FRSIM_FIRMWARE_VERBOSE'


def _silence_c_stdout():
    """The firmware's debug printf writes to file descriptor 1 and garbles our output. Point fd 1 at /dev/null and give Python its own copy of
    the original stdout. Set FRSIM_FIRMWARE_VERBOSE=1 to keep the firmware output."""
    if os.environ.get(VERBOSE_ENV):
        return
    sys.stdout.flush()
    saved = os.dup(1)
    devnull = os.open(os.devnull, os.O_WRONLY)
    os.dup2(devnull, 1)
    os.close(devnull)
    sys.stdout = os.fdopen(saved, 'w', buffering=1)


def load():
    """Return the `cffirmware` module from FRSIM_FIRMWARE_BUILD (default: the build_bindings.sh location)."""
    global _module
    if _module is not None:
        return _module
    build = config.firmware_build_dir()
    if not build.is_dir():
        raise FileNotFoundError('firmware bindings not found in %s: run simulator/firmware/build_bindings.sh or set %s' % (build, config.FIRMWARE_ENV))
    if str(build) not in sys.path:
        sys.path.insert(0, str(build))
    _module = importlib.import_module('cffirmware')
    _silence_c_stdout()
    return _module


def build_info():
    """Path and modification time of the loaded extension (for result files)."""
    import os
    import time
    mod = load()
    so = getattr(mod, '_cffirmware').__file__
    return {'so': so, 'so_mtime': time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(os.path.getmtime(so)))}
