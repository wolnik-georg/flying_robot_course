# Setup

## Requirements
* Linux x86-64, `git`, `make`, `gcc`, `swig` (4.0), `python3` with `python3-dev` (tested: Python 3.10.12, gcc 11.4, swig 4.0.2).
* Python packages: `numpy`, `scipy`, `pyyaml` (tested: numpy 2.2.6, scipy 1.15.3; not pinned). `matplotlib` only for plots.
* About 1 GB of disk for the firmware clone and the build. No GPU, no ROS, no drone.

## Build the firmware bindings (once)
```
./firmware/build_bindings.sh                                   # clones Bitcraze crazyflie-firmware from GitHub, applies our two patches, builds
./firmware/build_bindings.sh --source /path/to/crazyflie-firmware   # a local copy as the source (it is only read)
```
The script clones Bitcraze at commit `83aa19ed`, applies `firmware/patches/0001-local-snap-ccb0606d.patch` (one local commit of ours) and `firmware/patches/crazyflie-firmware-sim.patch` (the controller code and bindings), copies the host C files from `firmware/host/`, configures for the brushless Crazyflie 2.1 and builds the Python extension with the python3 that runs the script. It takes 1-2 minutes (about 25 s from a local source) and prints where the `.so` is. The Rust library of the flight firmware is **not** needed (`--with-rust LIB.a` links it as a fallback). `firmware/README.md` lists what the patches contain and what was left out.

By default the simulator looks for the build in `simulator/_build/crazyflie-firmware/build` (where the script puts it). Another location: `export FRSIM_FIRMWARE_BUILD=/path/to/build`. The firmware prints debug text; the simulator hides it (`FRSIM_FIRMWARE_VERBOSE=1` shows it).

## Check the installation
```
python3 tests/test_smoke.py --procs 4          # 15 flights, about 30 s
python3 tests/test_equivalence.py --quick      # 22 reference episodes, about 40 s
python3 tests/test_equivalence.py              # all 199 reference episodes, 5 min with 4 free processes, 12 min with 3 processes on a busy machine
```
The scripts wait for free CPU and memory before they start (`--no-wait` skips that; they use at most `--procs` processes).

## Plots
The scripts call `scripts/plot_trace.py` in a subprocess. If the matplotlib of your `python3` does not work (for example a numpy version mismatch), set `FRSIM_PLOT_PYTHON` to a python that has it; otherwise only the CSV trace is written.
