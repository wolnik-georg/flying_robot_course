#!/bin/bash
# Build the Python bindings (cffirmware) of the simulator from a fresh clone of Bitcraze crazyflie-firmware + our patches.
#   ./build_bindings.sh [--source URL_OR_LOCAL_REPO] [--build-dir DIR] [--with-rust LIBRARY.a] [--jobs N] [--clean]
# Default source: https://github.com/bitcraze/crazyflie-firmware.git (the local commit 'add snap.' is applied from patches/0001, the base is Bitcraze 83aa19ed).
# A local repo can be given as source (it is only read: git clone). Result: <build-dir>/crazyflie-firmware/build/_cffirmware.cpython-3xx-*.so + cffirmware.py;
# put that build/ directory in CFFIRMWARE_DIR. Needs: git, make, gcc, swig, python3 + python3-dev (the extension is built for the python3 that runs this script).
# Default build is WITHOUT the Rust library (rust_stubs.c). --with-rust LIBRARY.a links libcf_controller_rs.a instead (fallback; see host/SOURCES.md).
set -euo pipefail
HERE="$(cd "$(dirname "$(readlink -f "$0")")" && pwd)"; SIM="$(dirname "$HERE")"
BASE=83aa19ed47d23cf7fd5d5fd1788f4d490dd0cd3f
SRC=https://github.com/bitcraze/crazyflie-firmware.git; BUILD="$SIM/_build"; RUST=""; JOBS=2; CLEAN=0; PY=${PYTHON:-python3}
while [ $# -gt 0 ]; do case "$1" in
  --source) SRC="$2"; shift 2;; --build-dir) BUILD="$(readlink -f "$2")"; shift 2;; --with-rust) RUST="$(readlink -f "$2")"; shift 2;;
  --jobs) JOBS="$2"; shift 2;; --clean) CLEAN=1; shift;; *) echo "unknown argument $1"; exit 2;; esac; done
[ -z "$RUST" ] || [ -f "$RUST" ] || { echo "Rust library not found: $RUST"; exit 2; }
NAME=crazyflie-firmware; [ -z "$RUST" ] || NAME=crazyflie-firmware-rust
DIR="$BUILD/$NAME"; mkdir -p "$BUILD"
[ $CLEAN = 1 ] && rm -rf "$DIR"
if [ ! -d "$DIR/.git" ]; then
  nice -n 19 git clone -q "$SRC" "$DIR"
  git -C "$DIR" checkout -q "$BASE"
  # only the submodule the bindings need (CMSIS DSP); a local source gives its own vendor/CMSIS as the submodule origin
  if [ -d "$SRC/vendor/CMSIS/.git" ] || [ -f "$SRC/vendor/CMSIS/.git" ]; then URLOPT="-c submodule.vendor/CMSIS.url=$SRC/vendor/CMSIS"; else URLOPT=""; fi
  git -C "$DIR" -c protocol.file.allow=always $URLOPT submodule update --init vendor/CMSIS
  git -C "$DIR" apply "$HERE/patches/0001-local-snap-ccb0606d.patch"
  git -C "$DIR" apply "$HERE/patches/crazyflie-firmware-sim.patch"
fi
cd "$DIR"
export OOT_FIRMWARE_APP="$HERE/host"
if [ -n "$RUST" ]; then export OOT_WITH_RUST=1 OOT_RUST_LIB="$RUST"; else export OOT_WITH_RUST=0; fi
nice -n 19 make -j"$JOBS" cf21bl_defconfig > "$BUILD/$NAME.config.log" 2>&1       # build/.config (CONFIG_PLATFORM_CF21BL)
nice -n 19 make silentoldconfig >> "$BUILD/$NAME.config.log" 2>&1                    # build/include/generated/autoconf.h
nice -n 19 make PYTHON="$PY" bindings_python > "$BUILD/$NAME.build.log" 2>&1       # swig + setup.py build_ext (distutils compiles serially)
SO="$(ls "$DIR"/build/_cffirmware*.so)"
( cd "$DIR/build" && "$PY" -c "import cffirmware as fw; o = fw.controllerOmarIndi_t(); fw.controllerOmarIndiInit(o); print('import ok, mass', o.mass)" )
echo "bindings built: $SO"
echo "CFFIRMWARE_DIR=$DIR/build"
