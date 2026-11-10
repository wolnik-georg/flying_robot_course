# Firmware side of the simulator (Stage 2, 2026-11-10)

`build_bindings.sh` makes the Python bindings (`cffirmware`) from a fresh clone of Bitcraze crazyflie-firmware at `83aa19ed` (2026-05-08, "Bidirectional DSHOT and RPM telemetry (#1556)") plus:

| patch | content |
|---|---|
| `patches/0001-local-snap-ccb0606d.patch` | `git format-patch` of the local commit `ccb0606d` "add snap." (6 files, +14/-1: `configs/cf2_defconfig`, `pptraj.h/.c`, `stabilizer_types.h`, `crtp_commander_high_level.c`, submodule pointer `vendor/libdw1000`). `pptraj.c` and `stabilizer_types.h` are compiled into the bindings; the rest and the libdw1000 pointer are flight-only (`git apply` skips the gitlink hunk; the bindings do not need libdw1000). |
| `patches/crazyflie-firmware-sim.patch` | the working-tree changes of the FBL firmware tree (`~/Desktop/fbl_v2_firmware/fw`, authors-aligned FBL v2) that the host bindings need, 17 files, +2700/-19, 142 kB: `bindings/cffirmware.i`, `bindings/setup.py` (changed: host layer and Rust library chosen by environment variables, no absolute paths), `controller.h`, `controller_indi.h`, `stabilizer_types.h`, `math3d.h`, `platform_defaults_cf21bl.h`, and the new sources `controller_omar_indi.c/.h`, `controller_tk_indi.c`, `ns2_residual.c/.h`, `fbl_law.c/.h`, `fbl_residual.c/.h`, `fbl_nn_weights.c`. |

`host/` holds the C files from `flying_drone_stack/firmware_app` that the bindings link (see `host/SOURCES.md`) and `rust_stubs.c` (stubs for the Rust symbols; default build without the Rust library).

Left out of the patches (not compiled into the bindings, flight only): `src/modules/src/controller/controller.c` (OOT slots of the ARM build), `src/config/config.h` (stabilizer stack size), `src/deck/drivers/src/rpm.c`, `src/deck/drivers/src/usddeck.c` (RPM deck, uSD logging), `src/hal/src/sensors_bmi088_bmp3xx.c` (sensors), `src/modules/src/Kbuild`, `src/modules/src/controller/Kbuild`, `src/modules/src/Kconfig` (ARM build system; the generated `autoconf.h` of `make cf21bl_defconfig` is identical to the one of the old build, checked 2026-11-10), and the old `fbl_rom_table.c/.h` of the earlier FBL tree (not used by the FBL v2 firmware).

Usage: `./firmware/build_bindings.sh [--source <local crazyflie-firmware repo or URL>] [--build-dir DIR] [--with-rust LIB.a]`, then `export CFFIRMWARE_DIR=<printed build dir>`.
