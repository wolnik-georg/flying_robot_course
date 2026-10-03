# Makefile for the Rust out-of-tree Crazyflie controller.
#
# Usage (from this directory):
#   make          — configure + build firmware with Rust controller linked in
#   make clean    — clean the firmware build output
#   make cload    — flash over Crazyradio (requires cfloader on PATH)
#
# NS2 flash-resident weights (additive; plain `make` / `make cload` unchanged):
#   make cload-rnn-flash              — build residual_nn_flash + flash (default npz below)
#   make cload-rnn-flash RNN_WEIGHTS_NPZ=/path/to/other.npz
#
# The resulting firmware binary is at build/cf2.bin.

# ── Paths ─────────────────────────────────────────────────────────────────────

CRAZYFLIE_BASE := /home/georg/Desktop/crazyflie-firmware

# Export so that `cargo build` (invoked from Kbuild) can find the firmware
# headers via the CRAZYFLIE_BASE env var read in build.rs.
export CRAZYFLIE_BASE

# libclang location for bindgen (Ubuntu 22.04 ships clang-14)
export LIBCLANG_PATH := /usr/lib/llvm-14/lib

# ── Drone selection ───────────────────────────────────────────────────────────
# One knob to switch between all three drones. Default = bl (brushless — current focus):
#   make [cload]              → CF2.1 Brushless CF21BL (app-config-bl,  DRONE_PLATFORM=bl)
#   make DRONE=upgrade [cload]→ CF2.1 thrust-upgraded  (app-config,     DRONE_PLATFORM=cf2)
#   make DRONE=std     [cload]→ CF2.1 standard         (app-config-std, DRONE_PLATFORM=cf2)
# DRONE picks the firmware app-config (platform + thrust model) AND, via DRONE_PLATFORM,
# the compile-time physical constants in lib.rs (build.rs → cfg(drone_bl)).
DRONE ?= bl

ifeq ($(DRONE),bl)
  OOT_CONFIG := $(PWD)/app-config-bl
  export DRONE_PLATFORM := bl
else ifeq ($(DRONE),std)
  OOT_CONFIG := $(PWD)/app-config-std
  export DRONE_PLATFORM := cf2
else ifeq ($(DRONE),upgrade)
  OOT_CONFIG := $(PWD)/app-config
  export DRONE_PLATFORM := cf2
else
  $(error Unknown DRONE '$(DRONE)' — use one of: std, upgrade, bl)
endif

# When the selected drone changes, drop the cached base .config so the correct
# platform is re-seeded (CF2 and CF21BL are different platforms). Builds stay
# incremental while the drone is unchanged.
_reset := $(shell mkdir -p build 2>/dev/null; \
  if [ ! -f build/.drone-$(DRONE) ]; then rm -f build/.config build/.drone-*; touch build/.drone-$(DRONE); fi)

# ── Firmware build system ─────────────────────────────────────────────────────

include $(CRAZYFLIE_BASE)/tools/make/oot.mk

# Brushless: seed the CF21BL base config (via the firmware's own cf21bl_defconfig)
# before oot.mk's `all` merges our app-config. CF2 drones use oot.mk's default seed.
ifeq ($(DRONE),bl)
all: brushless-seed
.PHONY: brushless-seed
brushless-seed:
	@if [ ! -f build/.config ]; then \
		echo "Seeding CF21BL base config (configs/cf21bl_defconfig)"; \
		$(MAKE) cf21bl_defconfig; \
	fi
endif

# ── NS2: flash-resident RNN (does not alter default targets) ─────────────────
# Recurses with CF_CARGO_FEATURES / CF_RNN_WEIGHTS_NPZ only for these targets.
RNN_WEIGHTS_NPZ ?= $(abspath $(PWD)/../../experiments/analysis/out/c2_e2e_2026-10-01/full_bank_c1_complete.npz)
RNN_FLASH_MAKE = $(MAKE) CF_CARGO_FEATURES='--features residual_nn_flash' CF_RNN_WEIGHTS_NPZ='$(RNN_WEIGHTS_NPZ)'

.PHONY: all-rnn-flash cload-rnn-flash
all-rnn-flash:
	@test -f '$(RNN_WEIGHTS_NPZ)' || (echo "Missing RNN_WEIGHTS_NPZ: $(RNN_WEIGHTS_NPZ)" >&2; exit 1)
	$(RNN_FLASH_MAKE) all

cload-rnn-flash:
	@test -f '$(RNN_WEIGHTS_NPZ)' || (echo "Missing RNN_WEIGHTS_NPZ: $(RNN_WEIGHTS_NPZ)" >&2; exit 1)
	$(RNN_FLASH_MAKE) cload
