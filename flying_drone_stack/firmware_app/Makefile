# Makefile for the Rust out-of-tree Crazyflie controller.
#
# Usage (from this directory):
#   make          — configure + build firmware with Rust controller linked in
#   make clean    — clean the firmware build output
#   make cload    — flash over Crazyradio (requires cfloader on PATH)
#
# The resulting firmware binary is at build/cf2.bin.

# ── Paths ─────────────────────────────────────────────────────────────────────

CRAZYFLIE_BASE := /home/georg/Desktop/imrc-crazyflie-firmware

# Export so that `cargo build` (invoked from Kbuild) can find the firmware
# headers via the CRAZYFLIE_BASE env var read in build.rs.
export CRAZYFLIE_BASE

# libclang location for bindgen (Ubuntu 22.04 ships clang-14)
export LIBCLANG_PATH := /usr/lib/llvm-14/lib

# ── Firmware build system ─────────────────────────────────────────────────────

# OOT_CONFIG points to our app-config (activates CONFIG_APP_ENABLE and
# CONFIG_CONTROLLER_OOT; overrides the default oot-config filename)
OOT_CONFIG := $(PWD)/app-config

include $(CRAZYFLIE_BASE)/tools/make/oot.mk
