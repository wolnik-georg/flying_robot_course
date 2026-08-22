# Host (simulator) build of the out-of-tree controller

This directory is what lets the Crazyswarm2 SIL simulator run **the same controller source that
flies on the drone**, rather than a reimplementation. Nothing here is compiled into the firmware.

| File | Purpose |
|---|---|
| `oot_host.c` | Host glue: RPM injection, log sinks, stubs the firmware normally provides, the compiled airframe constants, and per-vehicle controller state |
| `cffirmware_bindings.patch` | The `crazyflie-firmware/bindings/` changes needed to link and expose it |

## Why the patch lives here

`~/Desktop/crazyflie-firmware` tracks **bitcraze upstream**, not a fork of ours, so those two files
cannot be committed there. They are only ~110 lines and touch no in-tree source, but the simulator
does not build without them — and being uncommitted in a tree we do not own, a `git checkout`,
`git stash` or upstream pull would delete them without warning. The patch is kept here, in a repo
we control, so that cannot happen.

**If the simulator suddenly fails to build, or `import cffirmware` stops having `oot_*` symbols,
check whether the firmware working tree was reset — then re-apply this patch.**

## Build

```bash
# 1. host build of the controller  (panic=abort is required: a no_std staticlib will not link without it)
cd flying_drone_stack/firmware_app
RUSTFLAGS="-C panic=abort" cargo build --release --target x86_64-unknown-linux-gnu

# 2. bindings changes -- only if not already applied
cd ~/Desktop/crazyflie-firmware
git diff --quiet bindings/ && git apply ~/Desktop/flying_robot_course/flying_drone_stack/firmware_app/host/cffirmware_bindings.patch

# 3. build the bindings
make bindings_python
export PYTHONPATH=~/Desktop/crazyflie-firmware/build:$PYTHONPATH   # required; not pip-installed
```

Verify:

```bash
python3 -c "import cffirmware as f; print(hasattr(f,'controllerOutOfTree'), f.oot_thrust_max())"
# -> True 0.2
```

## Regenerating the patch

If the bindings change again:

```bash
cd ~/Desktop/crazyflie-firmware
git diff bindings/ > ~/Desktop/flying_robot_course/flying_drone_stack/firmware_app/host/cffirmware_bindings.patch
```

## What the bindings changes do

- **`setup.py`** — compile and link `traj_iface.c`, `oot_host.c` and the Rust staticlib; rename five
  firmware symbols the out-of-tree code also defines; and **define the platform**
  (`CONFIG_PLATFORM_CF21BL`). That last one matters more than it looks: without it the bindings use
  generic defaults (`THRUST_MAX` 0.1125 N/motor instead of 0.2), `powerDistribution` maps a
  legitimate force command to a PWM above `UINT16_MAX`, the mixer saturates, and attitude INDI never
  leaves the ground. Override with `OOT_PLATFORM=CONFIG_PLATFORM_CF2` for the brushed drone.
- **`cffirmware.i`** — expose `controllerOutOfTree*`, RPM injection, log readback, the compiled
  airframe constants, `oot_select_drone`, and the INDI/position gain globals so `crazyflies.yaml`
  `firmware_params` can be applied in sim exactly as CRTP applies them on hardware.

Both are additive; a checkout without the out-of-tree controller still builds.

See [`docs/09_Simulation.md`](../../../docs/09_Simulation.md).
