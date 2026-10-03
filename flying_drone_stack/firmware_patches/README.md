# crazyflie-firmware local patches

Apply on a clean **bitcraze/crazyflie-firmware** checkout (this tree pins `stable-june20`).

Order matters for overlapping files — use:

```bash
FW=~/Desktop/crazyflie-firmware   # or a throw-away worktree
PATCH=$HOME/Desktop/flying_robot_course/flying_drone_stack/firmware_patches
cd "$FW"
for p in \
  stabilizer_stack_8x.patch \
  usddeck_56_variables.patch \
  controller_indi_cf21bl.patch \
  controller_oot_slots.patch \
  controller_omar_indi.h.patch \
  controller_omar_indi.c.patch \
  naindi_gyro_no_lpf_math3d.patch \
  platform_cf21bl_omar_constants.patch \
  rpm_deck_bcRpm.patch \
  cffirmware_bindings.patch
do
  git apply --check "$PATCH/$p" && git apply "$PATCH/$p"
done
make bindings_python
```

Regenerate from the live firmware tree:

```bash
cd ~/Desktop/crazyflie-firmware
# tracked files — same paths as in README loop above
git diff src/config/config.h > "$PATCH/stabilizer_stack_8x.patch"
# … see LOCAL_MODIFICATIONS.md § How to restore everything
```
