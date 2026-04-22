Rebuild the OOT firmware and flash it to the Crazyflie via Crazyradio.

Steps:
1. Build: `cd firmware_app && make`
2. Verify Rust symbols are present:
   `arm-none-eabi-nm firmware_app/target/thumbv7em-none-eabihf/release/libcf_controller_rs.a | grep controllerOutOfTree`
3. Flash: `cd firmware_app && make cload`

If step 3 fails (radio not connected), show the manual flash command:
```
cfloader flash /home/georg/Desktop/flying_robot_course/flying_drone_stack/firmware_app/build/cf2.bin stm32-fw -w radio://0/80/2M/E7E7E7E7E7
```

After flashing, remind that Mode B binaries (`spline_circle_test`, `spline_figure8_test`) and
Mode C HLC Python scripts both require this firmware.
