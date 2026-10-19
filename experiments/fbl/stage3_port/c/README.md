# FBL residual in C (host-tested, NOT in the firmware tree)

`fbl_residual.h/.c` — the downwash residual of the learned-FBL port (ROM lookup table, gate, fade, acceleration and jerk, 1–3 neighbours, peer-velocity differencing with the NS2 rules). Spec: `docs/fbl/PORT_SPEC.md`. Table: generated `../../out/stage3/rom_table_v1/fbl_rom_table.c/.h` (`../build_rom_tables.py`).

    make test      # generates test vectors from the Python reference (fbl_rom_runtime.py), builds, runs: expected "RESULT: ALL PASSED"
    make clean

Tolerance (proposal): acceleration 1e-5 + 1e-5·|ref|, jerk 1e-4 + 1e-5·|ref|. Also compiles for the target: `arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard -O2 -I. -I<table dir> -c fbl_residual.c`.
