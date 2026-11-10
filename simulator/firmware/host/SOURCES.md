# Host C files linked into the simulator bindings (copied unchanged, 2026-11-10)

Source repo: flying_robot_course, HEAD 846fee17 (last commit touching these files: 3ad1d1d4).

| file here | original path (repo-relative) | sha256 |
|---|---|---|
| traj_iface.c | flying_drone_stack/firmware_app/traj_iface.c | 11083226d6072f51ed5c4779f5e302f36bd22759bdc40c8351d1833bde67ffb6 |
| ns2_weights.c | flying_drone_stack/firmware_app/ns2_weights.c | fcadf6ef2b1b442361bcdb68f7205265909783a23f5f366b8ee2a0c15da118d4 |
| rpm_filter.h | flying_drone_stack/firmware_app/rpm_filter.h | f1c4781d43779fb4cbb9d6aa8cc9f0018dbecd8235ae2b29f2096b12b4009b38 |
| oot_host.c | flying_drone_stack/firmware_app/host/oot_host.c | cd729f3e4677fafdf5220d31704a8e3f767ae810e954733bdd17fd264b1616e1 |
| rust_stubs.c | new (stubs for cf_rnn_begin_upload / cf_rnn_set_weight / cf_rnn_finish_upload; not in the original build, where libcf_controller_rs.a provides them) | - |

traj_iface.c, oot_host.c and ns2_weights.c are byte-identical to the copies in ~/Desktop/fbl_v2_firmware/stack/firmware_app that the FBL bindings were built from (checked 2026-11-10).
With OOT_WITH_RUST=1 the Rust library libcf_controller_rs.a is linked instead of rust_stubs.c (build: cd flying_drone_stack/firmware_app && RUSTFLAGS="-C panic=abort" cargo build --release --target x86_64-unknown-linux-gnu).
