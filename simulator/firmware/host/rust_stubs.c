/* Stubs for the symbols that traj_iface.c expects from the Rust static library (firmware_app/src/lib.rs: residual-network upload).
   The simulator never calls the Rust controllers or the upload path; these definitions only let the extension load without libcf_controller_rs.a.
   Used when OOT_WITH_RUST is not 1 (build_bindings.sh default). */
#include <stdint.h>
void cf_rnn_begin_upload(uint16_t expected) { (void)expected; }
uint8_t cf_rnn_set_weight(uint16_t idx, float value) { (void)idx; (void)value; return 0; }
uint8_t cf_rnn_finish_upload(void) { return 0; }

/* Everything else the SWIG wrapper (bindings/cffirmware.i) references from the Rust controllers (OOT slots, NA-INDI test hooks, state pointers).
   None of these is called by the simulator (it runs controllerOmarIndi, the C Omar tree / FBL); the definitions only satisfy the dynamic loader.
   Stubs return 0. The list was produced from `nm -D --undefined-only` of a build without the Rust library. */
int controllerOutOfTree() { return 0; }
int controllerOutOfTreeInit() { return 0; }
int controllerOutOfTreeTest() { return 0; }
int controllerOutOfTree2() { return 0; }
int controllerOutOfTree2Init() { return 0; }
int controllerOutOfTree2Test() { return 0; }
int controllerOutOfTree3() { return 0; }
int controllerOutOfTree3Init() { return 0; }
int controllerOutOfTree3Test() { return 0; }
int controllerOutOfTree5() { return 0; }
int controllerOutOfTree5Init() { return 0; }
int controllerOutOfTree5Test() { return 0; }
int controllerOutOfTree6() { return 0; }
int controllerOutOfTree6Init() { return 0; }
int controllerOutOfTree6Test() { return 0; }
int naindi_hybrid_test_nn_forward() { return 0; }
int naindi_hybrid_test_set_arm() { return 0; }
int naindi_hybrid_test_set_gains() { return 0; }
int naindi_hybrid_test_set_j() { return 0; }
int naindi_test_set_arm() { return 0; }
int naindi_test_set_gains() { return 0; }
int naindi_test_set_j() { return 0; }
int naindi_test_set_pos_gains() { return 0; }
int omar_indi_rust_set_indi() { return 0; }
int oot_inertia() { return 0; }
int oot_state_ptr() { return 0; }
int oot2_state_ptr() { return 0; }
int oot3_state_ptr() { return 0; }
int oot5_state_ptr() { return 0; }
int oot6_state_ptr() { return 0; }
int oot_state_size() { return 0; }
int oot2_state_size() { return 0; }
int oot3_state_size() { return 0; }
int oot5_state_size() { return 0; }
int oot6_state_size() { return 0; }
