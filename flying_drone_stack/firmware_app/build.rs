// build.rs — generates Rust FFI bindings from the Crazyflie C headers.
//
// This runs on the HOST (x86_64) at build time; the generated bindings.rs is
// then compiled into the Rust staticlib targeting thumbv7em-none-eabihf.
//
// Lecture slides requirement: "prepare bindings using bindgen".

use std::env;
use std::path::PathBuf;

fn main() {
    // CRAZYFLIE_BASE is exported by firmware_app/Makefile so that the
    // Kbuild → cargo pipeline can find the firmware headers.
    // Falls back to the absolute path for direct `cargo build` invocations.
    let fw_base = env::var("CRAZYFLIE_BASE")
        .unwrap_or_else(|_| "/home/georg/Desktop/crazyflie-firmware".to_string());

    // ── Drone platform selection ───────────────────────────────────────────
    // Selects the compile-time physical constants (arm, torque ratio, inertia) in lib.rs.
    // Default (env unset) = CF2.1 standard/upgraded → identical to previous behavior.
    // `DRONE_PLATFORM=bl` (set by `make DRONE=bl`) → Crazyflie 2.1 Brushless (CF21BL).
    println!("cargo:rerun-if-env-changed=DRONE_PLATFORM");
    println!("cargo:rustc-check-cfg=cfg(drone_bl)");
    match env::var("DRONE_PLATFORM").as_deref() {
        Ok("bl") | Ok("brushless") => println!("cargo:rustc-cfg=drone_bl"),
        _ => {} // cf2 (standard / upgraded) — the default, unchanged
    }

    // Re-run if any input changes.
    println!("cargo:rerun-if-changed=wrapper.h");
    println!("cargo:rerun-if-env-changed=CRAZYFLIE_BASE");
    println!("cargo:rerun-if-changed={}/src/modules/interface/stabilizer_types.h", fw_base);
    println!("cargo:rerun-if-changed={}/src/modules/interface/pptraj.h", fw_base);

    // libclang location (Ubuntu 22.04 ships clang-14 without a generic symlink).
    // bindgen needs this to find libclang.so.
    if env::var("LIBCLANG_PATH").is_err() {
        println!("cargo:rustc-env=LIBCLANG_PATH=/usr/lib/llvm-14/lib");
        // Also set for the current process so bindgen can use it immediately.
        unsafe { env::set_var("LIBCLANG_PATH", "/usr/lib/llvm-14/lib"); }
    }

    // 2026-09-30 LOCAL FIX -- CRITICAL, ARM-TARGET ONLY. Without this, bindgen's clang parses C
    // enums with host (x86_64) conventions -- 4-byte ints. The real firmware compiles with
    // arm-none-eabi-gcc, which packs plain enums (e.g. stab_mode_t / mode_e, used in
    // setpoint_s.mode.{x,y,z,...}) as 1-byte types by default on this ARM target. The resulting
    // struct-layout mismatch made every Rust read of setpoint.mode.* garbage -- confirmed root
    // cause of controller=10 (omar_indi_rust.rs) never leaving the position-control branch
    // across six real flight attempts (2026-09-29/30). See
    // docs/41_Pure_INDI_Implementation_Comparison.md §17-18 for the full diagnosis (byte-level
    // offsetof probe compiled with the real ARM toolchain flags, confirming sizeof(stab_mode_t)
    // ==1 and 1-byte field stride in the true compiled layout).
    // lib.rs (controller=6) never reads setpoint.mode.* so it was never affected by this;
    // controller_omar_indi.c (controller=9) reads it identically but is pure C, no FFI layout
    // involved. omar_indi_rust.rs is the first Rust code in this project to cross this boundary.
    //
    // MUST be gated on the ARM target only. The host x86_64 build links Rust against C sources
    // compiled by ordinary `gcc`, which -- unlike arm-none-eabi-gcc on this target -- does NOT
    // default to short enums. Applying -fshort-enums unconditionally was tried first and broke
    // the host/SIL build the opposite way (Rust assumes 1-byte, host C stays 4-byte) --
    // test_omar_indi_rust_vs_c.py went from 7/7 to 1/7 (rust thrust pinned at 0.0, same
    // fallback-branch symptom, just on the other side of the FFI boundary). DO NOT apply this
    // unconditionally; DO NOT revert it for the ARM target.
    let target = env::var("TARGET").unwrap_or_default();
    let is_arm_target = target.starts_with("thumbv7em");

    let mut builder = bindgen::Builder::default()
        .header("wrapper.h")
        // Firmware include paths
        .clang_arg(format!("-I{}/src/modules/interface", fw_base))
        .clang_arg(format!("-I{}/src/modules/interface/controller", fw_base))
        .clang_arg(format!("-I{}/src/hal/interface", fw_base))
        .clang_arg(format!("-I{}/src/utils/interface/lighthouse", fw_base));
    if is_arm_target {
        builder = builder.clang_arg("-fshort-enums");
    }
    let bindings = builder
        // Generate no_std-compatible code (core:: instead of std::)
        .use_core()
        .ctypes_prefix("core::ffi")
        // Disable layout tests — they require std and can't run on the target
        .layout_tests(false)
        // Only generate the types we actually use — keeps the output minimal
        .allowlist_type("control_s")
        .allowlist_type("control_t")
        .allowlist_type("control_mode_e")
        .allowlist_type("setpoint_s")
        .allowlist_type("setpoint_t")
        .allowlist_type("sensorData_s")
        .allowlist_type("sensorData_t")
        .allowlist_type("state_s")
        .allowlist_type("state_t")
        .allowlist_type("stabilizerStep_t")
        .allowlist_type("Axis3f")
        .allowlist_type("quaternion_s")
        .allowlist_type("quaternion_t")
        .allowlist_type("attitude_s")
        .allowlist_type("attitude_t")
        .allowlist_type("vec3_s")
        .allowlist_type("baro_s")
        .allowlist_type("baro_t")
        .derive_default(true)
        .generate()
        .expect("bindgen failed to generate bindings from wrapper.h");

    let out_path = PathBuf::from(env::var("OUT_DIR").unwrap());
    bindings
        .write_to_file(out_path.join("bindings.rs"))
        .expect("Could not write bindings.rs");

    // Flash-resident weights (`residual_nn_flash`): embed a trained .npz at compile time.
    println!("cargo:rerun-if-env-changed=CF_RNN_WEIGHTS_NPZ");
    println!("cargo:rerun-if-changed=../tools/residual/export_weights_rs.py");
    if env::var("CARGO_FEATURE_RESIDUAL_NN_FLASH").is_ok() {
        let npz = env::var("CF_RNN_WEIGHTS_NPZ").unwrap_or_else(|_| {
            panic!(
                "residual_nn_flash requires CF_RNN_WEIGHTS_NPZ pointing at a trained .npz \
                 (19297 float32 weights)"
            );
        });
        let out_rs = out_path.join("rnn_weights_embedded.rs");
        let manifest_dir = PathBuf::from(env::var("CARGO_MANIFEST_DIR").unwrap());
        let exporter = manifest_dir.join("../tools/residual/export_weights_rs.py");
        let status = std::process::Command::new("python3")
            .arg(&exporter)
            .arg(&npz)
            .arg(&out_rs)
            .status()
            .unwrap_or_else(|e| panic!("failed to run export_weights_rs.py: {e}"));
        if !status.success() {
            panic!("export_weights_rs.py failed for {npz}");
        }
        println!("cargo:rerun-if-changed={npz}");
    }
}
