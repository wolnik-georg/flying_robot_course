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

    // Re-run if any input changes.
    println!("cargo:rerun-if-changed=wrapper.h");
    println!("cargo:rerun-if-env-changed=CRAZYFLIE_BASE");

    // libclang location (Ubuntu 22.04 ships clang-14 without a generic symlink).
    // bindgen needs this to find libclang.so.
    if env::var("LIBCLANG_PATH").is_err() {
        println!("cargo:rustc-env=LIBCLANG_PATH=/usr/lib/llvm-14/lib");
        // Also set for the current process so bindgen can use it immediately.
        unsafe { env::set_var("LIBCLANG_PATH", "/usr/lib/llvm-14/lib"); }
    }

    let bindings = bindgen::Builder::default()
        .header("wrapper.h")
        // Firmware include paths
        .clang_arg(format!("-I{}/src/modules/interface", fw_base))
        .clang_arg(format!("-I{}/src/modules/interface/controller", fw_base))
        .clang_arg(format!("-I{}/src/hal/interface", fw_base))
        .clang_arg(format!("-I{}/src/utils/interface/lighthouse", fw_base))
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
}
