//! Flight-layer helpers: pure functions used by `main.rs` control loops.
//!
//! These are extracted from `main.rs` so they can be unit-tested in isolation
//! and to keep the top-level control loops as concise orchestration code.
//!
//! Three sub-modules:
//!
//! * [`state_builder`] — build a [`MultirotorState`] from raw EKF log scalars
//!   (handles the degree-to-radian conversions and quaternion construction).
//!
//! * [`rpyt_control`] — all math required to turn the geometric-controller output
//!   into a `(roll_cmd, pitch_cmd, yaw_rate_cmd, thrust_pwm)` tuple that is safe
//!   to hand to `cf.commander.setpoint_rpyt(...)`.  Covers f-vec decomposition,
//!   anti-windup logic, and PWM scaling.
//!
//! * [`ekf_reset`] — lightweight EKF-jump detector and yaw-unwrap helper.

pub mod state_builder;
pub mod rpyt_control;
pub mod ekf_reset;

pub use state_builder::build_state;
pub use rpyt_control::{
    compute_force_vector,
    force_vector_to_rpyt,
    thrust_to_pwm,
    yaw_rate_cmd,
    RpytCmd,
};
pub use ekf_reset::{detect_ekf_reset, yaw_wrap_delta, deg_to_rad, EkfResetFlags};
