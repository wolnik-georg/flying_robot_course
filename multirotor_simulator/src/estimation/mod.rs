//! State estimation module.
//!
//! Currently provides the Multiplicative Extended Kalman Filter (MEKF).

pub mod mekf;

pub use mekf::{Mekf, MekfState, MekfParams, quat_to_euler};
