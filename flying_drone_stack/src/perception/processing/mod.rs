//! Image and sensor data processing for the perception module.
//!
//! Three sub-modules:
//! - `features`:    FAST-9 corner detection + BRIEF-256 descriptor.
//! - `calibration`: Pinhole camera model, lens distortion, project / unproject.
//! - `sync`:        IMU–camera temporal alignment via linear interpolation.

pub mod features;
pub mod calibration;
pub mod sync;
