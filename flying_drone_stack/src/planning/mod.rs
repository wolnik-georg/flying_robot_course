//! Motion planning module.
//!
//! Provides:
//! - Minimum-snap 8th-order polynomial spline planner (`spline`)
//! - Differential flatness for multirotors (`flatness`)

pub mod flatness;
pub mod spline;
pub mod exploration;

pub use flatness::{FlatOutput, FlatOutput as FlatState, FlatnessResult, compute_flatness, rot_to_quat, flatness_to_reference};
pub use spline::{SplineSegment, SplineTrajectory, Waypoint};
