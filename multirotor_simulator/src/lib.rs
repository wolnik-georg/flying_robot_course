//! Multirotor Dynamics Simulator
//!
//! A modular, well-architected library for simulating multirotor dynamics
//! with pluggable integration methods and comprehensive validation tools.
//! Multirotor Dynamics Simulator
//!
//! A modular, well-architected library for simulating multirotor dynamics
//! with pluggable integration methods and comprehensive validation tools.
pub mod safety;

pub mod math;
pub mod dynamics;
pub mod integration;
pub mod controller;
pub mod trajectory;
pub mod estimation;
pub mod planning;

/// Prelude for convenient imports
pub mod prelude {
    pub use crate::math::{Vec3, Quat, to_euler};
    pub use crate::dynamics::{
        MultirotorState,
        MultirotorParams,
        MultirotorSimulator,
        MotorAction,
        Integrator,
    };
    pub use crate::integration::{
        EulerIntegrator,
        RK4Integrator,
        ExpEulerIntegrator,
        ExpRK4Integrator,
    };
    pub use crate::controller::{
        Controller,
        GeometricController,
        TrajectoryReference,
        ControlOutput,
    };
    pub use crate::trajectory::{
        Trajectory,
        Figure8Trajectory,
        SmoothFigure8Trajectory,
        CircleTrajectory,
        CsvTrajectory,
        TakeoffTrajectory,
        SequencedTrajectory,
    };
    pub use crate::estimation::{Mekf, MekfState, MekfParams};
    pub use crate::planning::{
        FlatOutput, FlatnessResult, compute_flatness, rot_to_quat, flatness_to_reference,
        SplineTrajectory, SplineSegment, Waypoint,
    };
    pub use crate::safety::{SafetyLimits, check_safety};
}
