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
pub mod flight;
pub mod perception;
pub mod mapping;
pub mod flight_common;

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
        FlatOutput, FlatnessResult, body_torque_diagonal, compute_flatness, flatness_to_reference,
        rot_to_quat,
        SplineTrajectory, SplineSegment, Waypoint,
        RichterTrajectory, RichterWaypoint, FeasibilityReport,
        TrajectoryPlanner,
        Se3Trajectory, Se3Waypoint, Se3Output,
    };
    pub use crate::safety::{SafetyLimits, check_safety};
}
