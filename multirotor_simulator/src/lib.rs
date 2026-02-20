//! Multirotor Dynamics Simulator
//!
//! A modular, well-architected library for simulating multirotor dynamics
//! with pluggable integration methods and comprehensive validation tools.

pub mod math;
pub mod dynamics;
pub mod integration;
pub mod controller;
pub mod trajectory;

/// Prelude for convenient imports
pub mod prelude {
    pub use crate::math::{Vec3, Quat};
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
        CircleTrajectory,
        CsvTrajectory,
    };
}
