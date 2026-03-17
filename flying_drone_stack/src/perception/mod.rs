//! Perception module for flying_drone_stack.
//!
//! Provides a hardware-abstracted sensor layer that is interchangeable between
//! simulation and real hardware (Flow Deck v2, Multi-ranger Deck, AI Deck).
//!
//! ## Quick start
//!
//! **Simulation:**
//! ```rust
//! use multirotor_simulator::perception::sensors::sim::{SimFlowSensor, SimRangeSensor};
//! use multirotor_simulator::perception::types::FlowMeasurement;
//!
//! let mut flow = SimFlowSensor::new(0.01, 0.05);
//! let mut range = SimRangeSensor::new(0.002);
//! // In the control loop:
//! // let f = flow.measure(&sim.state());
//! // let r = range.measure(&sim.state());
//! ```
//!
//! **Feature detection:**
//! ```rust
//! use multirotor_simulator::perception::processing::features::detect_features;
//! // let features = detect_features(&image_frame, 20);
//! ```
//!
//! ## Architecture
//!
//! ```text
//!    [ Simulation ]              [ Real Hardware ]
//!   MultirotorState              Crazyflie CRTP log
//!        |                              |
//!   sensors/sim.rs              sensors/crtp.rs
//!        |                              |
//!        |                       AI Deck (CPX WiFi)
//!        |                              |
//!        |                      sensors/cpx.rs
//!        \_____ traits.rs ______________|
//!                    |
//!        SensorSource<FlowMeasurement>
//!        SensorSource<RangeMeasurement>
//!        ImageSource → ImageFrame
//!                    |
//!           processing/ pipeline
//!                    |
//!          Vec<Feature> (for SLAM)
//! ```

pub mod types;
pub mod traits;
pub mod sensors;
pub mod processing;

// Convenience re-exports for the most commonly used types.
pub use types::{
    FlowMeasurement, RangeMeasurement, MultiRangeMeasurement,
    CameraIntrinsics, ImageFrame, Feature,
};
pub use traits::{SensorSource, ImageSource};
