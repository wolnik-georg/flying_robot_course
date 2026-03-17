//! Sensor abstraction traits for the perception module.
//!
//! Both traits are synchronous (no async) to mirror the rest of the codebase.
//! Real hardware adapters get pre-decoded data passed in from the CRTP / CPX
//! receive loops and forward it through these traits.

use crate::perception::types::ImageFrame;

/// Generic sensor that produces measurements on demand.
///
/// `M` is the measurement type (e.g. `FlowMeasurement`, `RangeMeasurement`).
///
/// Returns `None` when no new measurement is available — either because:
/// - The hardware adapter has not received a new packet yet, or
/// - The simulation sensor was polled faster than its configured rate.
///
/// Implementations are not required to buffer: calling `poll()` twice before
/// a new sample arrives is allowed to return `None` on the second call.
pub trait SensorSource<M> {
    fn poll(&mut self) -> Option<M>;
}

/// A source that produces complete image frames.
///
/// Returns `None` when no new frame has arrived since the last call.
/// Implementations should drop stale frames (return only the latest).
pub trait ImageSource {
    fn next_frame(&mut self) -> Option<ImageFrame>;
}
