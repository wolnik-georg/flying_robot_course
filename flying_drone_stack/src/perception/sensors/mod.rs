//! Hardware-abstracted sensor implementations.
//!
//! Three families:
//! - `sim`:  driven from `MultirotorState`; no hardware required.
//! - `crtp`: unit-conversion wrappers for data decoded by the CRTP log reader.
//! - `cpx`:  async AI Deck JPEG frame receiver over TCP / CPX protocol.

pub mod sim;
pub mod crtp;
pub mod cpx;
