//! CRTP log adapters — unit-conversion wrappers for data decoded by the
//! Crazyflie CRTP log reader in `src/bin/main.rs`.
//!
//! These are pure conversion functions (no runtime state, no I/O).  Making
//! the unit conventions explicit here means they are testable in isolation and
//! the caller never needs to remember the raw formats.
//!
//! ## CRTP variable formats
//!
//! | Variable           | Type   | Unit   | Adapter field   |
//! |--------------------|--------|--------|-----------------|
//! | `motion.deltaX`    | int16  | px     | `dx_raw`        |
//! | `motion.deltaY`    | int16  | px     | `dy_raw`        |
//! | `range.zrange`     | uint16 | mm     | `range_mm`      |
//! | `range.front`      | uint16 | mm     | `front_mm`      |
//! | `range.back`       | uint16 | mm     | `back_mm`       |
//! | `range.left`       | uint16 | mm     | `left_mm`       |
//! | `range.right`      | uint16 | mm     | `right_mm`      |
//! | `range.up`         | uint16 | mm     | `up_mm`         |
//!
//! Out-of-range sentinel for all Multi-ranger variables: `65535` (`u16::MAX`).

use crate::perception::types::{FlowMeasurement, RangeMeasurement, MultiRangeMeasurement};

// VL53L1x range limit in mm (out-of-range sentinel from Multi-ranger firmware).
const OUT_OF_RANGE_MM: u16 = u16::MAX;
const RANGE_MAX_M: f32 = 4.0;

// ---------------------------------------------------------------------------
// CrtpFlowAdapter
// ---------------------------------------------------------------------------

/// Converts raw CRTP flow log variables to a `FlowMeasurement`.
pub struct CrtpFlowAdapter;

impl CrtpFlowAdapter {
    /// Convert raw CRTP `motion.deltaX` / `motion.deltaY` to a measurement.
    ///
    /// - `dx_raw`: `motion.deltaX` (int16, accumulated pixels since last read)
    /// - `dy_raw`: `motion.deltaY` (int16, accumulated pixels since last read)
    /// - `dt_s`:   time elapsed since last flow log packet [s]
    ///
    /// The values are passed through as-is; the PMW3901 axis ↔ body-frame
    /// remapping (`vel_x ↔ −flow_dy`, `vel_y ↔ −flow_dx`) is handled in the
    /// MEKF update, not here.
    pub fn from_log_row(dx_raw: f32, dy_raw: f32, dt_s: f32) -> FlowMeasurement {
        FlowMeasurement { dx_px: dx_raw, dy_px: dy_raw, dt_s }
    }
}

// ---------------------------------------------------------------------------
// CrtpRangeAdapter
// ---------------------------------------------------------------------------

/// Converts the CRTP `range.zrange` log variable to a `RangeMeasurement`.
pub struct CrtpRangeAdapter;

impl CrtpRangeAdapter {
    /// Convert `range.zrange` from mm (uint16) to metres.
    ///
    /// Returns `None` for zero values (sensor not ready) and for values
    /// above `RANGE_MAX_M` (4 m).
    pub fn from_log_row(range_mm: f32) -> Option<RangeMeasurement> {
        if range_mm <= 0.0 { return None; }
        let range_m = range_mm / 1000.0;
        if range_m > RANGE_MAX_M { return None; }
        Some(RangeMeasurement { range_m })
    }
}

// ---------------------------------------------------------------------------
// CrtpMultiRangeAdapter
// ---------------------------------------------------------------------------

/// Converts CRTP Multi-ranger Deck log variables to a `MultiRangeMeasurement`.
///
/// CRTP variable names (all uint16, mm):
///   `range.front`, `range.back`, `range.left`, `range.right`, `range.up`
///
/// Down channel: `range.zrange` from Flow Deck (passed as `down_mm`).
///
/// Out-of-range sentinel: `u16::MAX` (65535) → maps to `None`.
pub struct CrtpMultiRangeAdapter;

impl CrtpMultiRangeAdapter {
    /// Convert raw u16 mm values from the CRTP log to `MultiRangeMeasurement`.
    pub fn from_log_row(
        front_mm: u16,
        back_mm:  u16,
        left_mm:  u16,
        right_mm: u16,
        up_mm:    u16,
        down_mm:  u16,
    ) -> MultiRangeMeasurement {
        MultiRangeMeasurement {
            front_m: mm_to_metres(front_mm),
            back_m:  mm_to_metres(back_mm),
            left_m:  mm_to_metres(left_mm),
            right_m: mm_to_metres(right_mm),
            up_m:    mm_to_metres(up_mm),
            down_m:  mm_to_metres(down_mm),
        }
    }
}

/// Convert a raw u16 mm reading to `Option<f32>` metres.
/// Returns `None` for the out-of-range sentinel (u16::MAX) or values > 4 m.
#[inline]
fn mm_to_metres(mm: u16) -> Option<f32> {
    if mm == OUT_OF_RANGE_MM { return None; }
    let m = mm as f32 / 1000.0;
    if m > RANGE_MAX_M { None } else { Some(m) }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn crtp_flow_passthrough() {
        let m = CrtpFlowAdapter::from_log_row(3.5, -1.2, 0.01);
        assert_eq!(m.dx_px, 3.5);
        assert_eq!(m.dy_px, -1.2);
        assert_eq!(m.dt_s,  0.01);
    }

    #[test]
    fn crtp_range_mm_to_metres() {
        let m = CrtpRangeAdapter::from_log_row(500.0).unwrap();
        assert!((m.range_m - 0.5).abs() < 1e-5);
    }

    #[test]
    fn crtp_range_zero_is_none() {
        assert!(CrtpRangeAdapter::from_log_row(0.0).is_none());
    }

    #[test]
    fn crtp_range_overflow_is_none() {
        // 4001 mm > 4.0 m limit
        assert!(CrtpRangeAdapter::from_log_row(4001.0).is_none());
    }

    #[test]
    fn crtp_multi_range_sentinel() {
        // u16::MAX (65535) is the out-of-range sentinel and must map to None.
        let m = CrtpMultiRangeAdapter::from_log_row(
            u16::MAX, 1000, 2000, 1500, u16::MAX, 300,
        );
        assert!(m.front_m.is_none(), "front sentinel → None");
        assert!(m.up_m.is_none(),    "up sentinel → None");

        // Valid readings convert correctly.
        assert!((m.back_m.unwrap()  - 1.0).abs() < 1e-4, "back 1000 mm = 1.0 m");
        assert!((m.left_m.unwrap()  - 2.0).abs() < 1e-4, "left 2000 mm = 2.0 m");
        assert!((m.right_m.unwrap() - 1.5).abs() < 1e-4, "right 1500 mm = 1.5 m");
        assert!((m.down_m.unwrap()  - 0.3).abs() < 1e-4, "down 300 mm = 0.3 m");
    }

    #[test]
    fn crtp_multi_range_over_4m_is_none() {
        // 4001 mm converts to 4.001 m which exceeds RANGE_MAX_M.
        let m = CrtpMultiRangeAdapter::from_log_row(4001, 4001, 4001, 4001, 4001, 4001);
        assert!(m.front_m.is_none());
        assert!(m.back_m.is_none());
        assert!(m.left_m.is_none());
        assert!(m.right_m.is_none());
        assert!(m.up_m.is_none());
        assert!(m.down_m.is_none());
    }
}
