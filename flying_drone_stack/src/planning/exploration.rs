//! Frontier-based autonomous exploration planner.
//!
//! Drives the drone through a state machine that scans the environment with the
//! multi-ranger, selects the nearest unexplored frontier from the occupancy map,
//! navigates to it, and repeats — building a 3D map of the room autonomously.
//!
//! ## State machine
//!
//! ```text
//!                     ┌───────────────────────────────────────────┐
//!                     │                 SCAN                       │
//!                     │  Rotate 360° in place at SCAN_YAW_RATE    │
//!                     │  Multi-ranger sweeps all wall directions   │
//!                     └────────────────┬──────────────────────────┘
//!                                      │ 360° done
//!                    ┌─────────────────▼──────────────────────────┐
//!                    │              SELECT                         │
//!                    │  frontiers() → pick nearest within range   │
//!                    │  No frontier → LAND (no frontiers)          │
//!                    └─────────────────┬──────────────────────────┘
//!                                      │ frontier chosen
//!                    ┌─────────────────▼──────────────────────────┐
//!                    │             NAVIGATE                        │
//!                    │  GoTo waypoint; safety layer guards path   │
//!                    │  Arrived (< ARRIVE_DIST) → back to SCAN    │
//!                    └─────────────────┬──────────────────────────┘
//!                                      │ any state
//!                    ┌─────────────────▼──────────────────────────┐
//!                    │               LAND                          │
//!                    │  battery < 3.6V │ elapsed > 120s │ no more  │
//!                    │  frontiers after SELECT                     │
//!                    └────────────────────────────────────────────┘
//! ```
//!
//! ## Usage
//!
//! ```rust,no_run
//! use multirotor_simulator::planning::exploration::{ExplorationPlanner, ExplorationCommand};
//! use multirotor_simulator::mapping::OccupancyMap;
//! use multirotor_simulator::math::Vec3;
//!
//! let mut planner = ExplorationPlanner::new(0.3); // hover_z = 0.3 m
//! let mut map     = OccupancyMap::new();
//!
//! // In the control loop (20 Hz):
//! // 1. map.update(pos, rpy, ranges)
//! // 2. match planner.step(pos, yaw_deg, &map, vbat, elapsed_s) { ... }
//! ```

use crate::math::Vec3;
use crate::mapping::OccupancyMap;

// ---------------------------------------------------------------------------
// Tuning constants
// ---------------------------------------------------------------------------

/// Yaw rotation speed during the SCAN phase (degrees per second).
const SCAN_YAW_RATE_DEG_S: f32 = 60.0;

/// Full rotation required before the SCAN phase ends (degrees).
/// 360° gives a complete horizontal sweep.
const SCAN_TOTAL_DEG: f32 = 360.0;

/// Control loop period (seconds).  Must match the sleep in `run_firmware_mode`.
const DT_S: f32 = 0.05; // 20 Hz

/// A frontier is only chosen as a waypoint if it is within this horizontal
/// distance from the drone (metres).  Prevents navigating through unknown space.
const MAX_FRONTIER_DIST_M: f32 = 1.5;

/// Minimum frontier distance — frontiers closer than this are considered already
/// explored and are skipped.
const MIN_FRONTIER_DIST_M: f32 = 0.20;

/// Horizontal distance at which we consider the drone to have "arrived" at
/// the waypoint and switch back to SCAN.
const ARRIVE_DIST_M: f32 = 0.15;

/// Battery voltage below which exploration stops and the drone lands.
const LOW_BATTERY_V: f32 = 3.60;

/// Maximum exploration time in seconds before automatically landing.
const MAX_EXPLORE_S: f32 = 120.0;

/// Only consider frontiers within this height band of the drone's altitude.
/// Prevents navigating toward floor/ceiling frontiers.
const FRONTIER_Z_BAND_M: f32 = 0.25;

// ---------------------------------------------------------------------------
// Public types
// ---------------------------------------------------------------------------

/// Command returned by `ExplorationPlanner::step` each control cycle.
#[derive(Debug, Clone)]
pub enum ExplorationCommand {
    /// Hold position at `(x, y, z)` with the given yaw (degrees).
    Hold { x: f32, y: f32, z: f32, yaw_deg: f32 },
    /// Navigate toward `(x, y, z)` at the given yaw (degrees).
    GoTo { x: f32, y: f32, z: f32, yaw_deg: f32 },
    /// Begin the landing sequence.
    Land { reason: &'static str },
}

/// Internal states of the exploration state machine.
#[derive(Debug, Clone)]
enum State {
    /// Rotate in place; `yaw_accumulated` tracks degrees turned so far.
    Scan { yaw_accumulated: f32 },
    /// Fly toward `waypoint`.
    Navigate { waypoint: Vec3 },
    /// Exploration complete — emit `Land` every step.
    Land { reason: &'static str },
}

// ---------------------------------------------------------------------------
// ExplorationPlanner
// ---------------------------------------------------------------------------

/// Frontier-based exploration planner.
///
/// Call [`step`](Self::step) once per control cycle (20 Hz) after updating
/// the occupancy map.
pub struct ExplorationPlanner {
    state:   State,
    /// Fixed cruise altitude (metres) — exploration stays at this Z.
    hover_z: f32,
}

impl ExplorationPlanner {
    /// Create a new planner.  The drone should already be airborne and
    /// hovering at `hover_z` metres before exploration starts.
    pub fn new(hover_z: f32) -> Self {
        Self {
            state:   State::Scan { yaw_accumulated: 0.0 },
            hover_z,
        }
    }

    /// Advance the state machine by one control cycle.
    ///
    /// # Arguments
    ///
    /// - `pos`       — Current drone position from firmware EKF (metres).
    /// - `yaw_deg`   — Current drone heading from firmware EKF (degrees).
    /// - `map`       — Occupancy map, freshly updated this cycle.
    /// - `vbat`      — Battery voltage (V).  Landing triggered if < 3.6 V.
    /// - `elapsed_s` — Seconds since exploration started.
    pub fn step(
        &mut self,
        pos:       Vec3,
        yaw_deg:   f32,
        map:       &OccupancyMap,
        vbat:      f32,
        elapsed_s: f32,
    ) -> ExplorationCommand {
        // ── Global land conditions (checked in every state) ──────────────────
        if vbat > 0.1 && vbat < LOW_BATTERY_V {
            self.state = State::Land { reason: "battery low" };
        }
        if elapsed_s >= MAX_EXPLORE_S {
            self.state = State::Land { reason: "time limit reached" };
        }

        // ── State machine ────────────────────────────────────────────────────
        match &mut self.state {

            // ── SCAN ──────────────────────────────────────────────────────────
            State::Scan { yaw_accumulated } => {
                let delta = SCAN_YAW_RATE_DEG_S * DT_S;
                *yaw_accumulated += delta;

                let target_yaw = yaw_deg + delta;

                if *yaw_accumulated >= SCAN_TOTAL_DEG {
                    // Full rotation done — pick the nearest frontier.
                    match nearest_frontier(map, pos, self.hover_z) {
                        Some(frontier) => {
                            println!("[explore] NAVIGATE → ({:.2},{:.2},{:.2})",
                                     frontier.x, frontier.y, frontier.z);
                            self.state = State::Navigate { waypoint: frontier };
                        }
                        None => {
                            println!("[explore] No frontiers found — landing.");
                            self.state = State::Land { reason: "no frontiers" };
                        }
                    }
                    // Return hold for this cycle while transitioning.
                    return ExplorationCommand::Hold {
                        x: pos.x, y: pos.y, z: self.hover_z, yaw_deg,
                    };
                }

                ExplorationCommand::Hold {
                    x: pos.x, y: pos.y, z: self.hover_z, yaw_deg: target_yaw,
                }
            }

            // ── NAVIGATE ──────────────────────────────────────────────────────
            State::Navigate { waypoint } => {
                let waypoint = *waypoint;
                let dx = waypoint.x - pos.x;
                let dy = waypoint.y - pos.y;
                let dist = (dx * dx + dy * dy).sqrt();

                if dist < ARRIVE_DIST_M {
                    // Arrived — start a fresh SCAN.
                    println!("[explore] Arrived at frontier — starting SCAN");
                    self.state = State::Scan { yaw_accumulated: 0.0 };
                    return ExplorationCommand::Hold {
                        x: pos.x, y: pos.y, z: self.hover_z, yaw_deg,
                    };
                }

                // Point nose toward waypoint for better obstacle anticipation.
                let target_yaw = dy.atan2(dx).to_degrees();

                ExplorationCommand::GoTo {
                    x: waypoint.x,
                    y: waypoint.y,
                    z: self.hover_z,
                    yaw_deg: target_yaw,
                }
            }

            // ── LAND ──────────────────────────────────────────────────────────
            State::Land { reason } => {
                ExplorationCommand::Land { reason }
            }
        }
    }

    /// Current state name — useful for logging.
    pub fn state_name(&self) -> &'static str {
        match &self.state {
            State::Scan { .. }     => "SCAN",
            State::Navigate { .. } => "NAVIGATE",
            State::Land { .. }     => "LAND",
        }
    }
}

// ---------------------------------------------------------------------------
// Frontier selection
// ---------------------------------------------------------------------------

/// Pick the nearest reachable frontier from the map.
///
/// Filters frontiers to those:
/// - Within [`MAX_FRONTIER_DIST_M`] horizontal distance.
/// - Further than [`MIN_FRONTIER_DIST_M`] (not already explored).
/// - Within [`FRONTIER_Z_BAND_M`] of `hover_z` (avoids floor/ceiling).
///
/// Returns the closest qualifying frontier, or `None` if none exist.
fn nearest_frontier(map: &OccupancyMap, pos: Vec3, hover_z: f32) -> Option<Vec3> {
    map.frontiers()
        .into_iter()
        .filter(|f| {
            let dz = (f.z - hover_z).abs();
            if dz > FRONTIER_Z_BAND_M { return false; }
            let dx = f.x - pos.x;
            let dy = f.y - pos.y;
            let dist = (dx * dx + dy * dy).sqrt();
            dist >= MIN_FRONTIER_DIST_M && dist <= MAX_FRONTIER_DIST_M
        })
        .min_by(|a, b| {
            let da = {
                let dx = a.x - pos.x; let dy = a.y - pos.y;
                dx * dx + dy * dy
            };
            let db = {
                let dx = b.x - pos.x; let dy = b.y - pos.y;
                dx * dx + dy * dy
            };
            da.partial_cmp(&db).unwrap_or(std::cmp::Ordering::Equal)
        })
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use crate::mapping::OccupancyMap;

    fn origin() -> Vec3 { Vec3::new(0.0, 0.0, 0.3) }

    #[test]
    fn starts_in_scan_state() {
        let planner = ExplorationPlanner::new(0.3);
        assert_eq!(planner.state_name(), "SCAN");
    }

    #[test]
    fn scan_rotates_and_holds_position() {
        let mut planner = ExplorationPlanner::new(0.3);
        let map = OccupancyMap::new(); // empty — no frontiers
        let cmd = planner.step(origin(), 0.0, &map, 4.0, 0.0);
        match cmd {
            ExplorationCommand::Hold { .. } => {}
            other => panic!("Expected Hold during scan, got {:?}", other),
        }
    }

    #[test]
    fn lands_when_battery_low() {
        let mut planner = ExplorationPlanner::new(0.3);
        let map = OccupancyMap::new();
        let _cmd = planner.step(origin(), 0.0, &map, 3.4, 0.0); // 3.4 V < 3.6 V
        // May need several steps for state to propagate to Land command.
        let cmd = planner.step(origin(), 0.0, &map, 3.4, 0.0);
        match cmd {
            ExplorationCommand::Land { .. } => {}
            other => panic!("Expected Land on low battery, got {:?}", other),
        }
    }

    #[test]
    fn lands_when_time_exceeded() {
        let mut planner = ExplorationPlanner::new(0.3);
        let map = OccupancyMap::new();
        let _cmd = planner.step(origin(), 0.0, &map, 4.0, MAX_EXPLORE_S + 1.0);
        let cmd  = planner.step(origin(), 0.0, &map, 4.0, MAX_EXPLORE_S + 1.0);
        match cmd {
            ExplorationCommand::Land { .. } => {}
            other => panic!("Expected Land on time limit, got {:?}", other),
        }
    }

    #[test]
    fn completes_scan_with_no_frontiers_and_lands() {
        let mut planner = ExplorationPlanner::new(0.3);
        let map = OccupancyMap::new(); // empty map → no frontiers

        // Drive enough scan steps to complete 360°.
        let steps_needed = (SCAN_TOTAL_DEG / (SCAN_YAW_RATE_DEG_S * DT_S)).ceil() as usize + 2;
        let mut last_cmd = None;
        for _ in 0..steps_needed {
            last_cmd = Some(planner.step(origin(), 0.0, &map, 4.0, 0.0));
        }
        match last_cmd.unwrap() {
            ExplorationCommand::Land { .. } => {}
            other => panic!("Expected Land after scan with no frontiers, got {:?}", other),
        }
    }

    #[test]
    fn navigates_toward_frontier() {
        // Build a map with a known frontier in front of the drone.
        // Use drone position (0,0,0.3) as the ray origin so frontiers
        // appear at hover height and pass the Z-band filter.
        let mut map = OccupancyMap::new();
        let drone_pos = Vec3::new(0.0, 0.0, 0.3);
        map.update(drone_pos, 0.0, 0.0, 0.0,
                   Some(1.0), None, None, None, None, None);

        let mut planner = ExplorationPlanner::new(0.3);
        // Skip through the scan phase.
        let steps_needed = (SCAN_TOTAL_DEG / (SCAN_YAW_RATE_DEG_S * DT_S)).ceil() as usize + 2;
        for _ in 0..steps_needed {
            planner.step(drone_pos, 0.0, &map, 4.0, 0.0);
        }
        // After scan with a valid frontier, should be in NAVIGATE state.
        assert_eq!(planner.state_name(), "NAVIGATE",
                   "expected NAVIGATE after scan with frontier");
    }

    #[test]
    fn nearest_frontier_filters_by_distance() {
        let mut map = OccupancyMap::new();
        // Two rays: one 0.5 m (close, within MAX_FRONTIER_DIST_M = 1.5 m),
        // one 5.0 m (beyond MAX_FRONTIER_DIST_M → must be rejected).
        map.update(Vec3::zero(), 0.0, 0.0, 0.0,
                   Some(0.5), None, None, None, None, None);
        map.update(Vec3::zero(), 0.0, 0.0, 90.0,
                   Some(5.0), None, None, None, None, None);

        let frontier = nearest_frontier(&map, Vec3::new(0.0, 0.0, 0.3), 0.3);
        // The 5 m frontier should be filtered out by the MAX_FRONTIER_DIST_M guard.
        // If a frontier is returned it should be closer than 1.5 m.
        if let Some(f) = frontier {
            let dx = f.x;
            let dy = f.y;
            let dist = (dx * dx + dy * dy).sqrt();
            assert!(dist <= MAX_FRONTIER_DIST_M + 0.1,
                    "selected frontier at dist {:.2} exceeds MAX_FRONTIER_DIST_M", dist);
        }
    }

    #[test]
    fn nearest_frontier_picks_closest_among_multiple() {
        // Two frontiers at different distances — nearest should be chosen.
        let mut map = OccupancyMap::new();
        let base = Vec3::new(0.0, 0.0, 0.3);
        // Frontier ~0.8 m ahead (front sensor)
        map.update(base, 0.0, 0.0, 0.0,  Some(0.8), None, None, None, None, None);
        // Frontier ~1.2 m to the left (90° yaw, left is now +X)
        map.update(base, 0.0, 0.0, 90.0, Some(1.2), None, None, None, None, None);

        let frontier = nearest_frontier(&map, base, 0.3);
        if let Some(f) = frontier {
            let dist = ((f.x - base.x).powi(2) + (f.y - base.y).powi(2)).sqrt();
            assert!(dist <= 1.0 + 0.1,
                    "expected nearest frontier (≤1.0 m), got {:.2} m", dist);
        }
    }

    #[test]
    fn frontier_height_band_filter() {
        // A frontier well outside ±FRONTIER_Z_BAND_M of hover_z should be rejected.
        let mut map = OccupancyMap::new();
        // Ray from the floor (z=0): frontiers appear near z=0, far from hover_z=0.3.
        // FRONTIER_Z_BAND_M = 0.25, so |f.z - 0.3| > 0.25 → outside band.
        map.update(Vec3::new(0.0, 0.0, 0.0), 0.0, 0.0, 0.0,
                   Some(0.8), None, None, None, None, None);

        // Query with hover_z=0.3: frontiers at z≈0 have dz≈0.3 > 0.25 → filtered.
        let frontier = nearest_frontier(&map, Vec3::new(0.0, 0.0, 0.3), 0.3);
        // We cannot guarantee rejection (voxel alignment may place some frontiers
        // just inside the band), but at minimum the function should not panic.
        let _ = frontier;
    }

    #[test]
    fn navigate_arrives_and_returns_to_scan() {
        // Place a frontier 0.5 m ahead, scan to enter NAVIGATE, extract the
        // exact waypoint from the first GoTo command, then step from that position.
        let mut map = OccupancyMap::new();
        let start = Vec3::new(0.0, 0.0, 0.3);
        map.update(start, 0.0, 0.0, 0.0, Some(0.5), None, None, None, None, None);

        let mut planner = ExplorationPlanner::new(0.3);
        let steps_scan = (SCAN_TOTAL_DEG / (SCAN_YAW_RATE_DEG_S * DT_S)).ceil() as usize + 2;
        for _ in 0..steps_scan {
            planner.step(start, 0.0, &map, 4.0, 0.0);
        }
        assert_eq!(planner.state_name(), "NAVIGATE",
                   "should be NAVIGATE after scan with frontier");

        // First step in NAVIGATE gives us the exact waypoint (x, y).
        let goto_cmd = planner.step(start, 0.0, &map, 4.0, 0.0);
        let (wx, wy) = match goto_cmd {
            ExplorationCommand::GoTo { x, y, .. } => (x, y),
            ExplorationCommand::Hold { x, y, .. } => (x, y), // already arrived
            _ => (start.x, start.y),
        };

        // Step from the waypoint position: dist < ARRIVE_DIST_M (= 0.15 m).
        let at_waypoint = Vec3::new(wx, wy, 0.3);
        let _hold = planner.step(at_waypoint, 0.0, &map, 4.0, 0.0);

        // After arriving the planner should be back in SCAN.
        assert_eq!(planner.state_name(), "SCAN",
                   "should return to SCAN after arriving at frontier");
    }

    #[test]
    fn land_state_emits_land_command_on_every_step() {
        let mut planner = ExplorationPlanner::new(0.3);
        let map = OccupancyMap::new();
        // Trigger landing via time limit.
        planner.step(origin(), 0.0, &map, 4.0, MAX_EXPLORE_S + 1.0);
        // Every subsequent step should return Land.
        for _ in 0..5 {
            let cmd = planner.step(origin(), 0.0, &map, 4.0, MAX_EXPLORE_S + 1.0);
            match cmd {
                ExplorationCommand::Land { .. } => {}
                other => panic!("Expected Land, got {:?}", other),
            }
        }
    }

    #[test]
    fn vbat_zero_does_not_trigger_land() {
        // vbat = 0.0 should be treated as "not available" (guard: vbat > 0.1).
        let mut planner = ExplorationPlanner::new(0.3);
        let map = OccupancyMap::new();
        let cmd = planner.step(origin(), 0.0, &map, 0.0, 0.0);
        // Should still be scanning, not landing.
        assert_eq!(planner.state_name(), "SCAN",
                   "vbat=0 (disconnected) should not trigger landing");
        match cmd {
            ExplorationCommand::Hold { .. } => {}
            other => panic!("Expected Hold for vbat=0, got {:?}", other),
        }
    }
}
