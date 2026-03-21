//! Sparse 3D log-odds occupancy grid.
//!
//! ## Coordinate frame
//!
//! World frame, NED-aligned:
//! - +X = forward (drone nose direction at zero yaw)
//! - +Y = left
//! - +Z = up
//!
//! Voxel indices are signed 16-bit integers, giving a map extent of
//! ±(32767 × 0.05 m) = ±1638 m — more than enough for indoor flight.
//!
//! ## Log-odds belief
//!
//! Each voxel stores log p/(1−p).  Positive → occupied, negative → free,
//! zero / absent → unknown.  Saturated at ±3.0 to prevent over-confidence.
//!
//! ## Ray casting
//!
//! For each valid range reading:
//! 1. Rotate the body-frame sensor direction by the drone attitude (RPY degrees).
//! 2. Step along the ray at `VOXEL_SIZE_M` intervals, decrementing log-odds
//!    (free evidence).
//! 3. Mark the endpoint voxel with a positive log-odds increment (occupied
//!    evidence).
//!
//! ## Frontier detection
//!
//! A frontier voxel is a **free** cell (log-odds < −0.1) that has at least one
//! neighbour absent from the map (unknown).  Frontiers are exploration targets.

use crate::math::{Vec3, Quat};
use std::collections::HashMap;
use std::io::Write;

// ---------------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------------

/// Voxel edge length in metres.
pub const VOXEL_SIZE_M: f32 = 0.05;

/// Log-odds increment applied to the endpoint voxel (occupied evidence).
const LOG_ODDS_OCC: f32 = 0.7;

/// Log-odds increment applied to voxels along the ray (free-space evidence).
const LOG_ODDS_FREE: f32 = -0.4;

/// Log-odds saturation limits — prevents over-confident voxels.
const LOG_ODDS_MIN: f32 = -3.0;
const LOG_ODDS_MAX: f32 = 3.0;

/// Voxels above this threshold are considered occupied for export and queries.
const OCCUPIED_THRESH: f32 = 0.5;

/// Maximum valid range sensor reading (VL53L1x datasheet max: 4.0 m).
const MAX_RANGE_M: f32 = 4.0;

/// Minimum valid range sensor reading (sensor blind zone).
const MIN_RANGE_M: f32 = 0.05;

// ---------------------------------------------------------------------------
// Public types
// ---------------------------------------------------------------------------

/// Summary statistics about the current map state.
#[derive(Debug, Clone, Copy)]
pub struct MapStats {
    /// Total voxels touched (occupied + free, not unknown).
    pub n_total:    usize,
    /// Voxels with log-odds > `OCCUPIED_THRESH`.
    pub n_occupied: usize,
    /// Voxels with log-odds < −0.1 (more likely free than not).
    pub n_free:     usize,
}

// ---------------------------------------------------------------------------
// OccupancyMap
// ---------------------------------------------------------------------------

/// Sparse 3D log-odds occupancy grid backed by a `HashMap`.
///
/// Only voxels that have been observed at least once are stored.
/// Absent voxels are implicitly **unknown** (log-odds = 0).
pub struct OccupancyMap {
    /// Voxel storage: key = (ix, iy, iz) indices, value = log-odds.
    voxels: HashMap<(i16, i16, i16), f32>,
}

impl OccupancyMap {
    /// Create an empty map.
    pub fn new() -> Self {
        Self { voxels: HashMap::new() }
    }

    /// Number of voxels that have been observed (occupied or free).
    pub fn len(&self) -> usize { self.voxels.len() }

    /// True if no voxels have been observed yet.
    pub fn is_empty(&self) -> bool { self.voxels.is_empty() }

    // -----------------------------------------------------------------------
    // Map update
    // -----------------------------------------------------------------------

    /// Incorporate one set of range readings into the map.
    ///
    /// # Parameters
    ///
    /// - `pos_m`       — Drone position in world frame (metres).
    /// - `roll_deg`    — MEKF roll  (degrees, positive = right-side down).
    /// - `pitch_deg`   — MEKF pitch (degrees, positive = nose down).
    /// - `yaw_deg`     — MEKF yaw   (degrees, 0 = north/forward).
    /// - `front_m` … `down_m` — Range readings along each body axis (`None` =
    ///   sensor out of range or not available).
    ///
    /// # Sensor → body frame mapping
    ///
    /// | Parameter  | Body direction | Hardware              |
    /// |------------|----------------|-----------------------|
    /// | `front_m`  | +X             | Multi-ranger front    |
    /// | `back_m`   | −X             | Multi-ranger back     |
    /// | `left_m`   | +Y             | Multi-ranger left     |
    /// | `right_m`  | −Y             | Multi-ranger right    |
    /// | `up_m`     | +Z             | Multi-ranger up       |
    /// | `down_m`   | −Z             | Flow Deck VL53L1x     |
    #[allow(clippy::too_many_arguments)]
    pub fn update(
        &mut self,
        pos_m:     Vec3,
        roll_deg:  f32,
        pitch_deg: f32,
        yaw_deg:   f32,
        front_m:   Option<f32>,
        back_m:    Option<f32>,
        left_m:    Option<f32>,
        right_m:   Option<f32>,
        up_m:      Option<f32>,
        down_m:    Option<f32>,
    ) {
        let q = rpy_to_quat(roll_deg, pitch_deg, yaw_deg);

        // Body-frame unit direction vectors, rotated to world frame.
        let rays: [(Vec3, Option<f32>); 6] = [
            (q.rotate_vector(Vec3::new( 1.0,  0.0,  0.0)), front_m),
            (q.rotate_vector(Vec3::new(-1.0,  0.0,  0.0)), back_m),
            (q.rotate_vector(Vec3::new( 0.0,  1.0,  0.0)), left_m),
            (q.rotate_vector(Vec3::new( 0.0, -1.0,  0.0)), right_m),
            (q.rotate_vector(Vec3::new( 0.0,  0.0,  1.0)), up_m),
            (q.rotate_vector(Vec3::new( 0.0,  0.0, -1.0)), down_m),
        ];

        for (world_dir, range_opt) in rays {
            if let Some(range_m) = range_opt {
                if range_m >= MIN_RANGE_M && range_m <= MAX_RANGE_M {
                    self.cast_ray(pos_m, world_dir, range_m);
                }
            }
        }
    }

    // -----------------------------------------------------------------------
    // Queries
    // -----------------------------------------------------------------------

    /// Log-odds value for voxel at world position `p`, or 0.0 (unknown) if
    /// the voxel has never been observed.
    pub fn log_odds_at(&self, p: Vec3) -> f32 {
        *self.voxels.get(&world_to_key(p)).unwrap_or(&0.0)
    }

    /// True if the voxel at `p` is occupied (log-odds > threshold).
    pub fn is_occupied(&self, p: Vec3) -> bool {
        self.log_odds_at(p) > OCCUPIED_THRESH
    }

    /// Returns the centre positions (world frame, metres) of all frontier
    /// voxels: **free** cells that have at least one **unknown** neighbour.
    ///
    /// Useful for exploration: fly toward the nearest frontier to extend the
    /// known map.
    pub fn frontiers(&self) -> Vec<Vec3> {
        let mut result = Vec::new();
        for (&(ix, iy, iz), &log_odds) in &self.voxels {
            // Must be a free cell (not occupied, not unknown).
            if log_odds >= -0.1 { continue; }
            // At least one of the 6 face-neighbours must be unknown (absent).
            let unknown_neighbour = [
                (ix + 1, iy, iz), (ix - 1, iy, iz),
                (ix, iy + 1, iz), (ix, iy - 1, iz),
                (ix, iy, iz + 1), (ix, iy, iz - 1),
            ].iter().any(|k| !self.voxels.contains_key(k));

            if unknown_neighbour {
                result.push(key_to_world(ix, iy, iz));
            }
        }
        result
    }

    /// Summary statistics (occupied / free / total observed voxel counts).
    pub fn stats(&self) -> MapStats {
        let n_occupied = self.voxels.values().filter(|&&v| v > OCCUPIED_THRESH).count();
        let n_free     = self.voxels.values().filter(|&&v| v < -0.1).count();
        MapStats { n_total: self.voxels.len(), n_occupied, n_free }
    }

    // -----------------------------------------------------------------------
    // Export
    // -----------------------------------------------------------------------

    /// Serialise the occupied voxels as an ASCII PLY point cloud.
    ///
    /// Open the result in MeshLab, CloudCompare, or any PLY viewer.
    /// Each point carries an extra `log_odds` scalar for colour-mapping.
    pub fn to_ply(&self) -> Vec<u8> {
        let occupied: Vec<_> = self.voxels.iter()
            .filter(|(_, &v)| v > OCCUPIED_THRESH)
            .collect();

        let mut buf = Vec::with_capacity(occupied.len() * 40 + 256);
        writeln!(buf, "ply").unwrap();
        writeln!(buf, "format ascii 1.0").unwrap();
        writeln!(buf, "comment generated by flying_drone_stack OccupancyMap").unwrap();
        writeln!(buf, "element vertex {}", occupied.len()).unwrap();
        writeln!(buf, "property float x").unwrap();
        writeln!(buf, "property float y").unwrap();
        writeln!(buf, "property float z").unwrap();
        writeln!(buf, "property float log_odds").unwrap();
        writeln!(buf, "end_header").unwrap();
        for (&(ix, iy, iz), &log_odds) in &occupied {
            writeln!(buf, "{:.4} {:.4} {:.4} {:.4}",
                ix as f32 * VOXEL_SIZE_M,
                iy as f32 * VOXEL_SIZE_M,
                iz as f32 * VOXEL_SIZE_M,
                log_odds).unwrap();
        }
        buf
    }

    /// Export **all** observed voxels (occupied + free) as PLY.
    /// Free voxels have negative log_odds — useful for debugging ray casting.
    pub fn to_ply_full(&self) -> Vec<u8> {
        let mut buf = Vec::with_capacity(self.voxels.len() * 40 + 256);
        writeln!(buf, "ply").unwrap();
        writeln!(buf, "format ascii 1.0").unwrap();
        writeln!(buf, "comment generated by flying_drone_stack OccupancyMap (full)").unwrap();
        writeln!(buf, "element vertex {}", self.voxels.len()).unwrap();
        writeln!(buf, "property float x").unwrap();
        writeln!(buf, "property float y").unwrap();
        writeln!(buf, "property float z").unwrap();
        writeln!(buf, "property float log_odds").unwrap();
        writeln!(buf, "end_header").unwrap();
        for (&(ix, iy, iz), &log_odds) in &self.voxels {
            writeln!(buf, "{:.4} {:.4} {:.4} {:.4}",
                ix as f32 * VOXEL_SIZE_M,
                iy as f32 * VOXEL_SIZE_M,
                iz as f32 * VOXEL_SIZE_M,
                log_odds).unwrap();
        }
        buf
    }

    // -----------------------------------------------------------------------
    // Internal helpers
    // -----------------------------------------------------------------------

    /// Cast a single ray: mark intermediate voxels free and the endpoint occupied.
    fn cast_ray(&mut self, origin: Vec3, dir: Vec3, range_m: f32) {
        // Step through free-space voxels up to (but not including) the endpoint.
        let n_free_steps = ((range_m - VOXEL_SIZE_M).max(0.0) / VOXEL_SIZE_M) as usize;
        for s in 0..n_free_steps {
            let t = (s as f32 + 0.5) * VOXEL_SIZE_M;
            let p = origin + dir * t;
            update_voxel(&mut self.voxels, world_to_key(p), LOG_ODDS_FREE);
        }
        // Mark endpoint as occupied.
        let endpoint = origin + dir * range_m;
        update_voxel(&mut self.voxels, world_to_key(endpoint), LOG_ODDS_OCC);
    }
}

impl Default for OccupancyMap {
    fn default() -> Self { Self::new() }
}

// ---------------------------------------------------------------------------
// Free functions
// ---------------------------------------------------------------------------

/// Build a rotation quaternion from ZYX Euler angles (degrees).
/// Convention: q = q_yaw * q_pitch * q_roll (yaw applied last).
fn rpy_to_quat(roll_deg: f32, pitch_deg: f32, yaw_deg: f32) -> Quat {
    let q_roll  = Quat::from_axis_angle(Vec3::new(1.0, 0.0, 0.0), roll_deg.to_radians());
    let q_pitch = Quat::from_axis_angle(Vec3::new(0.0, 1.0, 0.0), pitch_deg.to_radians());
    let q_yaw   = Quat::from_axis_angle(Vec3::new(0.0, 0.0, 1.0), yaw_deg.to_radians());
    q_yaw * q_pitch * q_roll
}

/// Convert a world-frame position to a voxel grid key.
#[inline]
fn world_to_key(p: Vec3) -> (i16, i16, i16) {
    let clamp = |v: f32| (v.round() as i32).clamp(i16::MIN as i32, i16::MAX as i32) as i16;
    (clamp(p.x / VOXEL_SIZE_M),
     clamp(p.y / VOXEL_SIZE_M),
     clamp(p.z / VOXEL_SIZE_M))
}

/// Convert a voxel grid key back to its world-frame centre position.
#[inline]
fn key_to_world(ix: i16, iy: i16, iz: i16) -> Vec3 {
    Vec3::new(ix as f32 * VOXEL_SIZE_M,
              iy as f32 * VOXEL_SIZE_M,
              iz as f32 * VOXEL_SIZE_M)
}

/// Apply a log-odds delta to a voxel, inserting it as unknown (0.0) first if
/// absent, and clamping to [LOG_ODDS_MIN, LOG_ODDS_MAX].
#[inline]
fn update_voxel(map: &mut HashMap<(i16, i16, i16), f32>, key: (i16, i16, i16), delta: f32) {
    let v = map.entry(key).or_insert(0.0);
    *v = (*v + delta).clamp(LOG_ODDS_MIN, LOG_ODDS_MAX);
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    /// Helper: map with one horizontal ray in the +X direction.
    fn single_ray_map(range_m: f32) -> OccupancyMap {
        let mut m = OccupancyMap::new();
        // Level drone at origin, 1 ray forward at `range_m`.
        m.update(Vec3::zero(), 0.0, 0.0, 0.0,
                 Some(range_m), None, None, None, None, None);
        m
    }

    #[test]
    fn voxel_key_round_trip() {
        for &(x, y, z) in &[(0.0f32, 0.0, 0.0), (1.5, -0.75, 0.3), (-2.0, 0.1, 4.0)] {
            let p = Vec3::new(x, y, z);
            let (ix, iy, iz) = world_to_key(p);
            let q = key_to_world(ix, iy, iz);
            // Reconstructed position should be within half a voxel.
            assert!((q.x - p.x).abs() <= VOXEL_SIZE_M * 0.5 + 1e-5,
                "x round-trip failed: {} → {}", p.x, q.x);
            assert!((q.y - p.y).abs() <= VOXEL_SIZE_M * 0.5 + 1e-5,
                "y round-trip failed: {} → {}", p.y, q.y);
            assert!((q.z - p.z).abs() <= VOXEL_SIZE_M * 0.5 + 1e-5,
                "z round-trip failed: {} → {}", p.z, q.z);
        }
    }

    #[test]
    fn single_ray_endpoint_is_occupied() {
        let map = single_ray_map(0.5); // wall 0.5 m in front
        // The voxel at x≈0.5 should be occupied.
        assert!(map.is_occupied(Vec3::new(0.5, 0.0, 0.0)),
            "endpoint voxel should be occupied");
    }

    #[test]
    fn single_ray_midpoint_is_free() {
        let map = single_ray_map(1.0);
        // Voxel halfway along the ray should have negative log-odds (free).
        let lo = map.log_odds_at(Vec3::new(0.5, 0.0, 0.0));
        assert!(lo < 0.0, "midpoint voxel should be free, got log-odds={}", lo);
    }

    #[test]
    fn repeated_updates_saturate() {
        let mut map = OccupancyMap::new();
        for _ in 0..20 {
            map.update(Vec3::zero(), 0.0, 0.0, 0.0,
                       Some(0.3), None, None, None, None, None);
        }
        let lo = map.log_odds_at(Vec3::new(0.3, 0.0, 0.0));
        assert!(lo <= LOG_ODDS_MAX + 1e-5,
            "log-odds should be saturated at {}, got {}", LOG_ODDS_MAX, lo);
    }

    #[test]
    fn attitude_rotates_ray_direction() {
        let mut map = OccupancyMap::new();
        // Yaw 90° CCW: body +X maps to world +Y.
        // A 1m range in the "front" sensor should appear at ~(0, 1, 0) in world.
        map.update(Vec3::zero(), 0.0, 0.0, 90.0,
                   Some(1.0), None, None, None, None, None);
        // The endpoint should be near (0, 1, 0), NOT (1, 0, 0).
        assert!(!map.is_occupied(Vec3::new(1.0, 0.0, 0.0)),
            "wrong direction: (1,0,0) should NOT be occupied after 90° yaw");
        assert!(map.is_occupied(Vec3::new(0.0, 1.0, 0.0)),
            "correct direction: (0,1,0) should be occupied after 90° yaw");
    }

    #[test]
    fn frontier_detection() {
        let mut map = OccupancyMap::new();
        // A long ray leaves a trail of free voxels near the origin and an
        // occupied endpoint far away.  The free voxels near the origin should
        // be frontiers (they have unknown neighbours in the Y/Z directions).
        map.update(Vec3::zero(), 0.0, 0.0, 0.0,
                   Some(2.0), None, None, None, None, None);
        let fronts = map.frontiers();
        assert!(!fronts.is_empty(), "should find at least one frontier voxel");
    }

    #[test]
    fn ply_output_has_valid_header() {
        let map = single_ray_map(0.5);
        let ply = map.to_ply();
        let text = String::from_utf8(ply).expect("PLY should be valid UTF-8");
        assert!(text.starts_with("ply\n"), "PLY must start with 'ply'");
        assert!(text.contains("element vertex"), "PLY must declare vertices");
        assert!(text.contains("end_header"), "PLY must have end_header");
        // There should be at least one data line after the header.
        let header_end = text.find("end_header\n").unwrap() + "end_header\n".len();
        let data = &text[header_end..];
        assert!(!data.trim().is_empty(), "PLY must have at least one point");
    }

    #[test]
    fn stats_counts_match() {
        let map = single_ray_map(1.0);
        let s = map.stats();
        assert!(s.n_occupied >= 1, "should have at least one occupied voxel");
        assert!(s.n_free >= 1,     "should have at least one free voxel");
        assert_eq!(s.n_total, map.len());
    }

    #[test]
    fn down_ray_from_altitude() {
        // Drone at 0.3 m altitude, down sensor reads 0.3 m.
        // The occupied voxel should be at z≈0 (floor).
        let mut map = OccupancyMap::new();
        map.update(Vec3::new(0.0, 0.0, 0.3), 0.0, 0.0, 0.0,
                   None, None, None, None, None, Some(0.3));
        assert!(map.is_occupied(Vec3::new(0.0, 0.0, 0.0)),
            "floor voxel at z=0 should be occupied");
    }
}
