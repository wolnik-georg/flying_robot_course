//! 3D occupancy mapping, visual odometry, and SLAM.
//!
//! # Modules
//!
//! - [`occupancy`] — Sparse log-odds voxel grid updated from multi-ranger
//!   measurements; exports PLY point clouds and frontier lists for exploration.
//! - [`keyframe`] — Rolling keyframe buffer + unbounded global index.
//!   Accepts `ImageFrame`s, detects FAST-9 features, matches against the
//!   previous keyframe, and recovers metric translation via the 8-point
//!   essential matrix.  Also provides full-history loop-closure search via
//!   [`KeyframeStore::detect_loop`].
//! - [`vo_trajectory`] — Chains keyframe relative poses into a world-frame
//!   trajectory; can be re-seeded from MEKF after a loop closure.
//! - [`loop_closure`] — [`PoseGraph`] with unbounded Vec-backed node storage.
//!   Sequential edges (σ ≈ 10 cm) + loop edges optimised by Gauss-Seidel.
//!
//! # Quick start
//!
//! ```rust,no_run
//! use multirotor_simulator::mapping::OccupancyMap;
//! use multirotor_simulator::math::Vec3;
//!
//! let mut map = OccupancyMap::new();
//! // Drone at origin, level, 1.5 m wall in front:
//! map.update(Vec3::zero(), 0.0, 0.0, 0.0,
//!            Some(1.5), None, None, None, None, Some(0.3));
//! let stats = map.stats();
//! println!("{} occupied voxels", stats.n_occupied);
//! ```

pub mod occupancy;
pub mod keyframe;
pub mod vo_trajectory;
pub mod loop_closure;

pub use occupancy::OccupancyMap;
pub use occupancy::MapStats;
pub use keyframe::{KeyframeStore, KeyframeResult, Keyframe, FeatureMatch, CompactKeyframe};
pub use vo_trajectory::VoTrajectory;
pub use loop_closure::{LoopConstraint, PoseGraph};
