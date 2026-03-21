//! 3D occupancy mapping and autonomous exploration.
//!
//! # Modules
//!
//! - [`occupancy`] — Sparse log-odds voxel grid.  Updated from multi-ranger
//!   range measurements; exports PLY point clouds and frontier lists for
//!   autonomous exploration.
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

pub use occupancy::OccupancyMap;
pub use occupancy::MapStats;
