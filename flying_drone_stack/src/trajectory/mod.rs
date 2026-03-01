//! Trajectory Generation for Multirotor Control
//!
//! Provides trajectory generation utilities for testing geometric controllers.
//! Includes figure-8 patterns, circular trajectories, and CSV-based setpoints.

use crate::math::Vec3;
use crate::controller::TrajectoryReference;
use std::f32::consts::PI;

/// Trait for trajectory generators
pub trait Trajectory {
    /// Get the trajectory reference at a given time
    fn get_reference(&self, time: f32) -> TrajectoryReference;

    /// Get the total duration of the trajectory (if applicable)
    fn duration(&self) -> Option<f32> {
        None
    }
}

/// Figure-8 trajectory based on the polynomial coefficients from the Crazyflie Python example
pub struct Figure8Trajectory {
    /// Duration of one figure-8 loop in seconds
    pub duration: f32,
    /// Height offset for the trajectory
    pub height: f32,
    /// Scale factor for the trajectory size (spatial)
    pub scale: f32,
    /// Time scale factor: >1 slows the trajectory down, <1 speeds it up.
    /// Segment durations are multiplied by this value; velocity/acceleration
    /// are divided by time_scale / time_scale² accordingly.
    pub time_scale: f32,
}

/// Smooth parametric figure-8 (Lissajous) trajectory
///
/// Uses sinusoidal basis to generate a figure-8 that is C-infinite (all derivatives
/// are continuous). This is a simple, robust alternative to the polynomial segment
/// approach and produces smooth velocity/acceleration/jerk for safe tracking.
pub struct SmoothFigure8Trajectory {
    /// Duration of the loop [s]
    pub duration: f32,
    /// Height offset [m]
    pub height: f32,
    /// X amplitude [m]
    pub a: f32,
    /// Y amplitude [m]
    pub b: f32,
    /// Frequency scale: omega = 2*pi / duration
    pub time_scale: f32,
}

impl SmoothFigure8Trajectory {
    pub fn new() -> Self {
        Self { duration: 8.0, height: 0.25, a: 0.25, b: 0.25, time_scale: 1.0 }
    }

    pub fn with_params(duration: f32, height: f32, scale: f32) -> Self {
        // scale controls overall spatial amplitude
        Self { duration, height, a: scale, b: scale, time_scale: 1.0 }
    }
}

impl Trajectory for SmoothFigure8Trajectory {
    fn get_reference(&self, time: f32) -> TrajectoryReference {
        // map time into [0, duration)
        let t = time % self.duration;
        let omega = 2.0 * std::f32::consts::PI / self.duration;

        // Lissajous-style figure-8: x = a*sin(omega*t), y = b*sin(2*omega*t)
        let x = self.a * (omega * t).sin();
        let y = self.b * (2.0 * omega * t).sin();
        let z = self.height;

        // velocities
        let vx = self.a * omega * (omega * t).cos();
        let vy = self.b * 2.0 * omega * (2.0 * omega * t).cos();
        let vz = 0.0;

        // accelerations
        let ax = -self.a * omega * omega * (omega * t).sin();
        let ay = -self.b * (2.0 * omega) * (2.0 * omega) * (2.0 * omega * t).sin();
        let az = 0.0;

        // jerks
        let jx = -self.a * omega * omega * omega * (omega * t).cos();
        let jy = -self.b * (2.0 * omega) * (2.0 * omega) * (2.0 * omega) * (2.0 * omega * t).cos();
        let jz = 0.0;

        // yaw aligned with velocity direction
        use crate::math::Vec3;
        let yaw = (vy).atan2(vx);
        // yaw rate (d/dt atan2(vy,vx)) = (vx*ay - vy*ax) / (vx^2 + vy^2)
        let denom = vx*vx + vy*vy;
        let yaw_rate = if denom.abs() > 1e-8 { (vx*ay - vy*ax) / denom } else { 0.0 };
        // yaw acceleration approximate (set to zero for simplicity)
        let yaw_acceleration = 0.0;

        TrajectoryReference {
            position: Vec3::new(x, y, z),
            velocity: Vec3::new(vx, vy, vz),
            acceleration: Vec3::new(ax, ay, az),
            jerk: Vec3::new(jx, jy, jz),
            yaw,
            yaw_rate,
            yaw_acceleration,
        }
    }

    fn duration(&self) -> Option<f32> {
        Some(self.duration)
    }
}

impl Figure8Trajectory {
    /// Create a new figure-8 trajectory with default parameters
    pub fn new() -> Self {
        Self {
            duration: 8.0, // 8 seconds for one figure-8
            height: 0.5,   // 0.5m height
            scale: 1.0,    // Normal size
            time_scale: 1.0,
        }
    }

    /// Create figure-8 trajectory with custom parameters
    pub fn with_params(duration: f32, height: f32, scale: f32) -> Self {
        Self { duration, height, scale, time_scale: 1.0 }
    }

    /// Create figure-8 trajectory with explicit time scaling.
    /// `time_scale` > 1 stretches time (slower), < 1 compresses (faster).
    pub fn with_time_scale(height: f32, scale: f32, time_scale: f32) -> Self {
        // duration wraps at the scaled total segment time
        let base_total = 1.050000 + 0.710000 + 0.620000 + 0.700000 + 0.560000
                       + 0.560000 + 0.700000 + 0.620000 + 0.710000 + 1.053185;
        Self { duration: base_total * time_scale, height, scale, time_scale }
    }

    /// Evaluate 7th order polynomial p(s) = c0 + c1*s + c2*s² + ... + c7*s⁷
    fn evaluate_polynomial(&self, coeffs: &[f32], s: f32) -> f32 {
        coeffs.iter().enumerate()
            .map(|(i, &c)| c * s.powi(i as i32))
            .sum()
    }

    /// Evaluate first derivative p'(s)
    fn evaluate_polynomial_derivative(&self, coeffs: &[f32], s: f32) -> f32 {
        coeffs.iter().enumerate().skip(1)
            .map(|(i, &c)| c * (i as f32) * s.powi(i as i32 - 1))
            .sum()
    }

    /// Evaluate second derivative p''(s)
    fn evaluate_polynomial_second_derivative(&self, coeffs: &[f32], s: f32) -> f32 {
        coeffs.iter().enumerate().skip(2)
            .map(|(i, &c)| c * (i as f32) * ((i as f32) - 1.0) * s.powi(i as i32 - 2))
            .sum()
    }

    /// Evaluate third derivative p'''(s) for jerk computation
    fn evaluate_polynomial_third_derivative(&self, coeffs: &[f32], s: f32) -> f32 {
        coeffs.iter().enumerate().skip(3)
            .map(|(i, &c)| c * (i as f32) * ((i as f32) - 1.0) * ((i as f32) - 2.0) * s.powi(i as i32 - 3))
            .sum()
    }
}

impl Trajectory for Figure8Trajectory {
    fn get_reference(&self, time: f32) -> TrajectoryReference {
    // Wrap time to stay within one full trajectory period (seconds).
    // The coefficients include a duration field in seconds for each
    // segment, so we must compare the raw time against those values.
    // Previously we normalized t to [0,1] which meant segment_time
    // never exceeded the first segment's duration, so only the first
    // polynomial block was ever used.  The reference path therefore
    // collapsed to a tiny fraction of the intended figure‑8.
    let mut segment_time = time % self.duration;

        // Figure-8 polynomial coefficients (from autonomous_sequence_high_level.py)
        // Duration,x^0,x^1,x^2,x^3,x^4,x^5,x^6,x^7,y^0,y^1,y^2,y^3,y^4,y^5,y^6,y^7,z^0,z^1,z^2,z^3,z^4,z^5,z^6,z^7,yaw^0,yaw^1,yaw^2,yaw^3,yaw^4,yaw^5,yaw^6,yaw^7
        let coeffs = [
            [1.050000, 0.000000, -0.000000, 0.000000, -0.000000, 0.830443, -0.276140, -0.384219, 0.180493, -0.000000, 0.000000, -0.000000, 0.000000, -1.356107, 0.688430, 0.587426, -0.329106, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],
            [0.710000, 0.396058, 0.918033, 0.128965, -0.773546, 0.339704, 0.034310, -0.026417, -0.030049, -0.445604, -0.684403, 0.888433, 1.493630, -1.361618, -0.139316, 0.158875, 0.095799, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],
            [0.620000, 0.922409, 0.405715, -0.582968, -0.092188, -0.114670, 0.101046, 0.075834, -0.037926, -0.291165, 0.967514, 0.421451, -1.086348, 0.545211, 0.030109, -0.050046, -0.068177, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],
            [0.700000, 0.923174, -0.431533, -0.682975, 0.177173, 0.319468, -0.043852, -0.111269, 0.023166, 0.289869, 0.724722, -0.512011, -0.209623, -0.218710, 0.108797, 0.128756, -0.055461, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],
            [0.560000, 0.405364, -0.834716, 0.158939, 0.288175, -0.373738, -0.054995, 0.036090, 0.078627, 0.450742, -0.385534, -0.954089, 0.128288, 0.442620, 0.055630, -0.060142, -0.076163, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],
            [0.560000, 0.001062, -0.646270, -0.012560, -0.324065, 0.125327, 0.119738, 0.034567, -0.063130, 0.001593, -1.031457, 0.015159, 0.820816, -0.152665, -0.130729, -0.045679, 0.080444, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],
            [0.700000, -0.402804, -0.820508, -0.132914, 0.236278, 0.235164, -0.053551, -0.088687, 0.031253, -0.449354, -0.411507, 0.902946, 0.185335, -0.239125, -0.041696, 0.016857, 0.016709, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],
            [0.620000, -0.921641, -0.464596, 0.661875, 0.286582, -0.228921, -0.051987, 0.004669, 0.038463, -0.292459, 0.777682, 0.565788, -0.432472, -0.060568, -0.082048, -0.009439, 0.041158, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],
            [0.710000, -0.923935, 0.447832, 0.627381, -0.259808, -0.042325, -0.032258, 0.001420, 0.005294, 0.288570, 0.873350, -0.515586, -0.730207, -0.026023, 0.288755, 0.215678, -0.148061, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],
            [1.053185, -0.398611, 0.850510, -0.144007, -0.485368, -0.079781, 0.176330, 0.234482, -0.153567, 0.447039, -0.532729, -0.855023, 0.878509, 0.775168, -0.391051, -0.713519, 0.391628, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],
        ];

    // Find which segment we're in. Segment durations are scaled by time_scale
    // so the drone traverses the same geometric path at a different speed.
    let ts = self.time_scale;
    let mut segment_idx = 0;
    let mut _cumulative_time = 0.0;

        for (i, coeff) in coeffs.iter().enumerate() {
            let segment_duration = coeff[0] * ts;
            if segment_time <= segment_duration {
                segment_idx = i;
                break;
            }
            // advance into the next segment
            segment_time -= segment_duration;
            _cumulative_time += segment_duration;
        }

        // Normalize segment time to [0, 1] for polynomial evaluation
        let segment_duration = coeffs[segment_idx][0] * ts;
        let s = if segment_duration > 0.0 {
            segment_time / segment_duration
        } else {
            0.0
        };

        // Evaluate polynomials for x, y, z, yaw
        let x = self.evaluate_polynomial(&coeffs[segment_idx][1..9], s) * self.scale;
        let y = self.evaluate_polynomial(&coeffs[segment_idx][9..17], s) * self.scale;
        let z = self.evaluate_polynomial(&coeffs[segment_idx][17..25], s) * self.scale + self.height;
        let yaw = self.evaluate_polynomial(&coeffs[segment_idx][25..33], s);

        // Compute derivatives (velocity and acceleration)
        // Note: derivatives are with respect to s ∈ [0,1], so we divide by segment_duration to get time derivatives
        let vx = self.evaluate_polynomial_derivative(&coeffs[segment_idx][1..9], s) * self.scale / segment_duration;
        let vy = self.evaluate_polynomial_derivative(&coeffs[segment_idx][9..17], s) * self.scale / segment_duration;
        let vz = self.evaluate_polynomial_derivative(&coeffs[segment_idx][17..25], s) / segment_duration;

        let ax = self.evaluate_polynomial_second_derivative(&coeffs[segment_idx][1..9], s) * self.scale / (segment_duration * segment_duration);
        let ay = self.evaluate_polynomial_second_derivative(&coeffs[segment_idx][9..17], s) * self.scale / (segment_duration * segment_duration);
        let az = self.evaluate_polynomial_second_derivative(&coeffs[segment_idx][17..25], s) / (segment_duration * segment_duration);

        let jx = self.evaluate_polynomial_third_derivative(&coeffs[segment_idx][1..9], s) * self.scale / (segment_duration * segment_duration * segment_duration);
        let jy = self.evaluate_polynomial_third_derivative(&coeffs[segment_idx][9..17], s) * self.scale / (segment_duration * segment_duration * segment_duration);
        let jz = self.evaluate_polynomial_third_derivative(&coeffs[segment_idx][17..25], s) / (segment_duration * segment_duration * segment_duration);

        let yaw_rate = self.evaluate_polynomial_derivative(&coeffs[segment_idx][25..33], s) / segment_duration;
        let yaw_acceleration = self.evaluate_polynomial_second_derivative(&coeffs[segment_idx][25..33], s) / (segment_duration * segment_duration);

        TrajectoryReference {
            position: Vec3::new(x, y, z),
            velocity: Vec3::new(vx, vy, vz),
            acceleration: Vec3::new(ax, ay, az),
            jerk: Vec3::new(jx, jy, jz),
            yaw,
            yaw_rate,
            yaw_acceleration,
        }
    }

    fn duration(&self) -> Option<f32> {
        Some(self.duration)
    }
}

/// Circular trajectory in the xy-plane
pub struct CircleTrajectory {
    /// Radius of the circle [m]
    pub radius: f32,
    /// Height of the trajectory [m]
    pub height: f32,
    /// Angular velocity [rad/s]
    pub omega: f32,
    /// Center point (x, y) of the circle
    pub center: (f32, f32),
}

impl CircleTrajectory {
    /// Create a new circular trajectory
    pub fn new(radius: f32, height: f32, omega: f32) -> Self {
        Self {
            radius,
            height,
            omega,
            center: (0.0, 0.0),
        }
    }

    /// Create circular trajectory with custom center
    pub fn with_center(radius: f32, height: f32, omega: f32, center: (f32, f32)) -> Self {
        Self { radius, height, omega, center }
    }
}

impl Trajectory for CircleTrajectory {
    fn get_reference(&self, time: f32) -> TrajectoryReference {
        let theta = self.omega * time;

        // Position
        let x = self.center.0 + self.radius * theta.cos();
        let y = self.center.1 + self.radius * theta.sin();
        let z = self.height;

        // Velocity (tangential)
        let vx = -self.radius * self.omega * theta.sin();
        let vy = self.radius * self.omega * theta.cos();
        let vz = 0.0;

        // Acceleration (centripetal)
        let ax = -self.radius * self.omega * self.omega * theta.cos();
        let ay = -self.radius * self.omega * self.omega * theta.sin();
        let az = 0.0;

        // Jerk (derivative of acceleration)
        let jx = self.radius * self.omega * self.omega * self.omega * theta.sin();
        let jy = -self.radius * self.omega * self.omega * self.omega * theta.cos();
        let jz = 0.0;

        // Yaw always points tangent to the circle
        let yaw = if self.radius > 0.0 { theta + PI / 2.0 } else { 0.0 }; // Point in direction of motion, or 0 for stationary
        let yaw_rate = self.omega;
        let yaw_acceleration = 0.0;

        TrajectoryReference {
            position: Vec3::new(x, y, z),
            velocity: Vec3::new(vx, vy, vz),
            acceleration: Vec3::new(ax, ay, az),
            jerk: Vec3::new(jx, jy, jz),
            yaw,
            yaw_rate,
            yaw_acceleration,
        }
    }
}

/// CSV-based trajectory loaded from file
pub struct CsvTrajectory {
    /// List of (time, x, y, z, yaw) waypoints
    waypoints: Vec<(f32, f32, f32, f32, f32)>,
}

impl CsvTrajectory {
    /// Load trajectory from CSV file with format: time,x,y,z,yaw
    pub fn from_csv(filename: &str) -> Result<Self, Box<dyn std::error::Error>> {
        let mut waypoints = Vec::new();

        let content = std::fs::read_to_string(filename)?;
        for line in content.lines().skip(1) { // Skip header
            if line.trim().is_empty() { continue; }

            let parts: Vec<&str> = line.split(',').collect();
            if parts.len() >= 5 {
                let time = parts[0].trim().parse::<f32>()?;
                let x = parts[1].trim().parse::<f32>()?;
                let y = parts[2].trim().parse::<f32>()?;
                let z = parts[3].trim().parse::<f32>()?;
                let yaw = parts[4].trim().parse::<f32>()?;

                waypoints.push((time, x, y, z, yaw));
            }
        }

        waypoints.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap());
        Ok(Self { waypoints })
    }

    /// Create trajectory from vector of waypoints
    pub fn from_waypoints(waypoints: Vec<(f32, f32, f32, f32, f32)>) -> Self {
        Self { waypoints }
    }
}

impl Trajectory for CsvTrajectory {
    fn get_reference(&self, time: f32) -> TrajectoryReference {
        if self.waypoints.is_empty() {
            return TrajectoryReference {
                position: Vec3::zero(),
                velocity: Vec3::zero(),
                acceleration: Vec3::zero(),
                jerk: Vec3::zero(),
                yaw: 0.0,
                yaw_rate: 0.0,
                yaw_acceleration: 0.0,
            };
        }

        // Find the appropriate segment
        let mut idx = 0;
        for (i, &(t, _, _, _, _)) in self.waypoints.iter().enumerate() {
            if t >= time {
                idx = i;
                break;
            }
        }

        if idx == 0 {
            // Before first waypoint
            let (_t, x, y, z, yaw) = self.waypoints[0];
            TrajectoryReference {
                position: Vec3::new(x, y, z),
                velocity: Vec3::zero(),
                acceleration: Vec3::zero(),
                jerk: Vec3::zero(),
                yaw,
                yaw_rate: 0.0,
                yaw_acceleration: 0.0,
            }
        } else if idx >= self.waypoints.len() {
            // After last waypoint
            let (_t, x, y, z, yaw) = *self.waypoints.last().unwrap();
            TrajectoryReference {
                position: Vec3::new(x, y, z),
                velocity: Vec3::zero(),
                acceleration: Vec3::zero(),
                jerk: Vec3::zero(),
                yaw,
                yaw_rate: 0.0,
                yaw_acceleration: 0.0,
            }
        } else {
            // Interpolate between waypoints
            let (t1, x1, y1, z1, yaw1) = self.waypoints[idx - 1];
            let (t2, x2, y2, z2, yaw2) = self.waypoints[idx];

            let dt = t2 - t1;
            if dt > 0.0 {
                let alpha = (time - t1) / dt;

                // Linear interpolation for position and yaw
                let x = x1 + alpha * (x2 - x1);
                let y = y1 + alpha * (y2 - y1);
                let z = z1 + alpha * (z2 - z1);
                let yaw = yaw1 + alpha * (yaw2 - yaw1);

                // Simple finite difference for velocity (could be improved)
                let vx = (x2 - x1) / dt;
                let vy = (y2 - y1) / dt;
                let vz = (z2 - z1) / dt;
                let yaw_rate = (yaw2 - yaw1) / dt;

                TrajectoryReference {
                    position: Vec3::new(x, y, z),
                    velocity: Vec3::new(vx, vy, vz),
                    acceleration: Vec3::zero(), // No acceleration info in CSV
                    jerk: Vec3::zero(), // No jerk info in CSV
                    yaw,
                    yaw_rate,
                    yaw_acceleration: 0.0,
                }
            } else {
                // Degenerate case
                TrajectoryReference {
                    position: Vec3::new(x1, y1, z1),
                    velocity: Vec3::zero(),
                    acceleration: Vec3::zero(),
                    jerk: Vec3::zero(),
                    yaw: yaw1,
                    yaw_rate: 0.0,
                    yaw_acceleration: 0.0,
                }
            }
        }
    }

    fn duration(&self) -> Option<f32> {
        self.waypoints.last().map(|&(t, _, _, _, _)| t)
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// TakeoffTrajectory
// ─────────────────────────────────────────────────────────────────────────────

/// Smooth takeoff from the ground to a hover point using a minimum-jerk
/// polynomial.  The drone starts at `(start_x, start_y, 0)` with zero
/// velocity and acceleration and arrives at `(start_x, start_y, target_z)`
/// with zero velocity and acceleration after `duration` seconds.
///
/// Minimum-jerk solution for boundary conditions
///   p(0)=0, v(0)=0, a(0)=0, p(1)=1, v(1)=0, a(1)=0
/// gives the normalised polynomial:
///   p(s) = 10s³ - 15s⁴ + 6s⁵
pub struct TakeoffTrajectory {
    /// X/Y position to hold throughout the ascent
    pub start_x: f32,
    pub start_y: f32,
    /// Target height [m]
    pub target_z: f32,
    /// Duration of the takeoff phase [s]
    pub duration: f32,
    /// Yaw to hold throughout the ascent [rad]
    pub yaw: f32,
}

impl TakeoffTrajectory {
    pub fn new(start_x: f32, start_y: f32, target_z: f32, duration: f32, yaw: f32) -> Self {
        Self { start_x, start_y, target_z, duration, yaw }
    }
}

impl Trajectory for TakeoffTrajectory {
    fn get_reference(&self, time: f32) -> TrajectoryReference {
        // Clamp s to [0, 1]
        let s = (time / self.duration).clamp(0.0, 1.0);
        let h = self.target_z;
        let t_dur = self.duration;

        // Minimum-jerk polynomial and its derivatives w.r.t. s
        //   p(s)   = 10s³ - 15s⁴ + 6s⁵
        //   p'(s)  = 30s² - 60s³ + 30s⁴
        //   p''(s) = 60s  - 180s² + 120s³
        //   p'''(s)= 60   - 360s  + 360s²
        let p   =  10.0*s.powi(3) - 15.0*s.powi(4) +  6.0*s.powi(5);
        let dp  =  30.0*s.powi(2) - 60.0*s.powi(3) + 30.0*s.powi(4);
        let ddp =  60.0*s         -180.0*s.powi(2)  +120.0*s.powi(3);
        let dddp = 60.0           -360.0*s           +360.0*s.powi(2);

        // Chain rule: d/dt = d/ds * ds/dt = (1/T) * d/ds
        let z    = h * p;
        let vz   = h * dp   / t_dur;
        let az   = h * ddp  / (t_dur * t_dur);
        let jz   = h * dddp / (t_dur * t_dur * t_dur);

        TrajectoryReference {
            position:     Vec3::new(self.start_x, self.start_y, z),
            velocity:     Vec3::new(0.0, 0.0, vz),
            acceleration: Vec3::new(0.0, 0.0, az),
            jerk:         Vec3::new(0.0, 0.0, jz),
            yaw:          self.yaw,
            yaw_rate:     0.0,
            yaw_acceleration: 0.0,
        }
    }

    fn duration(&self) -> Option<f32> {
        Some(self.duration)
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// SequencedTrajectory
// ─────────────────────────────────────────────────────────────────────────────

/// Chains multiple trajectory phases back-to-back.
///
/// Each phase is described by a duration and a boxed `Trajectory`.  At any
/// simulation time the active phase is selected and `get_reference` is
/// forwarded to it with the *local* phase time (i.e. time since that phase
/// started).  Once the last phase is reached it is held indefinitely.
///
/// Example:
/// ```
/// let seq = SequencedTrajectory::new(vec![
///     (3.0, Box::new(TakeoffTrajectory::new(0.0, 0.0, 0.5, 3.0, 0.0))),
///     (2.0, Box::new(CircleTrajectory::new(0.0, 0.5, 0.0))),  // hover
///     (20.0, Box::new(CircleTrajectory::new(0.3, 0.3, PI/5.0))),
/// ]);
/// ```
pub struct SequencedTrajectory {
    /// (phase_duration, trajectory) pairs in execution order
    phases: Vec<(f32, Box<dyn Trajectory>)>,
    /// Cumulative start times cached at construction for fast lookup
    phase_starts: Vec<f32>,
    /// Total duration (sum of all phase durations)
    total_duration: f32,
}

impl SequencedTrajectory {
    /// Create a new sequence.  `phases` is a list of `(duration, trajectory)`.
    pub fn new(phases: Vec<(f32, Box<dyn Trajectory>)>) -> Self {
        let mut starts = Vec::with_capacity(phases.len());
        let mut t = 0.0_f32;
        for (dur, _) in &phases {
            starts.push(t);
            t += dur;
        }
        let total = t;
        Self { phases, phase_starts: starts, total_duration: total }
    }

    /// Returns the index of the active phase and the local time within it.
    pub fn active_phase(&self, time: f32) -> (usize, f32) {
        let mut idx = 0;
        for (i, &start) in self.phase_starts.iter().enumerate() {
            if time >= start {
                idx = i;
            } else {
                break;
            }
        }
        let local_t = time - self.phase_starts[idx];
        (idx, local_t)
    }

    /// How many seconds of prefix phases occur before the given phase index.
    /// Useful for plotting to know when the "real" trajectory phase starts.
    pub fn phase_start_time(&self, phase_idx: usize) -> f32 {
        self.phase_starts.get(phase_idx).copied().unwrap_or(self.total_duration)
    }
}

impl Trajectory for SequencedTrajectory {
    fn get_reference(&self, time: f32) -> TrajectoryReference {
        let (idx, local_t) = self.active_phase(time);
        self.phases[idx].1.get_reference(local_t)
    }

    fn duration(&self) -> Option<f32> {
        Some(self.total_duration)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_takeoff_trajectory() {
        let traj = TakeoffTrajectory::new(0.1, 0.2, 0.5, 3.0, 0.0);
        let start = traj.get_reference(0.0);
        let end   = traj.get_reference(3.0);
        // Starts on the ground
        assert!(start.position.z.abs() < 1e-5, "z at t=0 should be 0");
        assert!(start.velocity.z.abs() < 1e-5, "vz at t=0 should be 0");
        // Arrives at target height with zero velocity
        assert!((end.position.z - 0.5).abs() < 1e-4, "z at t=T should be target");
        assert!(end.velocity.z.abs() < 1e-4, "vz at t=T should be 0");
        // X/Y held constant
        assert!((start.position.x - 0.1).abs() < 1e-6);
        assert!((start.position.y - 0.2).abs() < 1e-6);
    }

    #[test]
    fn test_sequenced_trajectory() {
        let seq = SequencedTrajectory::new(vec![
            (3.0, Box::new(TakeoffTrajectory::new(0.0, 0.0, 0.5, 3.0, 0.0))),
            (2.0, Box::new(CircleTrajectory::new(0.0, 0.5, 0.0))), // hover
        ]);
        assert_eq!(seq.phase_start_time(0), 0.0);
        assert_eq!(seq.phase_start_time(1), 3.0);
        // At t=0 we're in takeoff, z=0
        let r0 = seq.get_reference(0.0);
        assert!(r0.position.z < 0.01);
        // At t=3 we're in hover, z=0.5
        let r3 = seq.get_reference(3.0);
        assert!((r3.position.z - 0.5).abs() < 1e-3);
    }

    #[test]
    fn test_figure8_trajectory() {
        let traj = Figure8Trajectory::new();
        let reference = traj.get_reference(0.0);

        // Should start at origin-ish
        assert!(reference.position.x.abs() < 0.1);
        assert!(reference.position.y.abs() < 0.1);
        assert!((reference.position.z - 0.5).abs() < 0.1);

        // after a couple seconds the trajectory should have moved significantly
        let later = traj.get_reference(2.0);
        let mag = (later.position.x.powi(2) + later.position.y.powi(2)).sqrt();
        // size is scaled by default so expect at least a few tenths of a meter
        assert!(mag > 0.3, "figure-8 too small: {}", mag);
    }

    #[test]
    fn test_circle_trajectory() {
        let traj = CircleTrajectory::new(1.0, 0.5, 1.0);
        let reference = traj.get_reference(0.0);

        // Should start at (1, 0, 0.5)
        assert!((reference.position.x - 1.0).abs() < 1e-6);
        assert!(reference.position.y.abs() < 1e-6);
        assert!((reference.position.z - 0.5).abs() < 1e-6);
    }

    #[test]
    fn test_csv_trajectory() {
        let waypoints = vec![
            (0.0, 0.0, 0.0, 0.0, 0.0),
            (1.0, 1.0, 0.0, 0.0, 0.0),
        ];
        let traj = CsvTrajectory::from_waypoints(waypoints);

        let ref_start = traj.get_reference(0.0);
        let ref_mid = traj.get_reference(0.5);
        let ref_end = traj.get_reference(1.0);

        assert_eq!(ref_start.position.x, 0.0);
        assert_eq!(ref_end.position.x, 1.0);
        assert_eq!(ref_mid.position.x, 0.5);
    }
}