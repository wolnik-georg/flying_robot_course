//! Simulation sensor models.
//!
//! All sensors are driven from `MultirotorState` and add deterministic
//! (reproducible) Gaussian noise via a seeded LCG.  They implement the same
//! measurement models as the MEKF update functions so that sim → MEKF
//! round-trips are numerically consistent.
//!
//! None of these types carry `&mut MultirotorState`; the caller passes the
//! state each time it calls `measure()` / `render()`.

use crate::dynamics::MultirotorState;
use crate::math::Quat;
use crate::perception::types::{
    FlowMeasurement, RangeMeasurement, MultiRangeMeasurement,
    ImageFrame, CameraIntrinsics,
};
use crate::perception::traits::{SensorSource, ImageSource};

// Flow calibration constants — must match MEKF (src/estimation/mekf.rs).
const NP:      f32 = 350.0;
const THETA_P: f32 = 3.50;

// VL53L1x physical range limits [m].
const RANGE_MIN_M: f32 = 0.02;
const RANGE_MAX_M: f32 = 4.0;

// ---------------------------------------------------------------------------
// Tiny deterministic LCG noise generator (no external deps).
// ---------------------------------------------------------------------------

struct Lcg(u64);

impl Lcg {
    fn new(seed: u64) -> Self { Self(seed) }

    fn next_u64(&mut self) -> u64 {
        self.0 = self.0
            .wrapping_mul(6_364_136_223_846_793_005)
            .wrapping_add(1_442_695_040_888_963_407);
        self.0
    }

    /// Approximate standard-normal sample via Box-Muller (single output).
    fn next_normal(&mut self) -> f32 {
        // u1 ∈ (0, 1], u2 ∈ [0, 2π)
        let u1 = (self.next_u64() >> 11) as f32 / (1u64 << 53) as f32;
        let u1 = u1.clamp(1e-7, 1.0);
        let u2 = (self.next_u64() >> 11) as f32 / (1u64 << 53) as f32
                 * 2.0 * std::f32::consts::PI;
        (-2.0 * u1.ln()).sqrt() * u2.cos()
    }
}

// ---------------------------------------------------------------------------
// Quaternion helpers (local, using the codebase's [w, x, y, z] convention).
// ---------------------------------------------------------------------------

/// Rotate world-frame vector `v` to body frame using the body-to-world
/// quaternion `q`.  Equivalent to `q_conj * [0, v] * q`.
fn rotate_world_to_body(q: Quat, v: [f32; 3]) -> [f32; 3] {
    // Rodrigues formula with q_conj (negate vector part):
    //   v' = v − 2w*(q_vec × v) + 2*(q_vec × (q_vec × v))
    // where q_vec = (−qx, −qy, −qz) for q_conj.
    // Simplifying: v' = v + 2w*(qvec × v) + 2*(qvec × (qvec × v))
    // with qvec = (−qx, −qy, −qz)  →  same as rotating by q then negating cross signs.
    // Direct Rodrigues with q_conj:
    let (qw, qx, qy, qz) = (q.w, -q.x, -q.y, -q.z);  // q_conj
    let [vx, vy, vz] = v;

    let t_x = qy * vz - qz * vy;
    let t_y = qz * vx - qx * vz;
    let t_z = qx * vy - qy * vx;

    [
        vx + 2.0 * qw * t_x + 2.0 * (qy * t_z - qz * t_y),
        vy + 2.0 * qw * t_y + 2.0 * (qz * t_x - qx * t_z),
        vz + 2.0 * qw * t_z + 2.0 * (qx * t_y - qy * t_x),
    ]
}

/// Rotate body-frame vector `v` to world frame using the body-to-world
/// quaternion `q`.  Equivalent to `q * [0, v] * q_conj`.
fn rotate_body_to_world(q: Quat, v: [f32; 3]) -> [f32; 3] {
    let (qw, qx, qy, qz) = (q.w, q.x, q.y, q.z);
    let [vx, vy, vz] = v;

    let t_x = qy * vz - qz * vy;
    let t_y = qz * vx - qx * vz;
    let t_z = qx * vy - qy * vx;

    [
        vx + 2.0 * qw * t_x + 2.0 * (qy * t_z - qz * t_y),
        vy + 2.0 * qw * t_y + 2.0 * (qz * t_x - qx * t_z),
        vz + 2.0 * qw * t_z + 2.0 * (qx * t_y - qy * t_x),
    ]
}

// ---------------------------------------------------------------------------
// SimFlowSensor
// ---------------------------------------------------------------------------

/// Synthetic PMW3901 optical flow sensor.
///
/// Computes expected pixel displacement from body-frame velocity and height,
/// then adds Gaussian noise.  Reproduces the MEKF flow forward model exactly:
///
/// ```text
/// dx_px = (v_body_x / height_m) × (dt × NP / THETA_P)  +  N(0, σ)
/// dy_px = (v_body_y / height_m) × (dt × NP / THETA_P)  +  N(0, σ)
/// ```
///
/// Constants `NP = 350`, `THETA_P = 3.50` match `src/estimation/mekf.rs`.
pub struct SimFlowSensor {
    dt_s:         f32,
    noise_stddev: f32,
    rng:          Lcg,
}

impl SimFlowSensor {
    /// Create a new simulated flow sensor.
    ///
    /// - `dt_s`:         nominal measurement period [s] (e.g. 0.01 for 100 Hz)
    /// - `noise_stddev`: standard deviation of the additive pixel noise [px]
    pub fn new(dt_s: f32, noise_stddev: f32) -> Self {
        Self { dt_s, noise_stddev, rng: Lcg::new(12345) }
    }

    /// Produce a measurement from the current simulator state.
    pub fn measure(&mut self, state: &MultirotorState) -> FlowMeasurement {
        let height = state.position.z.max(0.05); // clamp to avoid division by zero
        let dt     = self.dt_s;
        let scale  = dt * NP / (height * THETA_P);

        // Rotate world velocity to body frame.
        let v_world = [state.velocity.x, state.velocity.y, state.velocity.z];
        let v_body  = rotate_world_to_body(state.orientation, v_world);

        let dx = v_body[0] * scale + self.rng.next_normal() * self.noise_stddev;
        let dy = v_body[1] * scale + self.rng.next_normal() * self.noise_stddev;

        FlowMeasurement { dx_px: dx, dy_px: dy, dt_s: dt }
    }
}

/// The `SensorSource` impl returns `None`; use `measure()` directly to pass
/// the drone state in.
impl SensorSource<FlowMeasurement> for SimFlowSensor {
    fn poll(&mut self) -> Option<FlowMeasurement> { None }
}

// ---------------------------------------------------------------------------
// SimRangeSensor
// ---------------------------------------------------------------------------

/// Synthetic VL53L1x downward range sensor.
///
/// Returns `height / cos(tilt)` with Gaussian noise, clamped to [0.02, 4.0] m.
///
/// The tilt correction comes from R₃₃ = 1 − 2(qx² + qy²), the world-z
/// component of the body z-axis expressed in world coordinates.
pub struct SimRangeSensor {
    noise_stddev: f32,
    rng:          Lcg,
}

impl SimRangeSensor {
    /// Create a new simulated range sensor.
    ///
    /// `noise_stddev`: standard deviation of the additive distance noise [m]
    pub fn new(noise_stddev: f32) -> Self {
        Self { noise_stddev, rng: Lcg::new(67890) }
    }

    /// Produce a measurement from the current simulator state.
    pub fn measure(&mut self, state: &MultirotorState) -> RangeMeasurement {
        let q = state.orientation;
        // R[2][2] = 1 − 2(qx² + qy²)  =  w²−x²−y²+z²  (for unit quaternion).
        // This is the cosine of the tilt angle.
        let cos_tilt = (1.0 - 2.0 * (q.x * q.x + q.y * q.y)).abs().max(0.05);
        let raw   = state.position.z / cos_tilt;
        let noisy = (raw + self.rng.next_normal() * self.noise_stddev)
                        .clamp(RANGE_MIN_M, RANGE_MAX_M);
        RangeMeasurement { range_m: noisy }
    }
}

impl SensorSource<RangeMeasurement> for SimRangeSensor {
    fn poll(&mut self) -> Option<RangeMeasurement> { None }
}

// ---------------------------------------------------------------------------
// SimMultiRangeSensor
// ---------------------------------------------------------------------------

/// Synthetic Multi-ranger Deck: 6-axis VL53L1x sensors.
///
/// Takes the drone's current state and axis-aligned world-frame obstacle
/// distances.  For each of the 6 body-axis directions, the sensor ray is
/// rotated to world frame and the distance to the obstacle plane is computed
/// accounting for tilt.
///
/// `obstacle_distances`: `[front, back, left, right, up, down]`
///   in world-frame axis-aligned distances [m].
///   Use `f32::INFINITY` or any value > 4.0 for no obstacle.
pub struct SimMultiRangeSensor {
    noise_stddev: f32,
    rng:          Lcg,
}

impl SimMultiRangeSensor {
    /// Create a new simulated multi-range sensor.
    pub fn new(noise_stddev: f32) -> Self {
        Self { noise_stddev, rng: Lcg::new(11111) }
    }

    /// Compute six-axis range measurement.
    ///
    /// `obstacle_distances[i]`: world-axis-aligned distance to nearest obstacle.
    ///   Index 0 = +X (front), 1 = −X (back), 2 = +Y (left),
    ///   3 = −Y (right), 4 = +Z (up), 5 = −Z (down).
    pub fn measure(
        &mut self,
        state: &MultirotorState,
        obstacle_distances: [f32; 6],
    ) -> MultiRangeMeasurement {
        // Body-frame axis unit vectors for the 6 sensors.
        let body_axes: [[f32; 3]; 6] = [
            [ 1.0,  0.0,  0.0],  // front  (+X body)
            [-1.0,  0.0,  0.0],  // back   (−X body)
            [ 0.0,  1.0,  0.0],  // left   (+Y body)
            [ 0.0, -1.0,  0.0],  // right  (−Y body)
            [ 0.0,  0.0,  1.0],  // up     (+Z body)
            [ 0.0,  0.0, -1.0],  // down   (−Z body)
        ];
        // World-axis index for each sensor direction (0=X, 1=Y, 2=Z).
        let world_axis: [usize; 6] = [0, 0, 1, 1, 2, 2];

        let mut raw = [None::<f32>; 6];

        for i in 0..6 {
            let d_world = obstacle_distances[i];
            if d_world > RANGE_MAX_M {
                raw[i] = None;
                continue;
            }
            // Rotate body sensor axis to world frame.
            let world_dir = rotate_body_to_world(state.orientation, body_axes[i]);
            // cos factor: projection of the sensor ray onto the world axis.
            let cos_factor = world_dir[world_axis[i]].abs().max(0.05);
            // Sensor reading = world-axis obstacle distance / cos(angle).
            let range = d_world / cos_factor;
            let noisy = (range + self.rng.next_normal() * self.noise_stddev)
                            .clamp(RANGE_MIN_M, RANGE_MAX_M);
            raw[i] = Some(noisy);
        }

        MultiRangeMeasurement {
            front_m: raw[0],
            back_m:  raw[1],
            left_m:  raw[2],
            right_m: raw[3],
            up_m:    raw[4],
            down_m:  raw[5],
        }
    }
}

// ---------------------------------------------------------------------------
// SimCamera
// ---------------------------------------------------------------------------

/// Synthetic AI Deck grayscale camera.
///
/// Projects a list of world-frame landmark points through the pinhole camera
/// model and returns a grayscale `ImageFrame` with bright 3×3 spots at the
/// projected positions (background = 30, landmark = 220).
///
/// Uses default HiMax HM01B0 intrinsics (320×320, fx = fy = 164 px) unless
/// explicitly overridden via `new()`.
pub struct SimCamera {
    intrinsics:   CameraIntrinsics,
    width:        u16,
    height:       u16,
    timestamp_ms: u64,
}

impl SimCamera {
    /// Create a new simulated camera with explicit intrinsics and resolution.
    pub fn new(intrinsics: CameraIntrinsics, width: u16, height: u16) -> Self {
        Self { intrinsics, width, height, timestamp_ms: 0 }
    }

    /// Create with default HiMax HM01B0 intrinsics (320×320 grayscale).
    pub fn default_hm01b0() -> Self {
        Self::new(
            CameraIntrinsics {
                fx: 164.0, fy: 164.0,
                cx: 160.0, cy: 160.0,
                k1: 0.0,   k2: 0.0,
            },
            320, 320,
        )
    }

    /// Render a frame from the given drone state and world landmarks.
    ///
    /// `state`:     current drone pose.
    /// `landmarks`: world-frame `[x, y, z]` point positions.
    /// `step_ms`:   elapsed time since last render [ms] — advances the timestamp.
    pub fn render(
        &mut self,
        state:     &MultirotorState,
        landmarks: &[[f32; 3]],
        step_ms:   u64,
    ) -> ImageFrame {
        self.timestamp_ms += step_ms;

        let mut pixels = vec![30u8; self.width as usize * self.height as usize];

        let q   = state.orientation;
        let pos = state.position;

        for lm in landmarks {
            // Translate: vector from drone to landmark in world frame.
            let p_world = [lm[0] - pos.x, lm[1] - pos.y, lm[2] - pos.z];

            // Rotate to body frame.
            let p_body = rotate_world_to_body(q, p_world);

            // Camera frame convention: +Z forward, +X right, +Y down.
            // Body frame: +X forward, +Y left, +Z up.
            // Camera ← Body: cam_x = body_y (right), cam_y = −body_z (down), cam_z = body_x
            let cam_x = p_body[1];   //  +Y_body → +X_cam (left → right)
            let cam_y = -p_body[2];  //  −Z_body → +Y_cam (up → down in image)
            let cam_z = p_body[0];   //  +X_body → +Z_cam (forward)

            if cam_z <= 0.01 { continue; } // behind camera

            let u = self.intrinsics.fx * cam_x / cam_z + self.intrinsics.cx;
            let v = self.intrinsics.fy * cam_y / cam_z + self.intrinsics.cy;

            let col = u.round() as i32;
            let row = v.round() as i32;

            // Draw 3×3 bright spot.
            for dr in -1i32..=1 {
                for dc in -1i32..=1 {
                    let r2 = row + dr;
                    let c2 = col + dc;
                    if r2 >= 0 && r2 < self.height as i32
                    && c2 >= 0 && c2 < self.width as i32 {
                        pixels[r2 as usize * self.width as usize + c2 as usize] = 220;
                    }
                }
            }
        }

        ImageFrame {
            width:        self.width,
            height:       self.height,
            pixels,
            timestamp_ms: self.timestamp_ms,
        }
    }
}

impl ImageSource for SimCamera {
    /// Not used — call `render()` directly with the drone state.
    fn next_frame(&mut self) -> Option<ImageFrame> { None }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use crate::dynamics::MultirotorState;
    use crate::math::{Vec3, Quat};

    fn level_hover_state(height: f32, vx: f32, vy: f32) -> MultirotorState {
        let mut s = MultirotorState::new();
        s.position = Vec3::new(0.0, 0.0, height);
        s.velocity = Vec3::new(vx, vy, 0.0);
        s.orientation = Quat::identity();
        s
    }

    // ------------------------------------------------------------------
    // SimFlowSensor
    // ------------------------------------------------------------------

    #[test]
    fn sim_flow_reproduces_mekf_model() {
        // At height 1 m, velocity (vx=1.0, vy=0), dt=0.01:
        // expected dx_px = (1.0 / 1.0) * (0.01 * 350 / 3.50) = 1.0 px
        // expected dy_px = 0
        let state = level_hover_state(1.0, 1.0, 0.0);
        let mut sensor = SimFlowSensor::new(0.01, 0.0); // zero noise for exact check
        let m = sensor.measure(&state);

        assert!((m.dx_px - 1.0).abs() < 1e-4,
            "expected dx≈1.0 px, got {}", m.dx_px);
        assert!(m.dy_px.abs() < 1e-4,
            "expected dy≈0 px, got {}", m.dy_px);
    }

    #[test]
    fn sim_flow_scales_with_height() {
        // At 2 m height the same velocity gives half the pixel displacement.
        let s1 = level_hover_state(1.0, 1.0, 0.0);
        let s2 = level_hover_state(2.0, 1.0, 0.0);
        let mut sensor = SimFlowSensor::new(0.01, 0.0);
        let m1 = sensor.measure(&s1);
        let m2 = sensor.measure(&s2);
        assert!((m1.dx_px / m2.dx_px - 2.0).abs() < 1e-3,
            "height doubling should halve flow: {:.4} / {:.4}", m1.dx_px, m2.dx_px);
    }

    // ------------------------------------------------------------------
    // SimRangeSensor
    // ------------------------------------------------------------------

    #[test]
    fn sim_range_level_equals_height() {
        let state = level_hover_state(0.5, 0.0, 0.0);
        let mut sensor = SimRangeSensor::new(0.0);
        let m = sensor.measure(&state);
        assert!((m.range_m - 0.5).abs() < 1e-4,
            "level hover: range should equal height");
    }

    #[test]
    fn sim_range_cos_tilt_correction() {
        // Roll 30° → cos(30°) ≈ 0.866 → range = height / 0.866
        let roll_rad = 30.0_f32.to_radians();
        let q = Quat::from_axis_angle(Vec3::new(1.0, 0.0, 0.0), roll_rad);
        let mut state = MultirotorState::new();
        state.position = Vec3::new(0.0, 0.0, 0.5);
        state.orientation = q;

        let mut sensor = SimRangeSensor::new(0.0);
        let m = sensor.measure(&state);

        let expected = 0.5 / roll_rad.cos();
        assert!((m.range_m - expected).abs() < 0.01,
            "tilt correction: expected {:.4} m, got {:.4} m", expected, m.range_m);
    }

    // ------------------------------------------------------------------
    // SimMultiRangeSensor
    // ------------------------------------------------------------------

    #[test]
    fn sim_multi_range_all_axes() {
        let state = level_hover_state(1.0, 0.0, 0.0);
        let obstacles = [2.0, 2.0, 3.0, 3.0, 1.5, 1.0]; // [front,back,left,right,up,down]
        let mut sensor = SimMultiRangeSensor::new(0.0);
        let m = sensor.measure(&state, obstacles);

        assert!((m.front_m.unwrap() - 2.0).abs() < 0.01, "front");
        assert!((m.back_m.unwrap()  - 2.0).abs() < 0.01, "back");
        assert!((m.left_m.unwrap()  - 3.0).abs() < 0.01, "left");
        assert!((m.right_m.unwrap() - 3.0).abs() < 0.01, "right");
        assert!((m.up_m.unwrap()    - 1.5).abs() < 0.01, "up");
        assert!((m.down_m.unwrap()  - 1.0).abs() < 0.01, "down");
    }

    #[test]
    fn sim_multi_range_no_obstacle_is_none() {
        let state = level_hover_state(1.0, 0.0, 0.0);
        let obstacles = [f32::INFINITY; 6];
        let mut sensor = SimMultiRangeSensor::new(0.0);
        let m = sensor.measure(&state, obstacles);

        assert!(m.front_m.is_none(), "front should be None (no obstacle)");
        assert!(m.back_m.is_none(),  "back should be None");
    }

    // ------------------------------------------------------------------
    // SimCamera
    // ------------------------------------------------------------------

    #[test]
    fn sim_camera_projects_landmark() {
        // Landmark directly in front of the drone at (1 m forward, 0 m lateral, 0 m vertical).
        // With default HM01B0 intrinsics (fx=fy=164, cx=cy=160, 320×320) it should
        // project near the image centre.
        let state = level_hover_state(0.0, 0.0, 0.0); // drone at origin, level
        let landmarks = [[1.0, 0.0, 0.0]];
        let mut cam = SimCamera::default_hm01b0();
        let frame = cam.render(&state, &landmarks, 10);

        assert_eq!(frame.width, 320);
        assert_eq!(frame.height, 320);
        // Centre pixels should be bright (landmark projects near 160, 160).
        assert!(frame.pixel(160, 160) > 100 || frame.pixel(159, 160) > 100
             || frame.pixel(160, 159) > 100,
             "expected bright pixel near image centre");
    }

    #[test]
    fn sim_camera_behind_is_blank() {
        // Landmark directly behind the drone — should not appear.
        let state = level_hover_state(0.0, 0.0, 0.0);
        let landmarks = [[-1.0, 0.0, 0.0]];
        let mut cam = SimCamera::default_hm01b0();
        let frame = cam.render(&state, &landmarks, 10);
        let max_pixel = *frame.pixels.iter().max().unwrap();
        assert_eq!(max_pixel, 30, "behind-camera landmark should not project");
    }
}
