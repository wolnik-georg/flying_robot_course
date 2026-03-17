//! Core measurement and image types for the perception module.
//!
//! All types are plain-old-data structs with no hardware dependency.
//! The same types flow through both simulation and real-hardware paths.

// ---------------------------------------------------------------------------
// Sensor measurement types
// ---------------------------------------------------------------------------

/// Optical flow measurement from the PMW3901 sensor.
///
/// `dx_px` and `dy_px` are integrated pixel displacements since the last
/// measurement, in the sensor's pixel coordinate frame (body X ↔ −dy,
/// body Y ↔ −dx — see `CrtpFlowAdapter` for the sign convention).
#[derive(Debug, Clone, Copy)]
pub struct FlowMeasurement {
    /// Pixel displacement along sensor x-axis since last measurement.
    pub dx_px: f32,
    /// Pixel displacement along sensor y-axis since last measurement.
    pub dy_px: f32,
    /// Time elapsed since last measurement [s].
    pub dt_s:  f32,
}

/// Single downward-facing range measurement (Flow Deck / Z Ranger ToF).
#[derive(Debug, Clone, Copy)]
pub struct RangeMeasurement {
    /// Range to surface below the drone [m].
    pub range_m: f32,
}

/// Six-axis range from Multi-ranger Deck combined with the Flow Deck down sensor.
///
/// Each field is `None` when:
/// - The sensor returns an out-of-range value (> 4.0 m or saturated), or
/// - The corresponding sensor is not present (e.g. up sensor on Z Ranger only config).
///
/// Axis convention: +X body = forward (front), +Y body = left, +Z body = up.
#[derive(Debug, Clone, Copy, Default)]
pub struct MultiRangeMeasurement {
    /// Distance to obstacle in +X (forward) body direction [m].
    pub front_m: Option<f32>,
    /// Distance to obstacle in −X (backward) body direction [m].
    pub back_m:  Option<f32>,
    /// Distance to obstacle in +Y (left) body direction [m].
    pub left_m:  Option<f32>,
    /// Distance to obstacle in −Y (right) body direction [m].
    pub right_m: Option<f32>,
    /// Distance to obstacle in +Z (upward) body direction [m].
    pub up_m:    Option<f32>,
    /// Distance to floor in −Z (downward) body direction [m].
    /// Same physical sensor as the Flow Deck range sensor when combined.
    pub down_m:  Option<f32>,
}

// ---------------------------------------------------------------------------
// Camera types
// ---------------------------------------------------------------------------

/// Pinhole camera intrinsics with Brown-Conrady radial distortion.
///
/// Undistorted image coordinates:
/// ```text
/// u = fx * (x / z) * (1 + k1*r² + k2*r⁴) + cx
/// v = fy * (y / z) * (1 + k1*r² + k2*r⁴) + cy
/// ```
/// where `r² = ((x/z)² + (y/z)²)`.
#[derive(Debug, Clone, Copy)]
pub struct CameraIntrinsics {
    /// Horizontal focal length [pixels].
    pub fx: f32,
    /// Vertical focal length [pixels].
    pub fy: f32,
    /// Principal point column [pixels].
    pub cx: f32,
    /// Principal point row [pixels].
    pub cy: f32,
    /// First radial distortion coefficient.
    pub k1: f32,
    /// Second radial distortion coefficient.
    pub k2: f32,
}

/// A single grayscale image frame from the AI Deck (or simulation).
#[derive(Debug, Clone)]
pub struct ImageFrame {
    /// Image width [pixels].
    pub width:        u16,
    /// Image height [pixels].
    pub height:       u16,
    /// Row-major 8-bit grayscale pixel data, length = `width × height`.
    pub pixels:       Vec<u8>,
    /// Capture timestamp [ms] since some arbitrary epoch.
    pub timestamp_ms: u64,
}

impl ImageFrame {
    /// Return the pixel value at (row, col). Returns 0 for out-of-bounds.
    #[inline]
    pub fn pixel(&self, row: usize, col: usize) -> u8 {
        if row < self.height as usize && col < self.width as usize {
            self.pixels[row * self.width as usize + col]
        } else {
            0
        }
    }

    /// Return the pixel value at (row, col) using signed coordinates.
    /// Returns 0 for out-of-bounds.
    #[inline]
    pub fn pixel_i(&self, row: i32, col: i32) -> u8 {
        if row >= 0 && col >= 0 {
            self.pixel(row as usize, col as usize)
        } else {
            0
        }
    }
}

/// A detected image feature with position, score, and BRIEF-256 descriptor.
#[derive(Debug, Clone)]
pub struct Feature {
    /// Column coordinate in image [pixels].
    pub x:          f32,
    /// Row coordinate in image [pixels].
    pub y:          f32,
    /// FAST corner response score (higher = stronger corner).
    pub score:      f32,
    /// BRIEF-256 descriptor packed as 32 bytes (256 bits).
    pub descriptor: [u8; 32],
}

impl Feature {
    /// Hamming distance between two BRIEF-256 descriptors.
    #[inline]
    pub fn hamming_distance(&self, other: &Feature) -> u32 {
        self.descriptor.iter()
            .zip(other.descriptor.iter())
            .map(|(a, b)| (a ^ b).count_ones())
            .sum()
    }
}
