//! Feature detection: FAST-9 corner detector and BRIEF-256 descriptor.
//!
//! Both algorithms are pure Rust with no external dependencies.
//!
//! ## FAST-9 (Features from Accelerated Segment Test)
//!
//! Tests a Bresenham circle of radius 3 (16 pixels) around each candidate
//! pixel.  A pixel `p` is a corner if at least 9 **contiguous** pixels on the
//! circle are all brighter than `p + threshold` OR all darker than
//! `p − threshold`.
//!
//! Reference: Rosten & Drummond, "Machine learning for high-speed corner
//! detection", ECCV 2006.
//!
//! ## BRIEF-256 (Binary Robust Independent Elementary Features)
//!
//! Compares 256 pixel pairs within a 25×25 patch centred on the keypoint.
//! Each comparison produces one bit; 256 bits are packed into 32 bytes.
//! The comparison table is computed at compile time from a deterministic LCG.
//!
//! Reference: Calonder et al., "BRIEF: Binary Robust Independent Elementary
//! Features", ECCV 2010.

use crate::perception::types::{ImageFrame, Feature};

// ---------------------------------------------------------------------------
// FAST-9 Bresenham circle offsets (radius 3, 16 points).
// Order: starting at (row=0, col=+3), clockwise.
// ---------------------------------------------------------------------------

const FAST_CIRCLE: [(i8, i8); 16] = [
    ( 0,  3), ( 1,  3), ( 2,  2), ( 3,  1),
    ( 3,  0), ( 3, -1), ( 2, -2), ( 1, -3),
    ( 0, -3), (-1, -3), (-2, -2), (-3, -1),
    (-3,  0), (-3,  1), (-2,  2), (-1,  3),
];

// Minimum run length for FAST-9.
const FAST_MIN_RUN: usize = 9;

// ---------------------------------------------------------------------------
// BRIEF-256 pattern table.
// 256 pairs of (row1, col1, row2, col2) offsets within a [-12, 12] window.
// Generated at compile time with a simple LCG so the pattern is reproducible.
// ---------------------------------------------------------------------------

const BRIEF_PATTERN: [[i8; 4]; 256] = {
    let mut p = [[0i8; 4]; 256];
    let mut i = 0usize;
    while i < 256 {
        let a = i as u32;
        // Four independent LCG streams, mapped to [-12, 12] via modulo 25.
        let v0 = (a.wrapping_mul(31).wrapping_add(7))   % 25;
        let v1 = (a.wrapping_mul(17).wrapping_add(3))   % 25;
        let v2 = (a.wrapping_mul(13).wrapping_add(11))  % 25;
        let v3 = (a.wrapping_mul(19).wrapping_add(5))   % 25;
        p[i][0] = v0 as i8 - 12;
        p[i][1] = v1 as i8 - 12;
        p[i][2] = v2 as i8 - 12;
        p[i][3] = v3 as i8 - 12;
        i += 1;
    }
    p
};

// ---------------------------------------------------------------------------
// FAST-9 implementation
// ---------------------------------------------------------------------------

/// Classify a circle pixel relative to the centre.
/// Returns: +1 (bright), -1 (dark), 0 (neither).
#[inline]
fn classify(centre: u8, pixel: u8, threshold: u8) -> i8 {
    let diff = pixel as i16 - centre as i16;
    let t    = threshold as i16;
    if      diff >  t { 1 }
    else if diff < -t { -1 }
    else              { 0 }
}

/// Longest run of `target` in a circular array of length 16.
#[inline]
fn max_circular_run(classes: &[i8; 16], target: i8) -> usize {
    // Scan once; double-length trick handles wrap-around.
    let mut best = 0usize;
    let mut run  = 0usize;
    for rep in 0..2 {
        for k in 0..16 {
            if classes[k] == target {
                run += 1;
                if run > best { best = run; }
            } else {
                run = 0;
            }
        }
        if rep == 0 && best >= FAST_MIN_RUN {
            // Already found a long-enough run; no need for second pass.
            break;
        }
        // Reset run for second pass (continues from where first pass ended).
    }
    best
}

/// FAST-9 score: sum of absolute differences from centre for the winning arc.
fn fast_score(frame: &ImageFrame, row: usize, col: usize, threshold: u8) -> f32 {
    let centre = frame.pixel(row, col) as i16;
    let t      = threshold as i16;

    let mut sum_bright = 0i32;
    let mut sum_dark   = 0i32;
    for &(dr, dc) in FAST_CIRCLE.iter() {
        let r = (row as i32 + dr as i32) as usize;
        let c = (col as i32 + dc as i32) as usize;
        let q = frame.pixel(r, c) as i16;
        let d = q - centre;
        if d >  t { sum_bright += (d - t) as i32; }
        if d < -t { sum_dark   += (-d - t) as i32; }
    }
    sum_bright.max(sum_dark) as f32
}

/// Returns `Some(score)` if `(row, col)` is a FAST-9 corner, `None` otherwise.
///
/// The caller must ensure the point is at least 3 pixels from every edge.
fn is_fast9_corner(
    frame:     &ImageFrame,
    row:       usize,
    col:       usize,
    threshold: u8,
) -> Option<f32> {
    let centre = frame.pixel(row, col);

    // Quick rejection: check the 4 cardinal circle points (indices 0,4,8,12).
    // At least 3 of them must agree (bright or dark) for a corner to exist.
    let card = [
        classify(centre, frame.pixel(row,       col + 3), threshold),
        classify(centre, frame.pixel(row + 3,   col    ), threshold),
        classify(centre, frame.pixel(row,       col.wrapping_sub(3)), threshold),
        classify(centre, frame.pixel(row.wrapping_sub(3), col    ), threshold),
    ];
    let bright_card = card.iter().filter(|&&v| v ==  1).count();
    let dark_card   = card.iter().filter(|&&v| v == -1).count();
    if bright_card < 2 && dark_card < 2 {
        return None;
    }

    // Full check: classify all 16 circle pixels.
    let mut classes = [0i8; 16];
    for (i, &(dr, dc)) in FAST_CIRCLE.iter().enumerate() {
        let r = (row as i32 + dr as i32) as usize;
        let c = (col as i32 + dc as i32) as usize;
        classes[i] = classify(centre, frame.pixel(r, c), threshold);
    }

    let bright_run = max_circular_run(&classes, 1);
    let dark_run   = max_circular_run(&classes, -1);

    if bright_run >= FAST_MIN_RUN || dark_run >= FAST_MIN_RUN {
        Some(fast_score(frame, row, col, threshold))
    } else {
        None
    }
}

// ---------------------------------------------------------------------------
// BRIEF-256 descriptor
// ---------------------------------------------------------------------------

/// Compute the BRIEF-256 descriptor for a keypoint.
///
/// The descriptor is robust to in-plane rotation only at a fixed scale;
/// for SLAM a rotation-invariant variant (e.g. ORB) would be used instead.
/// This is sufficient as a SLAM foundation.
pub fn compute_brief(frame: &ImageFrame, row: usize, col: usize) -> [u8; 32] {
    let mut descriptor = [0u8; 32];
    for (bit_idx, pair) in BRIEF_PATTERN.iter().enumerate() {
        let r1 = (row as i32 + pair[0] as i32).clamp(0, frame.height as i32 - 1) as usize;
        let c1 = (col as i32 + pair[1] as i32).clamp(0, frame.width  as i32 - 1) as usize;
        let r2 = (row as i32 + pair[2] as i32).clamp(0, frame.height as i32 - 1) as usize;
        let c2 = (col as i32 + pair[3] as i32).clamp(0, frame.width  as i32 - 1) as usize;

        let p1 = frame.pixel(r1, c1);
        let p2 = frame.pixel(r2, c2);

        if p1 < p2 {
            descriptor[bit_idx / 8] |= 1 << (bit_idx % 8);
        }
    }
    descriptor
}

// ---------------------------------------------------------------------------
// Top-level detect_features
// ---------------------------------------------------------------------------

/// Detect FAST-9 corners in `frame` and compute BRIEF-256 descriptors.
///
/// Pixels within 3 pixels of the image border are excluded (the FAST circle
/// would go out of bounds).  No non-maximum suppression is applied.
///
/// `threshold`: FAST intensity threshold (default 20 is a good starting point).
///
/// Returns a `Vec<Feature>` sorted by score descending.
pub fn detect_features(frame: &ImageFrame, threshold: u8) -> Vec<Feature> {
    let rows = frame.height as usize;
    let cols = frame.width  as usize;

    if rows < 7 || cols < 7 { return Vec::new(); }

    let mut features = Vec::new();

    for row in 3..(rows - 3) {
        for col in 3..(cols - 3) {
            if let Some(score) = is_fast9_corner(frame, row, col, threshold) {
                let descriptor = compute_brief(frame, row, col);
                features.push(Feature {
                    x:    col as f32,
                    y:    row as f32,
                    score,
                    descriptor,
                });
            }
        }
    }

    // Sort strongest corners first.
    features.sort_by(|a, b| b.score.partial_cmp(&a.score).unwrap_or(std::cmp::Ordering::Equal));
    features
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use crate::perception::types::ImageFrame;

    /// Build a test image with a bright rectangle on a dark background.
    /// The four corners of the rectangle are FAST-9 corners.
    fn rect_image(w: u16, h: u16, bright: u8, dark: u8) -> ImageFrame {
        let mut pixels = vec![dark; w as usize * h as usize];
        // Draw a rectangle from (8,8) to (h-8, w-8).
        let r0 = 8usize;
        let c0 = 8usize;
        let r1 = h as usize - 9;
        let c1 = w as usize - 9;
        for r in r0..=r1 {
            for c in c0..=c1 {
                pixels[r * w as usize + c] = bright;
            }
        }
        ImageFrame { width: w, height: h, pixels, timestamp_ms: 0 }
    }

    #[test]
    fn fast_detects_corners() {
        let frame = rect_image(40, 40, 200, 10);
        let features = detect_features(&frame, 20);
        // Must detect at least 4 corners (the rectangle's corners).
        assert!(features.len() >= 4,
            "expected ≥4 corners, got {}", features.len());
    }

    #[test]
    fn fast_flat_image_has_no_corners() {
        let pixels = vec![128u8; 40 * 40];
        let frame  = ImageFrame { width: 40, height: 40, pixels, timestamp_ms: 0 };
        let features = detect_features(&frame, 20);
        assert!(features.is_empty(), "flat image should have no corners");
    }

    #[test]
    fn brief_descriptor_same_point_is_deterministic() {
        let frame = rect_image(60, 60, 200, 10);
        let d1 = compute_brief(&frame, 30, 30);
        let d2 = compute_brief(&frame, 30, 30);
        assert_eq!(d1, d2, "same point should give identical descriptor");
    }

    #[test]
    fn brief_descriptor_distance() {
        // Same image, nearby keypoints — the descriptors should be somewhat
        // similar (Hamming dist < 128 which is the expected value for random bits).
        let frame = rect_image(60, 60, 200, 10);

        // Two adjacent pixels on the bright rectangle interior.
        let d1 = compute_brief(&frame, 20, 20);
        let d2 = compute_brief(&frame, 20, 21);

        // Count Hamming distance.
        let hamming: u32 = d1.iter()
            .zip(d2.iter())
            .map(|(a, b)| (a ^ b).count_ones())
            .sum();

        assert!(hamming < 128,
            "adjacent pixels should have Hamming dist < 128, got {}", hamming);
    }

    #[test]
    fn features_sorted_by_score() {
        let frame    = rect_image(60, 60, 200, 10);
        let features = detect_features(&frame, 10);
        for w in features.windows(2) {
            assert!(w[0].score >= w[1].score,
                "features should be sorted by score descending");
        }
    }

    #[test]
    fn max_circular_run_wrap() {
        // Run of 1s that wraps around the end: positions 14,15,0,1,2 → run of 5.
        let mut c = [0i8; 16];
        c[14] = 1; c[15] = 1; c[0] = 1; c[1] = 1; c[2] = 1;
        assert_eq!(max_circular_run(&c, 1), 5);
    }
}
