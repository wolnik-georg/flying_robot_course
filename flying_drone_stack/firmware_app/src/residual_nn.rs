//! Learned residual-force model, evaluated onboard.
//!
//! Predicts the interaction acceleration `f_res / m` a vehicle is about to experience from the
//! relative states of its neighbours, so a controller can cancel it before it shows up in the
//! tracking error. That is what separates a *learned* compensation method from INDI, which can
//! only react once the disturbance has already been measured.
//!
//! # Why deep sets rather than a plain MLP
//!
//! A fixed-input MLP has to be retrained for every neighbour count, and its answer depends on
//! the order the neighbours happen to be listed in — neither of which is true of the physics.
//! The deep-sets form used by Neural-Swarm2 avoids both:
//!
//! ```text
//!     a_res  =  rho( sum_j phi(relative_state_j) )
//! ```
//!
//! `phi` is applied to each neighbour separately and the results are summed, so the output is
//! permutation-invariant by construction and the same weights serve two drones or three. That
//! matters here directly: the thesis flies both 2- and 3-robot formations, and B1/B2 exist
//! precisely to test whether interactions superpose.
//!
//! # Sizing
//!
//! phi: 6 -> 16 -> 16 -> 8, rho: 8 -> 16 -> 16 -> 3. Around a thousand weights and roughly
//! 1400 multiply-accumulates for two neighbours, which is comfortable inside a 500 Hz control
//! loop on an M4F and small enough that a full weight upload over CRTP parameters takes
//! seconds rather than minutes. The dimensions are compile-time constants; grow them once the
//! measured accuracy justifies the cost, not before.
//!
//! # Normalisation
//!
//! Deliberately not implemented here. Input scaling and mean-subtraction are folded into the
//! first layer's weights and biases at export time, which is exactly equivalent and keeps the
//! firmware free of a second set of constants that could drift out of step with the model.

#![allow(dead_code)]

use crate::Vec3;

// ── Architecture ────────────────────────────────────────────────────────────
pub const PHI_IN: usize = 6; // neighbour relative position (3) and velocity (3)
pub const PHI_H1: usize = 16;
pub const PHI_H2: usize = 16;
pub const LATENT: usize = 8;
pub const RHO_H1: usize = 16;
pub const RHO_H2: usize = 16;
pub const RHO_OUT: usize = 3; // residual acceleration, world frame [m/s^2]

/// Weights plus biases, laid out layer by layer, row-major within a layer.
pub const N_WEIGHTS: usize = (PHI_IN * PHI_H1 + PHI_H1)
    + (PHI_H1 * PHI_H2 + PHI_H2)
    + (PHI_H2 * LATENT + LATENT)
    + (LATENT * RHO_H1 + RHO_H1)
    + (RHO_H1 * RHO_H2 + RHO_H2)
    + (RHO_H2 * RHO_OUT + RHO_OUT);

/// Neighbours considered per evaluation. Three is enough for the planned formations; a
/// vehicle in a 4-robot team would use the three nearest, which is where the interaction is.
pub const MAX_NEIGHBOURS: usize = 3;

/// Hard ceiling on the predicted acceleration, in m/s^2.
///
/// A network that is untrained, half-uploaded or numerically broken must not be able to
/// command an arbitrary acceleration. Downwash between Crazyflies is on the order of
/// 0.1-3 m/s^2; anything past this is not a prediction, it is a fault, and the output is
/// clamped rather than trusted. Being clamped is visible in the logs, which is the point.
pub const OUT_CLAMP: f32 = 8.0;

/// Below this distance the model is extrapolating past anything it can have been trained on,
/// and a learned function is at its least trustworthy exactly where the physics is strongest.
/// Neighbours closer than this are evaluated at this distance instead of being trusted.
const MIN_DIST: f32 = 0.04;

/// Beyond this a neighbour contributes nothing measurable, so it is skipped — this also keeps
/// the cost proportional to the neighbours that matter rather than the ones in the room.
const MAX_DIST: f32 = 2.0;

// ── State ───────────────────────────────────────────────────────────────────

pub struct ResidualNet {
    w: [f32; N_WEIGHTS],
    /// Weights present and self-consistent. Inference returns zero until this is true.
    pub loaded: bool,
    /// How many weights the host said it would send.
    pub expected: u16,
    /// How many distinct indices have actually been written.
    pub written: u16,
    /// Set when the last evaluation hit OUT_CLAMP. Surfaced as a log variable, because a
    /// silently clamped network looks exactly like a well-behaved one from the outside.
    pub clamped: bool,
}

impl ResidualNet {
    pub const fn new() -> Self {
        Self { w: [0.0; N_WEIGHTS], loaded: false, expected: 0, written: 0, clamped: false }
    }

    /// Write one weight. Returns false for an out-of-range index rather than corrupting memory.
    pub fn set_weight(&mut self, idx: usize, value: f32) -> bool {
        if idx >= N_WEIGHTS || !value.is_finite() {
            return false;
        }
        self.w[idx] = value;
        self.written = self.written.saturating_add(1);
        true
    }

    pub fn begin_upload(&mut self, expected: u16) {
        self.loaded = false;
        self.written = 0;
        self.expected = expected;
        self.w = [0.0; N_WEIGHTS];
    }

    /// Accept the uploaded set only if the count matches and every weight is finite.
    ///
    /// A partially-arrived network is the dangerous case: it produces plausible-looking
    /// numbers rather than an obvious failure, and would be indistinguishable from a badly
    /// trained model. Refusing here means a dropped parameter packet shows up as
    /// "compensation off", which is diagnosable.
    pub fn finish_upload(&mut self) -> bool {
        self.loaded = self.expected as usize == N_WEIGHTS
            && self.written >= N_WEIGHTS as u16
            && self.w.iter().all(|v| v.is_finite());
        self.loaded
    }

    /// Residual acceleration from up to `MAX_NEIGHBOURS` relative states.
    ///
    /// `rel[k] = (position_of_neighbour - own_position, velocity_of_neighbour - own_velocity)`
    /// in the world frame. Returns zero when no weights are loaded, so an un-uploaded network
    /// is inert rather than harmful.
    pub fn eval(&mut self, rel: &[(Vec3, Vec3)], n: usize) -> Vec3 {
        self.clamped = false;
        if !self.loaded || n == 0 {
            return Vec3::zero();
        }

        let mut latent = [0.0f32; LATENT];
        let mut any = false;

        for item in rel.iter().take(n.min(MAX_NEIGHBOURS)) {
            let (dp, dv) = *item;
            let d2 = dp.x * dp.x + dp.y * dp.y + dp.z * dp.z;
            if d2 > MAX_DIST * MAX_DIST {
                continue; // too far to matter
            }
            // Clamp the radial distance without changing the direction: the model should not
            // be asked to extrapolate below the closest separation it was trained on.
            let d = libm::sqrtf(d2);
            let scale = if d < MIN_DIST && d > 1e-6 { MIN_DIST / d } else { 1.0 };

            let x = [dp.x * scale, dp.y * scale, dp.z * scale, dv.x, dv.y, dv.z];
            let mut h1 = [0.0f32; PHI_H1];
            let mut h2 = [0.0f32; PHI_H2];
            let mut out = [0.0f32; LATENT];

            let mut o = 0;
            o = layer_relu(&self.w, o, &x, &mut h1);
            o = layer_relu(&self.w, o, &h1, &mut h2);
            let _ = layer_relu(&self.w, o, &h2, &mut out);

            for (acc, v) in latent.iter_mut().zip(out.iter()) {
                *acc += *v;
            }
            any = true;
        }

        if !any {
            return Vec3::zero();
        }

        // rho starts after the whole of phi.
        let rho0 = (PHI_IN * PHI_H1 + PHI_H1) + (PHI_H1 * PHI_H2 + PHI_H2)
            + (PHI_H2 * LATENT + LATENT);
        let mut g1 = [0.0f32; RHO_H1];
        let mut g2 = [0.0f32; RHO_H2];
        let mut y = [0.0f32; RHO_OUT];

        let mut o = rho0;
        o = layer_relu(&self.w, o, &latent, &mut g1);
        o = layer_relu(&self.w, o, &g1, &mut g2);
        let _ = layer_linear(&self.w, o, &g2, &mut y); // linear output: a force can be negative

        let mut v = Vec3::new(y[0], y[1], y[2]);
        let mag = libm::sqrtf(v.x * v.x + v.y * v.y + v.z * v.z);
        if !mag.is_finite() {
            self.clamped = true;
            return Vec3::zero();
        }
        if mag > OUT_CLAMP {
            self.clamped = true;
            v = v.scale(OUT_CLAMP / mag);
        }
        v
    }
}

/// One fully-connected layer with ReLU. Returns the offset just past the weights it consumed.
#[inline]
fn layer_relu(w: &[f32], off: usize, x: &[f32], y: &mut [f32]) -> usize {
    let n_in = x.len();
    let n_out = y.len();
    for (j, out) in y.iter_mut().enumerate() {
        let mut acc = w[off + n_out * n_in + j]; // bias block follows the weight block
        let row = off + j * n_in;
        for (i, xi) in x.iter().enumerate() {
            acc += w[row + i] * *xi;
        }
        *out = if acc > 0.0 { acc } else { 0.0 };
    }
    off + n_out * n_in + n_out
}

/// Same, without the activation.
#[inline]
fn layer_linear(w: &[f32], off: usize, x: &[f32], y: &mut [f32]) -> usize {
    let n_in = x.len();
    let n_out = y.len();
    for (j, out) in y.iter_mut().enumerate() {
        let mut acc = w[off + n_out * n_in + j];
        let row = off + j * n_in;
        for (i, xi) in x.iter().enumerate() {
            acc += w[row + i] * *xi;
        }
        *out = acc;
    }
    off + n_out * n_in + n_out
}
