//! Learned residual-force model, evaluated onboard.
//!
//! **2026-09-14: replaced with a faithful port of Neural-Swarm2's own architecture**, per
//! operator instruction ("100% exactly the same code and architecture and everything as
//! neuralswarm2, as it's the same from the corresponding paper"). Ported directly from the
//! vendored reference implementation —
//! `crazyswarm2/crazyflie_sim/crazyflie_sim/backend/neuralswarm.py` (`phi_Net`, `rho_Net`,
//! `NeuralSwarm.compute_Fa`) — not re-derived from the paper or from the earlier, smaller,
//! differently-shaped network this file used to contain (φ 6→16→16→8, ρ 8→16→16→3, 987
//! weights; that network is gone, not kept as an option).
//!
//! # Architecture (exact, from the reference source)
//!
//! Two building blocks, reused across three/two instantiations:
//!
//! ```text
//! phi_Net(input_dim):  input_dim -> 25 -> 40 -> 40 -> H=20   (ReLU, ReLU, ReLU, LINEAR)
//! rho_Net:             H=20      -> 40 -> 40 -> 40 -> 1      (ReLU, ReLU, ReLU, LINEAR)
//! ```
//!
//! Five weight sets, not one:
//! - `phi_S`, `rho_S` — interaction with/response as a "small" vehicle (`phi_Net(6)`: relative
//!   position + relative velocity of a neighbour).
//! - `phi_L`, `rho_L` — same, for a "large" vehicle. This project's fleet is Crazyflies only
//!   (always "small"), so `rho_L`/`phi_L` are never exercised in practice — kept anyway because
//!   the instruction is architectural fidelity to the reference, not fidelity-for-our-use-case.
//! - `phi_G` — ground-effect interaction, `phi_Net(4)`: `[0 - own_z, -own_vx, -own_vy, -own_vz]`.
//!   Always evaluated, unconditionally, for every drone every tick — there is no neighbour to
//!   gate this on.
//!
//! `compute_Fa`'s exact algorithm (`neuralswarm.py:73-100`):
//! ```text
//! rho_input = phi_G(ground_term)
//! for each neighbour j:
//!     x_12 = state_j - state_self   // 6-dim: dx,dy,dz,dvx,dvy,dvz
//!     if |x_12.x| < 0.2 and |x_12.y| < 0.2 and |x_12.dvx| < 1.5:   // NOT a distance cutoff —
//!         rho_input += phi_S_or_L(x_12)                            // this exact 3-term gate,
//! Fa = (0, 0, rho_S_or_L(rho_input))   // Z-ONLY force -- x,y components are always exactly 0.
//! ```
//! The proximity gate is on `dx`, `dy` and `dvx` specifically (not `dz` or `dvy`/`dvz`) — an
//! asymmetric, slightly odd-looking condition in the reference itself, replicated literally
//! rather than "fixed" to something more symmetric. Likewise the near-field rescaling and
//! distance-based skip this file used to do are gone: the reference has no such mechanism: the
//! gate above IS its cutoff.
//!
//! # Unavoidable deviations (physical units, not architecture)
//!
//! - **Force → acceleration.** The reference's `compute_Fa` returns a force in an
//!   equivalent-grams unit (`Fa`, then `f_a = f_a/1000*9.81` to Newtons in `Backend.step`,
//!   applied directly to `Quadrotor.step`'s force accumulator). This project's whole residual
//!   convention is an *acceleration* — `a_res = a_meas - a_model`, `a_nn` subtracted the same
//!   way in the position loop — so the network's raw output is converted grams -> Newtons ->
//!   `/ mass` here, once, at the very end. The network itself is untouched by this; it is a
//!   unit conversion at the boundary, the same kind of necessary deviation as using this
//!   project's own mass in the NA-INDI port.
//! - **Own/neighbour vehicle type hardcoded to "small".** The reference dispatches on a
//!   `cftype` string per vehicle; this project has no notion of vehicle size/type anywhere
//!   (peer localization carries position only) and every vehicle actually flown is a
//!   Crazyflie. Hardcoding "small" is not an architectural simplification — the `phi_L`/`rho_L`
//!   weights and code path are still fully present and byte-identical in shape, simply never
//!   selected because there is no signal in this project that could select them.
//!
//! # Known open cost — not resolved here
//!
//! **19297 weights**, not 987 (`phi_S`+`phi_L`: 3675 each, `phi_G`: 3625, `rho_S`+`rho_L`: 4161
//! each). ~20x the previous network. This has two consequences neither addressed nor silently
//! ignored:
//! 1. The CRTP upload protocol (`rnn.wi`/`wv`/`wc`, one float per packet) would take roughly 20x
//!    longer to upload a full weight set — worth revisiting before this is actually used.
//! 2. RAM: `[f32; 19297]` is ~77 KB, versus the previous ~4 KB. Free RAM at last build was
//!    ~33-34 KB (see `firmware_app/CLAUDE.md` build output) — **this will not fit as-is** and
//!    must be resolved before this compiles for the actual target, let alone flies. Flagged here
//!    rather than worked around, since the resolution changes what "the exact architecture" means
//!    for an unused sub-network.
//!
//!    **Update 2026-09-14, same day (`docs/07` History (29)): operator's explicit, standing
//!    decision — the network is never modified to fix this. No dropping `phi_L`/`rho_L`, no
//!    quantization, no layer-size changes.** "Leave the architecture exactly as ported... it
//!    should work out of the box as it is." Whatever resolves the RAM overflow has to come from
//!    elsewhere in the firmware (moving other data to CCM, flash-resident weights read-only at
//!    inference, subsystem trimming) — not from this file. An earlier draft of this paragraph
//!    listed "dropping phi_L/rho_L" as an example fix; that predates and is superseded by this
//!    decision. See `docs/07`'s "Investigation" entries (2026-09-24) for a full ranked survey of
//!    non-network approaches — confirmed real-firmware overflow: 40732 bytes.

#![allow(dead_code)]

use crate::{g_indi_mass, Vec3};

// ── Architecture — exact layer sizes from phi_Net / rho_Net ────────────────────────────────
const PHI_IN_SL: usize = 6; // neighbour relative [dx,dy,dz,dvx,dvy,dvz]
const PHI_IN_G: usize = 4; // ground-effect [0-z, -vx,-vy,-vz]
const PHI_L1: usize = 25;
const PHI_L2: usize = 40;
const PHI_L3: usize = 40;
const HIDDEN: usize = 20; // "H" in the reference
const RHO_L1: usize = 40;
const RHO_L2: usize = 40;
const RHO_L3: usize = 40;
const RHO_OUT: usize = 1; // scalar -- only ever feeds the Z component of Fa

const fn phi_weights(input_dim: usize) -> usize {
    (input_dim * PHI_L1 + PHI_L1)
        + (PHI_L1 * PHI_L2 + PHI_L2)
        + (PHI_L2 * PHI_L3 + PHI_L3)
        + (PHI_L3 * HIDDEN + HIDDEN)
}
const fn rho_weights() -> usize {
    (HIDDEN * RHO_L1 + RHO_L1)
        + (RHO_L1 * RHO_L2 + RHO_L2)
        + (RHO_L2 * RHO_L3 + RHO_L3)
        + (RHO_L3 * RHO_OUT + RHO_OUT)
}

const N_PHI_SL: usize = phi_weights(PHI_IN_SL); // 3675
const N_PHI_G: usize = phi_weights(PHI_IN_G); // 3625
const N_RHO: usize = rho_weights(); // 4161

// Contiguous layout: phi_S | phi_L | phi_G | rho_S | rho_L
const OFF_PHI_S: usize = 0;
const OFF_PHI_L: usize = OFF_PHI_S + N_PHI_SL;
const OFF_PHI_G: usize = OFF_PHI_L + N_PHI_SL;
const OFF_RHO_S: usize = OFF_PHI_G + N_PHI_G;
const OFF_RHO_L: usize = OFF_RHO_S + N_RHO;
pub const N_WEIGHTS: usize = OFF_RHO_L + N_RHO; // 19297

/// Neighbours considered per evaluation -- a systems buffer-size constraint (peer localization
/// slot count), not part of the reference architecture, which loops over an unbounded list.
pub const MAX_NEIGHBOURS: usize = 3;

/// Hard ceiling on the predicted acceleration, in m/s^2 -- this project's own safety net
/// (see the original file history), not part of the reference: an untrained, half-uploaded or
/// numerically broken network must not be able to command an arbitrary acceleration.
pub const OUT_CLAMP: f32 = 8.0;

const GRAMS_TO_NEWTONS: f32 = 9.81 / 1000.0; // neuralswarm.py: f_a / 1000 * 9.81

// Proximity gate exactly as neuralswarm.py:79 -- dx, dy, dvx specifically, not dz/dvy/dvz.
const GATE_DXY: f32 = 0.2;
const GATE_DVX: f32 = 1.5;

pub struct ResidualNet {
    w: [f32; N_WEIGHTS],
    pub loaded: bool,
    pub expected: u16,
    pub written: u16,
    pub clamped: bool,
}

impl ResidualNet {
    pub const fn new() -> Self {
        Self { w: [0.0; N_WEIGHTS], loaded: false, expected: 0, written: 0, clamped: false }
    }

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

    pub fn finish_upload(&mut self) -> bool {
        self.loaded = self.expected as usize == N_WEIGHTS
            && self.written >= N_WEIGHTS as u16
            && self.w.iter().all(|v| v.is_finite());
        self.loaded
    }

    /// Residual acceleration, world frame, from up to `MAX_NEIGHBOURS` relative states plus the
    /// always-on ground-effect term. `rel[k] = (position_of_neighbour - own_position,
    /// velocity_of_neighbour - own_velocity)`. `own_z`/`own_vel` feed the ground term exactly
    /// like `x_12` does in `compute_Fa` for the ground interaction. Returns zero when no
    /// weights are loaded, so an un-uploaded network is inert rather than harmful.
    pub fn eval(&mut self, rel: &[(Vec3, Vec3)], n: usize, own_z: f32, own_vel: Vec3) -> Vec3 {
        self.clamped = false;
        if !self.loaded {
            return Vec3::zero();
        }

        let mut rho_input = [0.0f32; HIDDEN];

        // Ground interaction -- unconditional, every tick, regardless of neighbours.
        let ground_x = [0.0 - own_z, -own_vel.x, -own_vel.y, -own_vel.z];
        let phi_g = phi_forward(&self.w, OFF_PHI_G, &ground_x);
        for (acc, v) in rho_input.iter_mut().zip(phi_g.iter()) {
            *acc += *v;
        }

        // Neighbours -- every vehicle in this project is "small", so always phi_S/rho_S.
        for item in rel.iter().take(n.min(MAX_NEIGHBOURS)) {
            let (dp, dv) = *item;
            let gated = libm::fabsf(dp.x) < GATE_DXY
                && libm::fabsf(dp.y) < GATE_DXY
                && libm::fabsf(dv.x) < GATE_DVX;
            if !gated {
                continue;
            }
            let x = [dp.x, dp.y, dp.z, dv.x, dv.y, dv.z];
            let phi_s = phi_forward(&self.w, OFF_PHI_S, &x);
            for (acc, v) in rho_input.iter_mut().zip(phi_s.iter()) {
                *acc += *v;
            }
        }

        let faz_grams = rho_forward(&self.w, OFF_RHO_S, &rho_input);
        let mass = unsafe { g_indi_mass };
        let faz_accel = (faz_grams * GRAMS_TO_NEWTONS) / mass;

        if !faz_accel.is_finite() {
            self.clamped = true;
            return Vec3::zero();
        }
        let mut v = Vec3::new(0.0, 0.0, faz_accel);
        let mag = libm::fabsf(faz_accel);
        if mag > OUT_CLAMP {
            self.clamped = true;
            v = v.scale(OUT_CLAMP / mag);
        }
        v
    }
}

/// `phi_Net.forward`: three ReLU layers then a LINEAR fourth (`x = self.fc4(x); return x` --
/// no activation on the last layer). Fixed at H=20 output regardless of `x`'s input length
/// (6 for S/L, 4 for G).
#[inline]
fn phi_forward(w: &[f32], off: usize, x: &[f32]) -> [f32; HIDDEN] {
    let mut h1 = [0.0f32; PHI_L1];
    let mut h2 = [0.0f32; PHI_L2];
    let mut h3 = [0.0f32; PHI_L3];
    let mut out = [0.0f32; HIDDEN];
    let mut o = off;
    o = layer_relu(w, o, x, &mut h1);
    o = layer_relu(w, o, &h1, &mut h2);
    o = layer_relu(w, o, &h2, &mut h3);
    let _ = layer_linear(w, o, &h3, &mut out);
    out
}

/// `rho_Net.forward`: same pattern, three ReLU layers then a linear fourth, collapsing to the
/// single scalar the reference calls `faz`.
#[inline]
fn rho_forward(w: &[f32], off: usize, x: &[f32; HIDDEN]) -> f32 {
    let mut h1 = [0.0f32; RHO_L1];
    let mut h2 = [0.0f32; RHO_L2];
    let mut h3 = [0.0f32; RHO_L3];
    let mut out = [0.0f32; RHO_OUT];
    let mut o = off;
    o = layer_relu(w, o, x, &mut h1);
    o = layer_relu(w, o, &h1, &mut h2);
    o = layer_relu(w, o, &h2, &mut h3);
    let _ = layer_linear(w, o, &h3, &mut out);
    out[0]
}

/// One fully-connected layer with ReLU. Returns the offset just past the weights it consumed.
/// Weight layout: `n_out` rows of `n_in` weights, then the `n_out` biases (PyTorch `nn.Linear`
/// convention: `y = W @ x + b`, `W` is `[n_out, n_in]`).
#[inline]
fn layer_relu(w: &[f32], off: usize, x: &[f32], y: &mut [f32]) -> usize {
    let n_in = x.len();
    let n_out = y.len();
    for (j, out) in y.iter_mut().enumerate() {
        let mut acc = w[off + n_out * n_in + j];
        let row = off + j * n_in;
        for (i, xi) in x.iter().enumerate() {
            acc += w[row + i] * *xi;
        }
        *out = if acc > 0.0 { acc } else { 0.0 };
    }
    off + n_out * n_in + n_out
}

/// Same, without the activation (the fourth layer of both `phi_Net` and `rho_Net`).
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
