//! Pose-graph loop closure for SLAM.
//!
//! ## Overview
//!
//! When the drone revisits a previously-seen location, [`KeyframeStore::detect_loop`]
//! returns a [`LoopConstraint`] describing the relative displacement between the two
//! keyframes.  The constraint is added to the [`PoseGraph`], which maintains a sparse
//! graph of XY nodes (one per accepted keyframe) connected by:
//!
//! - **Sequential edges** — added automatically in [`PoseGraph::add_node`], weight
//!   `W_SEQ = 100` (≈10 cm σ per 0.3 m step).
//! - **Loop edges** — added by [`PoseGraph::add_loop`], weight
//!   `LOOP_WEIGHT_FACTOR × inlier_count` (typically 80–200 for 8–20 inliers).
//!
//! ## Gauss-Seidel optimisation
//!
//! [`PoseGraph::optimize`] runs `PG_ITERATIONS` sweeps of Gauss-Seidel on the
//! weighted least-squares objective:
//!
//! ```text
//! min Σ_{(i,j,Δ,w)} w · ‖x_j - x_i - Δ‖²
//! ```
//!
//! Slot 0 is the anchor (fixed).  Each non-anchor slot update:
//!
//! ```text
//! x_s ← (Σ_{j:to_s} w·(x_j - Δ) + Σ_{i:from_s} w·(x_i + Δ)) / Σ w
//! ```
//!
//! Convergence is guaranteed for any connected graph with a fixed anchor.
//! Timing: 100 × N × (N+loops) — for 200 nodes ≈ 4M multiply-adds ≈ 2 ms on a laptop CPU.
//!
//! ## Integration with MEKF
//!
//! After `optimize()` returns `Some(corrected_xy)`, the main loop calls
//! `mekf_update_vo` with a tight noise `R_LOOP = 0.01 m²` (10 cm σ) and
//! reseeds the `VoTrajectory` to the corrected MEKF position.

// ---------------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------------

/// Loop edge information-weight multiplier.
/// Weight = LOOP_WEIGHT_FACTOR × inlier_count.
const LOOP_WEIGHT_FACTOR: f32 = 10.0;

/// Number of Gauss-Seidel sweeps per `optimize()` call.
const PG_ITERATIONS: usize = 100;

/// Sequential edge information weight (fixed).
/// 100 ≈ σ_seq = 0.1 m per step (0.3 m keyframe spacing).
const W_SEQ: f32 = 100.0;

// ---------------------------------------------------------------------------
// Public types
// ---------------------------------------------------------------------------

/// A detected loop closure: the estimated world-frame displacement from an
/// older keyframe (`from_idx`) to the newest keyframe (`to_idx`).
#[derive(Debug, Clone)]
pub struct LoopConstraint {
    /// `kf_index` of the older keyframe (from keyframe store monotonic counter).
    pub from_idx: usize,
    /// `kf_index` of the newer keyframe.
    pub to_idx: usize,
    /// World-frame displacement from→to [m].
    pub translation_world: [f32; 2],
    /// Number of essential-matrix inliers used to compute the translation.
    pub inlier_count: usize,
}

// ---------------------------------------------------------------------------
// PoseGraph
// ---------------------------------------------------------------------------

/// Sparse XY pose graph with Gauss-Seidel optimisation.
///
/// Node slots are indexed by insertion order (0-based).  Slot 0 is the
/// anchor and is never updated by the optimiser.
pub struct PoseGraph {
    /// Ordered list of `kf_index` values — maps slot → kf_index.
    kf_indices: Vec<usize>,
    /// XY position estimate for each slot.
    positions: Vec<[f32; 2]>,
    /// Number of registered nodes.
    n_nodes: usize,
    /// Sequential edges: (from_slot, to_slot, delta_xy, weight).
    seq_edges: Vec<(usize, usize, [f32; 2], f32)>,
    /// Loop closure edges: (from_slot, to_slot, delta_xy, weight).
    loop_edges: Vec<(usize, usize, [f32; 2], f32)>,
    /// Total loop closures added (never decremented).
    pub lc_count: usize,
}

impl PoseGraph {
    /// Create an empty pose graph.
    pub fn new() -> Self {
        Self {
            kf_indices: Vec::new(),
            positions:  Vec::new(),
            n_nodes:    0,
            seq_edges:  Vec::new(),
            loop_edges: Vec::new(),
            lc_count:   0,
        }
    }

    /// Linear search: slot index for a given `kf_index`, or `None`.
    fn slot_of(&self, kf_idx: usize) -> Option<usize> {
        self.kf_indices.iter().position(|&k| k == kf_idx)
    }

    /// Register a new keyframe node at the given XY position.
    ///
    /// Automatically adds a sequential edge from the previous node to this one
    /// (delta = position difference at registration time, w = `W_SEQ`).
    pub fn add_node(&mut self, kf_idx: usize, xy: [f32; 2]) {
        let slot = self.n_nodes;
        self.kf_indices.push(kf_idx);
        self.positions.push(xy);

        // Sequential edge from the previous node.
        if slot > 0 {
            let prev = self.positions[slot - 1];
            let delta = [xy[0] - prev[0], xy[1] - prev[1]];
            self.seq_edges.push((slot - 1, slot, delta, W_SEQ));
        }

        self.n_nodes += 1;
    }

    /// Add a loop closure edge.
    ///
    /// `lc.from_idx` and `lc.to_idx` must already be registered nodes;
    /// silently ignored if either is not found.
    /// Edge weight = `LOOP_WEIGHT_FACTOR × lc.inlier_count`.
    pub fn add_loop(&mut self, lc: &LoopConstraint) {
        let from_slot = match self.slot_of(lc.from_idx) {
            Some(s) => s,
            None    => return,
        };
        let to_slot = match self.slot_of(lc.to_idx) {
            Some(s) => s,
            None    => return,
        };
        let w = LOOP_WEIGHT_FACTOR * lc.inlier_count as f32;
        self.loop_edges.push((from_slot, to_slot, lc.translation_world, w));
        self.lc_count += 1;
    }

    /// Run Gauss-Seidel for `PG_ITERATIONS` sweeps.
    ///
    /// Returns the corrected XY of the latest node if the change exceeds 1 mm,
    /// or `None` if the graph has fewer than 2 nodes or the result didn't move.
    /// Slot 0 is the anchor and is never updated.
    pub fn optimize(&mut self) -> Option<[f32; 2]> {
        if self.n_nodes < 2 {
            return None;
        }

        let initial_latest = self.positions[self.n_nodes - 1];

        for _ in 0..PG_ITERATIONS {
            // Update every non-anchor slot.
            for s in 1..self.n_nodes {
                let mut num   = [0.0f32; 2];
                let mut denom = 0.0f32;

                for &(from_slot, to_slot, delta, w) in
                    self.seq_edges.iter().chain(self.loop_edges.iter())
                {
                    if from_slot == s {
                        // Edge s→to: x_s ≈ x_to - delta
                        let pos_to = self.positions[to_slot];
                        num[0] += w * (pos_to[0] - delta[0]);
                        num[1] += w * (pos_to[1] - delta[1]);
                        denom  += w;
                    }
                    if to_slot == s {
                        // Edge from→s: x_s ≈ x_from + delta
                        let pos_from = self.positions[from_slot];
                        num[0] += w * (pos_from[0] + delta[0]);
                        num[1] += w * (pos_from[1] + delta[1]);
                        denom  += w;
                    }
                }

                if denom > 1e-6 {
                    self.positions[s] = [num[0] / denom, num[1] / denom];
                }
            }
        }

        let latest = self.positions[self.n_nodes - 1];
        let dx = latest[0] - initial_latest[0];
        let dy = latest[1] - initial_latest[1];
        if dx * dx + dy * dy > 1e-6 {
            Some(latest)
        } else {
            None
        }
    }

    /// Return the XY position of the most recently registered node,
    /// or `None` if the graph is empty.
    pub fn latest_pos(&self) -> Option<[f32; 2]> {
        if self.n_nodes == 0 {
            None
        } else {
            Some(self.positions[self.n_nodes - 1])
        }
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn pose_graph_empty_returns_none() {
        let mut pg = PoseGraph::new();
        assert!(pg.optimize().is_none());
    }

    #[test]
    fn pose_graph_single_node_returns_none() {
        let mut pg = PoseGraph::new();
        pg.add_node(0, [1.0, 2.0]);
        assert!(pg.optimize().is_none());
    }

    #[test]
    fn anchor_slot_never_moves() {
        let mut pg = PoseGraph::new();
        pg.add_node(0, [1.0, 2.0]);
        pg.add_node(1, [2.0, 2.0]);
        // Loop that pulls node 1 strongly — anchor must stay fixed.
        pg.add_loop(&LoopConstraint {
            from_idx: 0, to_idx: 1,
            translation_world: [0.3, 0.0],
            inlier_count: 50,
        });
        pg.optimize();
        assert_eq!(pg.positions[0], [1.0, 2.0],
            "anchor slot must not move");
    }

    #[test]
    fn sequential_only_no_change() {
        // Perfectly consistent sequential chain — no loop → positions unchanged → None.
        let mut pg = PoseGraph::new();
        pg.add_node(0, [0.0, 0.0]);
        pg.add_node(1, [1.0, 0.0]);
        pg.add_node(2, [2.0, 0.0]);
        // Sequential edges added automatically; no loop added.
        let result = pg.optimize();
        assert!(result.is_none(),
            "consistent sequential chain should produce no change");
    }

    #[test]
    fn loop_corrects_drift() {
        // 3-node graph: sequential edges say x2=2.0, loop says x2≈1.5 → x2 pulled between them.
        let mut pg = PoseGraph::new();
        pg.add_node(0, [0.0, 0.0]);
        pg.add_node(1, [1.0, 0.0]);
        pg.add_node(2, [2.0, 0.0]);
        // Loop closure from node 0 to node 2 with translation [1.5, 0].
        pg.add_loop(&LoopConstraint {
            from_idx: 0,
            to_idx: 2,
            translation_world: [1.5, 0.0],
            inlier_count: 20, // w = 200
        });
        let result = pg.optimize();
        assert!(result.is_some(), "loop should cause a change");
        let pos = result.unwrap();
        assert!(pos[0] > 1.5 && pos[0] < 2.0,
            "loop should pull x2 between 1.5 and 2.0, got {}", pos[0]);
    }

    #[test]
    fn loop_weight_larger_gives_stronger_pull() {
        // Higher inlier_count → larger loop weight → node pulled further towards loop target.
        let make_graph = |inliers: usize| {
            let mut pg = PoseGraph::new();
            pg.add_node(0, [0.0, 0.0]);
            pg.add_node(1, [1.0, 0.0]);
            pg.add_node(2, [2.0, 0.0]);
            pg.add_loop(&LoopConstraint {
                from_idx: 0, to_idx: 2,
                translation_world: [1.5, 0.0],
                inlier_count: inliers,
            });
            pg.optimize().map(|p| p[0]).unwrap_or(2.0)
        };
        let x_low  = make_graph(10); // w=100
        let x_high = make_graph(20); // w=200
        assert!(x_high < x_low,
            "higher weight should pull harder: low={:.4} high={:.4}", x_low, x_high);
    }

    #[test]
    fn lc_count_increments_on_add_loop() {
        let mut pg = PoseGraph::new();
        pg.add_node(0, [0.0, 0.0]);
        pg.add_node(1, [1.0, 0.0]);
        pg.add_node(2, [2.0, 0.0]);
        assert_eq!(pg.lc_count, 0);
        pg.add_loop(&LoopConstraint {
            from_idx: 0, to_idx: 2,
            translation_world: [1.5, 0.0],
            inlier_count: 10,
        });
        assert_eq!(pg.lc_count, 1);
        pg.add_loop(&LoopConstraint {
            from_idx: 0, to_idx: 2,
            translation_world: [1.5, 0.0],
            inlier_count: 10,
        });
        assert_eq!(pg.lc_count, 2);
    }

    #[test]
    fn latest_pos_returns_last_node() {
        let mut pg = PoseGraph::new();
        assert!(pg.latest_pos().is_none());
        pg.add_node(0, [1.0, 2.0]);
        assert_eq!(pg.latest_pos(), Some([1.0, 2.0]));
        pg.add_node(1, [3.0, 4.0]);
        assert_eq!(pg.latest_pos(), Some([3.0, 4.0]));
        pg.add_node(2, [5.0, 6.0]);
        assert_eq!(pg.latest_pos(), Some([5.0, 6.0]));
    }

    #[test]
    fn pose_graph_grows_beyond_old_limit() {
        // Old PG_MAX_NODES was 60; ensure we can add 80 nodes without panic.
        let mut pg = PoseGraph::new();
        for i in 0..80 {
            pg.add_node(i, [i as f32 * 0.3, 0.0]);
        }
        assert_eq!(pg.n_nodes, 80, "should store 80 nodes");
        // optimize() must not panic on a large graph.
        let _ = pg.optimize();
    }

    #[test]
    fn pose_graph_large_sequential_chain() {
        // 100 nodes in a perfectly consistent line — sequential edges only,
        // no loop → optimizer finds no drift to correct → returns None.
        let mut pg = PoseGraph::new();
        for i in 0..100 {
            pg.add_node(i, [i as f32 * 0.3, 0.0]);
        }
        assert_eq!(pg.n_nodes, 100);
        let result = pg.optimize();
        assert!(result.is_none(),
            "consistent sequential chain with no loop should return None");
    }
}
