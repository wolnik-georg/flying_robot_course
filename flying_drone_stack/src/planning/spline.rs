//! Polynomial spline trajectory planner.
//!
//! Uses 8th-order polynomials (9 coefficients per axis per segment) and
//! minimises the integral of snap² to produce smooth, dynamically feasible
//! reference trajectories.  Continuity up to the 4th derivative (snap) is
//! enforced at every interior waypoint.  Zero boundary conditions are imposed
//! on derivatives 1–4 (velocity → snap) at both the start and end.
//!
//! The QP is solved with the `simple_qp` / Clarabel backend.

use simple_qp::constraint;
use simple_qp::problem_variables::ProblemVariables;
use simple_qp::solver::clarabel_solver::ClarabelSolver;
use simple_qp::solver::Solver;
use simple_qp::expressions::variable::Variable;
use simple_qp::expressions::affine_expression::AffineExpression;
use simple_qp::expressions::quadratic_expression::QuadraticExpression;

use crate::math::Vec3;
use super::flatness::FlatOutput;

// ---------------------------------------------------------------------------
// Polynomial helpers (8th-order, 9 coefficients, unit segment [0,1])
// ---------------------------------------------------------------------------

/// Evaluate p(t) = Σ aᵢ tⁱ
fn poly(a: &[Variable], t: f32) -> AffineExpression {
    let mut e: AffineExpression = AffineExpression::from(0.0_f32);
    let mut ti = 1.0_f32;
    for ai in a.iter() {
        e = e + *ai * ti;
        ti *= t;
    }
    e
}

/// Evaluate d/dt p(t)
fn poly_d1(a: &[Variable], t: f32) -> AffineExpression {
    // p'(t) = a1 + 2a2 t + 3a3 t² + ... + 8a8 t⁷
    let coeffs: &[f32] = &[1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0];
    let mut e: AffineExpression = AffineExpression::from(0.0_f32);
    let mut ti = 1.0_f32;
    for (i, ai) in a[1..].iter().enumerate() {
        e = e + *ai * (coeffs[i] * ti);
        ti *= t;
    }
    e
}

/// Evaluate d²/dt² p(t)
fn poly_d2(a: &[Variable], t: f32) -> AffineExpression {
    // p''(t) = 2a2 + 6a3 t + 12a4 t² + 20a5 t³ + 30a6 t⁴ + 42a7 t⁵ + 56a8 t⁶
    let coeffs: &[f32] = &[2.0, 6.0, 12.0, 20.0, 30.0, 42.0, 56.0];
    let mut e: AffineExpression = AffineExpression::from(0.0_f32);
    let mut ti = 1.0_f32;
    for (i, ai) in a[2..].iter().enumerate() {
        e = e + *ai * (coeffs[i] * ti);
        ti *= t;
    }
    e
}

/// Evaluate d³/dt³ p(t)
fn poly_d3(a: &[Variable], t: f32) -> AffineExpression {
    // p'''(t) = 6a3 + 24a4 t + 60a5 t² + 120a6 t³ + 210a7 t⁴ + 336a8 t⁵
    let coeffs: &[f32] = &[6.0, 24.0, 60.0, 120.0, 210.0, 336.0];
    let mut e: AffineExpression = AffineExpression::from(0.0_f32);
    let mut ti = 1.0_f32;
    for (i, ai) in a[3..].iter().enumerate() {
        e = e + *ai * (coeffs[i] * ti);
        ti *= t;
    }
    e
}

/// Evaluate d⁴/dt⁴ p(t)  (snap)
fn poly_d4(a: &[Variable], t: f32) -> AffineExpression {
    // p''''(t) = 24a4 + 120a5 t + 360a6 t² + 840a7 t³ + 1680a8 t⁴
    let coeffs: &[f32] = &[24.0, 120.0, 360.0, 840.0, 1680.0];
    let mut e: AffineExpression = AffineExpression::from(0.0_f32);
    let mut ti = 1.0_f32;
    for (i, ai) in a[4..].iter().enumerate() {
        e = e + *ai * (coeffs[i] * ti);
        ti *= t;
    }
    e
}

/// ∫₀¹ (p''''(t))² dt as a quadratic in coefficients.
///
/// For p(t) = Σ aᵢ tⁱ, the snap is p''''(t) = Σ cᵢ aᵢ tⁱ⁻⁴ (i=4..8).
/// Writing s(t) = Σⱼ sⱼ tʲ where sⱼ = (j+4)! / j! · aⱼ₊₄  (j=0..4),
/// ∫₀¹ s(t)² dt = Σᵢⱼ sᵢ sⱼ / (i+j+1).
///
/// Returns a `QuadraticExpression` representing the snap cost.
fn snap_cost(a: &[Variable]) -> QuadraticExpression {
    // Derivative factors for coefficients a[4]..a[8]
    // (d⁴/dt⁴ of aᵢ tⁱ = i!/(i-4)! · aᵢ tⁱ⁻⁴)
    let factors: [f32; 5] = [24.0, 120.0, 360.0, 840.0, 1680.0];
    // Gram matrix G[i][j] = ∫₀¹ tⁱ tʲ dt = 1/(i+j+1)
    // cost = Σᵢ Σⱼ factors[i]*factors[j] * a[i+4]*a[j+4] / (i+j+1)
    let mut cost: QuadraticExpression = QuadraticExpression::from(0.0_f32);
    for i in 0..5usize {
        for j in 0..5usize {
            let g = 1.0 / (i + j + 1) as f32;
            let w = factors[i] * factors[j] * g;
            cost = cost + a[i + 4] * a[j + 4] * w;
        }
    }
    cost
}

// ---------------------------------------------------------------------------
// Scalable helpers (polynomial on [0, T] segment via substitution t → τ/T)
// ---------------------------------------------------------------------------

/// Evaluate p-th derivative of polynomial in physical time τ ∈ [0, T]
/// by using scaled coefficients: dⁿp/dτⁿ = (1/Tⁿ) * dⁿp/dtⁿ at t=τ/T
///
/// For the QP we normalise each segment to [0,1], then scale derivatives
/// by T⁻ⁿ when enforcing physical continuity or evaluating trajectory.

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

/// One spline segment per axis (x, y, z, yaw), 9 coefficients each.
#[derive(Debug, Clone)]
pub struct SplineSegment {
    pub cx: [f32; 9],
    pub cy: [f32; 9],
    pub cz: [f32; 9],
    pub cyaw: [f32; 9],
    /// Duration of this segment [s]
    pub duration: f32,
}

/// Full spline trajectory over N-1 segments between N waypoints.
#[derive(Debug, Clone)]
pub struct SplineTrajectory {
    pub segments: Vec<SplineSegment>,
    pub total_time: f32,
}

/// A waypoint specifying position and yaw (other derivatives are free).
#[derive(Debug, Clone, Copy)]
pub struct Waypoint {
    pub pos: Vec3,
    pub yaw: f32,
}

impl SplineTrajectory {
    /// Plan a minimum-snap spline through `waypoints` with per-segment durations.
    ///
    /// `durations[i]` is the time to travel from waypoint `i` to waypoint `i+1`.
    ///
    /// If `periodic` is true, derivatives 1–4 at the end of the last segment are
    /// constrained to match those at the start of the first segment (smooth looping).
    /// The first and last waypoint positions must be equal when `periodic` is true.
    pub fn plan(waypoints: &[Waypoint], durations: &[f32], periodic: bool) -> Result<Self, String> {
        let n_wp = waypoints.len();
        let n_seg = n_wp - 1;
        assert_eq!(durations.len(), n_seg, "need n_seg durations");

        let no_pins = vec![None::<f32>; n_wp];
        let cx = solve_axis(n_seg, |i| waypoints[i].pos.x, &no_pins, durations, periodic)?;
        let cy = solve_axis(n_seg, |i| waypoints[i].pos.y, &no_pins, durations, periodic)?;
        let cz = solve_axis(n_seg, |i| waypoints[i].pos.z, &no_pins, durations, periodic)?;
        let cyaw = solve_axis(n_seg, |i| waypoints[i].yaw, &no_pins, durations, periodic)?;

        let segments = (0..n_seg)
            .map(|i| SplineSegment {
                cx: cx[i],
                cy: cy[i],
                cz: cz[i],
                cyaw: cyaw[i],
                duration: durations[i],
            })
            .collect();

        Ok(SplineTrajectory {
            segments,
            total_time: durations.iter().sum(),
        })
    }

    /// Plan a minimum-snap spline with optional per-waypoint acceleration constraints.
    ///
    /// `accel_pins`: list of `(waypoint_index, acceleration_m_s2)` pairs.  At each pinned
    /// waypoint the physical second derivative d²pos/dt² is forced to equal the given Vec3.
    /// This is the correct way to enforce a desired thrust direction at a waypoint (the
    /// attitude follows from differential flatness: body_z = normalize(a + g·ẑ)).
    ///
    /// Example — force inversion at the apex of a vertical loop (index 2, top waypoint):
    /// ```ignore
    /// SplineTrajectory::plan_with_accel_pins(&wps, &durs, &[(2, Vec3::new(0.0, 0.0, -1.5*9.81))], true)
    /// ```
    /// With a_z = −1.5 g: body_z = normalize(0, 0, −1.5g + g) = (0, 0, −1) → fully inverted.
    pub fn plan_with_accel_pins(
        waypoints: &[Waypoint],
        durations: &[f32],
        accel_pins: &[(usize, Vec3)],
        periodic: bool,
    ) -> Result<Self, String> {
        let n_wp = waypoints.len();
        let n_seg = n_wp - 1;
        assert_eq!(durations.len(), n_seg, "need n_seg durations");

        let mut pins_x = vec![None::<f32>; n_wp];
        let mut pins_y = vec![None::<f32>; n_wp];
        let mut pins_z = vec![None::<f32>; n_wp];
        for &(idx, a) in accel_pins {
            assert!(idx < n_wp, "accel_pin index {idx} out of range (n_wp={n_wp})");
            pins_x[idx] = Some(a.x);
            pins_y[idx] = Some(a.y);
            pins_z[idx] = Some(a.z);
        }
        let no_pins = vec![None::<f32>; n_wp];

        let cx = solve_axis(n_seg, |i| waypoints[i].pos.x, &pins_x, durations, periodic)?;
        let cy = solve_axis(n_seg, |i| waypoints[i].pos.y, &pins_y, durations, periodic)?;
        let cz = solve_axis(n_seg, |i| waypoints[i].pos.z, &pins_z, durations, periodic)?;
        let cyaw = solve_axis(n_seg, |i| waypoints[i].yaw, &no_pins, durations, periodic)?;

        let segments = (0..n_seg)
            .map(|i| SplineSegment {
                cx: cx[i], cy: cy[i], cz: cz[i], cyaw: cyaw[i],
                duration: durations[i],
            })
            .collect();

        Ok(SplineTrajectory { segments, total_time: durations.iter().sum() })
    }

    /// Evaluate flat outputs at time `t` (clamped to [0, total_time]).
    pub fn eval(&self, t: f32) -> FlatOutput {
        let t = t.clamp(0.0, self.total_time);
        // Find segment
        let mut trem = t;
        let mut seg_idx = 0usize;
        for (i, seg) in self.segments.iter().enumerate() {
            if trem <= seg.duration || i == self.segments.len() - 1 {
                seg_idx = i;
                break;
            }
            trem -= seg.duration;
        }
        let seg = &self.segments[seg_idx];
        let t_norm = (trem / seg.duration).clamp(0.0, 1.0);
        let dt = seg.duration; // T for this segment

        // Helper: evaluate polynomial and its derivatives using f32 coefficients
        let eval_f32 = |c: &[f32; 9]| -> f32 {
            c.iter().enumerate().map(|(i, &ai)| ai * t_norm.powi(i as i32)).sum()
        };
        let eval_d1_f32 = |c: &[f32; 9]| -> f32 {
            let factors = [0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0];
            c.iter().enumerate().skip(1).map(|(i, &ai)| ai * factors[i] * t_norm.powi(i as i32 - 1)).sum::<f32>() / dt
        };
        let eval_d2_f32 = |c: &[f32; 9]| -> f32 {
            let factors = [0.0, 0.0, 2.0, 6.0, 12.0, 20.0, 30.0, 42.0, 56.0];
            c.iter().enumerate().skip(2).map(|(i, &ai)| ai * factors[i] * t_norm.powi(i as i32 - 2)).sum::<f32>() / (dt * dt)
        };
        let eval_d3_f32 = |c: &[f32; 9]| -> f32 {
            let factors = [0.0, 0.0, 0.0, 6.0, 24.0, 60.0, 120.0, 210.0, 336.0];
            c.iter().enumerate().skip(3).map(|(i, &ai)| ai * factors[i] * t_norm.powi(i as i32 - 3)).sum::<f32>() / (dt * dt * dt)
        };
        let eval_d4_f32 = |c: &[f32; 9]| -> f32 {
            let factors = [0.0, 0.0, 0.0, 0.0, 24.0, 120.0, 360.0, 840.0, 1680.0];
            c.iter().enumerate().skip(4).map(|(i, &ai)| ai * factors[i] * t_norm.powi(i as i32 - 4)).sum::<f32>() / (dt * dt * dt * dt)
        };

        FlatOutput {
            pos: Vec3::new(eval_f32(&seg.cx), eval_f32(&seg.cy), eval_f32(&seg.cz)),
            vel: Vec3::new(eval_d1_f32(&seg.cx), eval_d1_f32(&seg.cy), eval_d1_f32(&seg.cz)),
            acc: Vec3::new(eval_d2_f32(&seg.cx), eval_d2_f32(&seg.cy), eval_d2_f32(&seg.cz)),
            jerk: Vec3::new(eval_d3_f32(&seg.cx), eval_d3_f32(&seg.cy), eval_d3_f32(&seg.cz)),
            snap: Vec3::new(eval_d4_f32(&seg.cx), eval_d4_f32(&seg.cy), eval_d4_f32(&seg.cz)),
            yaw: eval_f32(&seg.cyaw),
            yaw_dot: eval_d1_f32(&seg.cyaw),
            yaw_ddot: eval_d2_f32(&seg.cyaw),
        }
    }
}

/// Solve a single-axis minimum-snap QP.
///
/// `accel_at[k]`: if `Some(a)`, pin the physical acceleration d²p/dt² at waypoint k to `a`.
/// Length must equal `n_seg + 1`.  Interior pins add an extra equality constraint on
/// `poly_d2(seg_k, 0.0) = a * T_k²`.  Boundary pins override the rest-to-rest d²=0 condition.
///
/// Returns a Vec of [f32; 9] coefficient arrays, one per segment, normalised
/// to the unit interval [0, 1].  Physical-time derivatives are recovered by
/// dividing by the appropriate power of `duration`.
fn solve_axis<F>(
    n_seg: usize, wp: F, accel_at: &[Option<f32>], durations: &[f32], periodic: bool,
) -> Result<Vec<[f32; 9]>, String>
where
    F: Fn(usize) -> f32,
{
    const N_COEFF: usize = 9; // 8th-order polynomial

    let mut problem = ProblemVariables::default();
    // a[seg][coeff]: n_seg * 9 variables
    let vars: Vec<Vec<Variable>> = (0..n_seg)
        .map(|_| problem.add_vector(N_COEFF, None, None))
        .collect();

    // ── Objective: sum of snap² integrals ───────────────────────────────────
    let mut objective: QuadraticExpression = QuadraticExpression::from(0.0_f32);
    for seg in 0..n_seg {
        // Snap integral on [0,1]; scale by duration^7 (see derivation)
        // ∫₀ᵀ snap(τ)² dτ = (1/T⁷) ∫₀¹ snap(t)² dt  (change of vars τ=Tt)
        // We scale the objective weight by 1/T^7 to penalise faster segments less.
        let t = durations[seg];
        let scale = 1.0 / (t * t * t * t * t * t * t); // 1/T^7
        objective = objective + snap_cost(&vars[seg]) * scale;
    }

    // ── Constraints ─────────────────────────────────────────────────────────
    let mut constraints = Vec::new();

    // Boundary: position at start and end always fixed
    constraints.push(constraint!(poly(&vars[0], 0.0) == wp(0)));
    let last = n_seg - 1;
    constraints.push(constraint!(poly(&vars[last], 1.0) == wp(n_seg)));

    if periodic {
        // Periodic: derivatives 1–4 at end of last segment = at start of first segment.
        // In physical time: (1/T_last^n) * poly_dn(last, 1) = (1/T_0^n) * poly_dn(0, 0)
        // => T_0^n * poly_dn(last, 1) = T_last^n * poly_dn(0, 0)
        let t0 = durations[0];
        let tl = durations[last];
        for deriv in 1u32..=4 {
            let sl = t0.powi(deriv as i32);  // scale for end side
            let sr = tl.powi(deriv as i32);  // scale for start side
            match deriv {
                1 => constraints.push(constraint!(poly_d1(&vars[last], 1.0) * sl == poly_d1(&vars[0], 0.0) * sr)),
                2 => constraints.push(constraint!(poly_d2(&vars[last], 1.0) * sl == poly_d2(&vars[0], 0.0) * sr)),
                3 => constraints.push(constraint!(poly_d3(&vars[last], 1.0) * sl == poly_d3(&vars[0], 0.0) * sr)),
                4 => constraints.push(constraint!(poly_d4(&vars[last], 1.0) * sl == poly_d4(&vars[0], 0.0) * sr)),
                _ => unreachable!(),
            }
        }
    } else {
        // Rest-to-rest: derivatives 1–4 zero at start and end.
        // d2 constraints are skipped when overridden by an accel_at pin.
        constraints.push(constraint!(poly_d1(&vars[0], 0.0) == 0.0_f32));
        if accel_at[0].is_none() {
            constraints.push(constraint!(poly_d2(&vars[0], 0.0) == 0.0_f32));
        }
        constraints.push(constraint!(poly_d3(&vars[0], 0.0) == 0.0_f32));
        constraints.push(constraint!(poly_d4(&vars[0], 0.0) == 0.0_f32));
        constraints.push(constraint!(poly_d1(&vars[last], 1.0) == 0.0_f32));
        if accel_at[n_seg].is_none() {
            constraints.push(constraint!(poly_d2(&vars[last], 1.0) == 0.0_f32));
        }
        constraints.push(constraint!(poly_d3(&vars[last], 1.0) == 0.0_f32));
        constraints.push(constraint!(poly_d4(&vars[last], 1.0) == 0.0_f32));
        // Boundary acceleration pins (replace the zero constraints above)
        if let Some(a) = accel_at[0] {
            constraints.push(constraint!(poly_d2(&vars[0], 0.0) == a * durations[0].powi(2)));
        }
        if let Some(a) = accel_at[n_seg] {
            constraints.push(constraint!(poly_d2(&vars[last], 1.0) == a * durations[last].powi(2)));
        }
    }

    // Interior waypoints: position match + continuity up to snap
    for k in 0..n_seg - 1 {
        let t_k = durations[k];
        let t_k1 = durations[k + 1];

        // Position at end of segment k == waypoint k+1
        constraints.push(constraint!(poly(&vars[k], 1.0) == wp(k + 1)));
        // Position at start of segment k+1 == waypoint k+1
        constraints.push(constraint!(poly(&vars[k + 1], 0.0) == wp(k + 1)));

        // Continuity: d^n/dτ^n matches at junction for n = 1, 2, 3, 4
        // dⁿp/dτⁿ = (1/Tⁿ) dⁿp/dtⁿ  at t=1 (left) == (1/T'ⁿ) dⁿp/dtⁿ at t=0 (right)
        // In unit-t space: (1/t_k^n) poly_dn(k, 1) == (1/t_k1^n) poly_dn(k+1, 0)
        // Rearranging: t_k1^n * poly_dn(k, 1) == t_k^n * poly_dn(k+1, 0)
        for deriv in 1u32..=4 {
            let scale_left = t_k1.powi(deriv as i32);
            let scale_right = t_k.powi(deriv as i32);
            match deriv {
                1 => constraints.push(constraint!(
                    poly_d1(&vars[k], 1.0) * scale_left == poly_d1(&vars[k + 1], 0.0) * scale_right
                )),
                2 => constraints.push(constraint!(
                    poly_d2(&vars[k], 1.0) * scale_left == poly_d2(&vars[k + 1], 0.0) * scale_right
                )),
                3 => constraints.push(constraint!(
                    poly_d3(&vars[k], 1.0) * scale_left == poly_d3(&vars[k + 1], 0.0) * scale_right
                )),
                4 => constraints.push(constraint!(
                    poly_d4(&vars[k], 1.0) * scale_left == poly_d4(&vars[k + 1], 0.0) * scale_right
                )),
                _ => unreachable!(),
            }
        }

        // Acceleration pin at interior waypoint k+1: d²p/dt² = accel_at[k+1]
        // In normalised time: poly_d2(seg_{k+1}, 0) = a * T_{k+1}²
        if let Some(a) = accel_at[k + 1] {
            constraints.push(constraint!(poly_d2(&vars[k + 1], 0.0) == a * t_k1.powi(2)));
        }
    }

    // ── Solve ────────────────────────────────────────────────────────────────
    let solver = ClarabelSolver::default();
    let res = solver
        .solve(problem, objective, constraints)
        .map_err(|e| format!("QP solver failed: {:?}", e))?;

    // Extract solutions
    let mut out = Vec::with_capacity(n_seg);
    for seg in 0..n_seg {
        let sol = res.eval_vec(&vars[seg]);
        let mut arr = [0.0f32; 9];
        for (i, v) in sol.iter().enumerate() {
            arr[i] = *v as f32;
        }
        out.push(arr);
    }
    Ok(out)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn plan_simple_trajectory() {
        let wps = vec![
            Waypoint { pos: Vec3::new(0.0, 0.0, 1.0), yaw: 0.0 },
            Waypoint { pos: Vec3::new(1.0, 0.0, 1.0), yaw: 0.0 },
            Waypoint { pos: Vec3::new(1.0, 1.0, 1.0), yaw: 0.0 },
        ];
        let durs = vec![1.0, 1.0];
        let traj = SplineTrajectory::plan(&wps, &durs, false).expect("plan failed");

        assert_eq!(traj.segments.len(), 2);

        let start = traj.eval(0.0);
        let mid   = traj.eval(1.0);
        let end   = traj.eval(2.0);

        assert!((start.pos.x - 0.0).abs() < 1e-3, "start x");
        assert!((mid.pos.x - 1.0).abs() < 1e-2, "mid x");
        assert!((end.pos.y - 1.0).abs() < 1e-3, "end y");

        // Velocity should be near zero at start and end
        assert!(start.vel.x.abs() < 1e-3, "start vx");
        assert!(end.vel.y.abs() < 1e-3, "end vy");
    }
}
