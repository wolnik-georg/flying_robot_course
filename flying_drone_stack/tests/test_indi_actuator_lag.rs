//! Validation of the act_dyn INDI base against the BENCH-MEASURED brushless actuator lag
//! (2026-07-22, docs/investigation §16: linear first-order pole fc = 3.6 Hz, τ ≈ 44 ms).
//!
//! Replicates the FIRMWARE INDI attitude chain (firmware_app/src/lib.rs, flown config:
//! filt_order=1, filt_tau=1, fc_bw=60, 500 Hz) on a single roll axis with a plant that
//! includes the measured actuator pole, and compares the two increment-base variants:
//!   - RPM base (today's behaviour, indi_gains.act_tau = 0): base = actual lagged torque
//!     (the optical RPM deck measures the real rotor state — synchronized, per §15.4).
//!   - act_dyn base (proposed fix, act_tau > 0): base = first-order model of the previous
//!     COMMANDS with the measured time constant (Smeur / stock controller_indi.c structure).
//!
//! Expected (loop analysis §16): at brushless gains kr=2400/kw=170 with a 44 ms actuator,
//! the RPM base is linearly unstable (|L|≈2.1 @ ≈−189°) → sustained ~6.3 Hz limit cycle;
//! the act_dyn base inverts the pole → converges. With a fast actuator (τ=5 ms) the RPM
//! base is fine — which is why the brushed drones never shook (regression case).

const DT: f32 = 0.002; // 500 Hz
const J_TRUE: f32 = 23.951e-6;
const J_MODEL: f32 = 23.951e-6;
const TAU_CLAMP: f32 = 0.014; // physical torque headroom (≈ mixer saturation)

/// Same bilinear 2nd-order Butterworth as firmware_app/src/lib.rs (fc=60 Hz, 500 Hz).
struct Bw2 { b: f32, a1: f32, a2: f32, x1: f32, x2: f32, y1: f32, y2: f32 }
impl Bw2 {
    fn new(fc: f32) -> Self {
        let tau = 1.0 / (2.0 * std::f32::consts::PI * fc);
        let d = tau * tau + std::f32::consts::SQRT_2 * tau * DT + DT * DT;
        Bw2 { b: DT * DT / d, a1: 2.0 * (DT * DT - tau * tau) / d,
              a2: (tau * tau - std::f32::consts::SQRT_2 * tau * DT + DT * DT) / d,
              x1: 0.0, x2: 0.0, y1: 0.0, y2: 0.0 }
    }
    fn update(&mut self, x: f32) -> f32 {
        let y = self.b * (x + 2.0 * self.x1 + self.x2) - self.a1 * self.y1 - self.a2 * self.y2;
        self.x2 = self.x1; self.x1 = x;
        self.y2 = self.y1; self.y1 = y;
        y
    }
}

struct SimResult { omega_sigma: f32, freq_hz: f32 }

// Bench §16: the fitted 3.6 Hz pole alone predicts −60° at 6.3 Hz but −68° was measured
// (−70° vs −81° at 10 Hz) → the chain carries ~4 ms of additional dead time (DShot frame +
// ESC commutation + rev-averaged RPM sensing). Modelled as 2 ticks at 500 Hz.
const PLANT_DEAD_TICKS: usize = 2;

/// 1-axis closed-loop sim. act_model_tau: None = RPM base, Some(τ) = act_dyn base.
fn run(kr: f32, kw: f32, tau_act_plant: f32, act_model_tau: Option<f32>) -> SimResult {
    let (mut theta, mut omega) = (0.05_f32, 0.0_f32); // ~3° initial error kicks the loop
    let mut tau_applied = 0.0_f32;                    // plant actuator state (real pole)
    let mut tau_prev_cmd = 0.0_f32;
    let mut act_model = 0.0_f32;                      // act_dyn base state
    let mut dead = [0.0_f32; PLANT_DEAD_TICKS];       // plant dead time (measured excess phase)
    let k_plant = DT / (DT + tau_act_plant);

    let mut bw_pre = Bw2::new(60.0);  // gyro pre-filter (filt_order=1)
    let mut bw_ref = Bw2::new(60.0);  // alpha_ref filter
    let mut bw_tau = Bw2::new(60.0);  // base filter (filt_tau=1)
    let mut omega_filt_prev = 0.0_f32;

    // Optical RPM sensor: per-rev averaging ≈ 1-2 samples at 500 Hz (§15.4: synchronized)
    let mut rpm_delay = [0.0_f32; 2];

    let n = (10.0 / DT) as usize;
    let mut trace: Vec<f32> = Vec::with_capacity(n);
    for _ in 0..n {
        // ── firmware INDI chain (lib.rs controller_step, INDI branch) ──
        let omega_filt = bw_pre.update(omega);
        let alpha_meas = (omega_filt - omega_filt_prev) / DT;
        omega_filt_prev = omega_filt;

        let er = theta.sin();
        let alpha_ref = -kr * er - kw * omega;
        let alpha_ref_f = bw_ref.update(alpha_ref);

        let base_raw = match act_model_tau {
            None => rpm_delay[1], // RPM base: actual (lagged) torque, small sensor delay
            Some(tm) => {
                let km = DT / (DT + tm);
                act_model += (tau_prev_cmd - act_model) * km;
                act_model
            }
        };
        let base = bw_tau.update(base_raw);

        let tau_cmd = (base + J_MODEL * (alpha_ref_f - alpha_meas))
            .clamp(-TAU_CLAMP, TAU_CLAMP);
        tau_prev_cmd = tau_cmd;

        // ── plant: dead time + measured first-order actuator pole + rigid body ──
        let tau_delayed = dead[PLANT_DEAD_TICKS - 1];
        for i in (1..PLANT_DEAD_TICKS).rev() { dead[i] = dead[i - 1]; }
        dead[0] = tau_cmd;
        tau_applied += (tau_delayed - tau_applied) * k_plant;
        rpm_delay[1] = rpm_delay[0];
        rpm_delay[0] = tau_applied;
        let alpha = tau_applied / J_TRUE;
        omega += alpha * DT;
        theta += omega * DT;
        trace.push(omega);
    }

    // Metrics over the last 4 s (settled region)
    let tail = &trace[trace.len() - (4.0 / DT) as usize..];
    let mean = tail.iter().sum::<f32>() / tail.len() as f32;
    let sigma = (tail.iter().map(|w| (w - mean) * (w - mean)).sum::<f32>()
        / tail.len() as f32).sqrt();
    // Dominant frequency from mean-crossings
    let mut crossings = 0usize;
    for w in tail.windows(2) {
        if (w[0] - mean) * (w[1] - mean) < 0.0 { crossings += 1; }
    }
    let freq = crossings as f32 / 2.0 / 4.0;
    SimResult { omega_sigma: sigma, freq_hz: freq }
}

#[test]
fn rpm_base_limit_cycles_with_measured_brushless_actuator() {
    // Today's law + bench-measured actuator (44 ms pole + ~4 ms dead time) at the flown
    // brushless gains → sustained limit cycle. Matches flight (6.3 Hz; sim shows ~5 Hz —
    // crude 1-axis model, in-flight extra lags shift the exact frequency).
    let r = run(2400.0, 170.0, 0.044, None);
    println!("RPM base, kr=2400: omega sigma = {:.3} rad/s at {:.1} Hz", r.omega_sigma, r.freq_hz);
    assert!(r.omega_sigma > 0.5, "expected sustained oscillation, got sigma={}", r.omega_sigma);
    assert!(r.freq_hz > 3.0 && r.freq_hz < 10.0, "cycle at {} Hz", r.freq_hz);
}

#[test]
fn act_dyn_with_matched_model_is_equivalent_to_rpm_base() {
    // KEY NEGATIVE RESULT (2026-07-22): an act_dyn base whose model MATCHES the real
    // actuator produces the same signal the RPM deck already measures → identical loop,
    // identical instability. "Add act_dyn" is NOT a fix for the brushless shake; the RPM
    // base was never the deficiency. (Refutes doc §16's initial fix proposal — kept as a
    // regression guard against re-proposing it.)
    let a = run(2400.0, 170.0, 0.044, None);
    let b = run(2400.0, 170.0, 0.044, Some(0.044));
    println!("kr=2400: rpm sigma={:.2}, act_dyn(44ms) sigma={:.2}", a.omega_sigma, b.omega_sigma);
    assert!(b.omega_sigma > 0.5, "act_dyn(matched) unexpectedly stabilized: {}", b.omega_sigma);
    let rel = (a.omega_sigma - b.omega_sigma).abs() / a.omega_sigma;
    assert!(rel < 0.2, "expected near-identical behaviour, rel diff {}", rel);
}

#[test]
fn stability_boundary_sits_near_kr_800() {
    // The measured actuator imposes a physical attitude-stiffness ceiling: stable at
    // kr=600, unstable at kr=1050 (zeta=1.74 ladder) → ceiling ~kr 800 in this model.
    // Flight shows shake even at ~600 → in-flight extra lag (EKF attitude in eR,
    // position-loop coupling, voltage sag) pushes the real ceiling lower — documented gap.
    let lo = run(603.0, 85.0, 0.044, None);
    let hi = run(1050.0, 112.0, 0.044, None);
    println!("kr=603: sigma={:.3}; kr=1050: sigma={:.3}", lo.omega_sigma, hi.omega_sigma);
    assert!(lo.omega_sigma < 0.05, "kr=603 should be stable in this model");
    assert!(hi.omega_sigma > 0.5, "kr=1050 should be unstable in this model");
}

#[test]
fn rpm_base_stays_valid_with_fast_actuator() {
    // Brushed-drone regression: with a fast actuator (tau=5 ms) even kr=2400 is stable →
    // explains why only the brushless shakes IF the brushed spool is fast (bench pending).
    let r = run(2400.0, 170.0, 0.005, None);
    println!("RPM base, tau_act=5ms, kr=2400: sigma = {:.4} rad/s", r.omega_sigma);
    assert!(r.omega_sigma < 0.05, "fast-actuator case should be stable, got {}", r.omega_sigma);
}

#[test]
fn overestimated_act_dyn_model_acts_as_lead_and_extends_boundary() {
    // Curiosity, NOT a validated fix: act_dyn with a deliberately 2x-slow model (88 ms)
    // leaves a lead-like residual that roughly doubles the stable-kr ceiling in this
    // NOISE-FREE sim. Real gyro noise will pay for that lead — do not deploy without a
    // noise study. Recorded so the observation isn't lost.
    let r = run(1500.0, 134.0, 0.044, Some(0.088));
    let base = run(1500.0, 134.0, 0.044, None);
    println!("kr=1500: rpm sigma={:.3}, act_dyn(88ms) sigma={:.3}", base.omega_sigma, r.omega_sigma);
    assert!(base.omega_sigma > 0.5 && r.omega_sigma < 0.05,
        "expected lead-trick stabilization at kr=1500");
}
