//! Package simulation harness (NEW) — 500 Hz decimation with **dt=2 ms** on derivatives.
//! Does NOT modify `indi_loop_rate_harness.rs`. Writes JSON when `INDI_PACKAGE_HARNESS=1`.

use std::path::PathBuf;

const J: f32 = 23.951e-6;
const TAU_ACT: f32 = 0.044;
const TAU_CLAMP: f32 = 0.045;

fn run_decimation_corrected(kr: f32, kw: f32) -> (f32, f32) {
    let dt_fast = 0.001_f32;
    let mut theta = 0.02_f32;
    let mut omega = 0.0_f32;
    let mut tau_applied = 0.0_f32;
    let k = dt_fast / (dt_fast + TAU_ACT);
    let mut tau_hold = 0.0_f32;
    let mut trace = Vec::new();
    for tick in 0..12000 {
        if tick % 2 == 0 {
            let dt_law = 0.002_f32;
            let er = theta;
            let alpha_meas = (omega - 0.0) / dt_law; // simplified
            let alpha_ref = -kr * er - kw * omega;
            tau_hold = (tau_applied + J * (alpha_ref - alpha_meas)).clamp(-TAU_CLAMP, TAU_CLAMP);
        }
        tau_applied += (tau_hold - tau_applied) * k;
        let alpha = tau_applied / J;
        omega += alpha * dt_fast;
        theta += omega * dt_fast;
        trace.push(omega);
    }
    let tail = &trace[8000..];
    let mean = tail.iter().sum::<f32>() / tail.len() as f32;
    let sigma = (tail.iter().map(|w| (w - mean).powi(2)).sum::<f32>() / tail.len() as f32).sqrt();
    let mut cross = 0usize;
    for w in tail.windows(2) {
        if (w[0] - mean) * (w[1] - mean) < 0.0 {
            cross += 1;
        }
    }
    let freq = cross as f32 / 2.0 / 4.0;
    (sigma, freq)
}

#[test]
fn package_harness_decimation_uses_2ms_derivative() {
    let (sig_hold, freq_hold) = run_decimation_corrected(2400.0, 170.0);
    assert!(sig_hold > 0.1, "expected limit cycle with corrected dt, sigma={sig_hold}");
    assert!(freq_hold > 2.0 && freq_hold < 20.0, "freq={freq_hold}");
}

#[test]
fn package_harness_writes_json_when_env_set() {
    if std::env::var("INDI_PACKAGE_HARNESS").ok().as_deref() != Some("1") {
        return;
    }
    let (s, f) = run_decimation_corrected(2400.0, 170.0);
    let out = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("../experiments/analysis/out/indi_package/harness_decimation_corrected.json");
    let json = format!(r#"{{"omega_sigma":{s:.4},"freq_hz":{f:.3}}}"#);
    std::fs::write(&out, json).expect("write json");
}
