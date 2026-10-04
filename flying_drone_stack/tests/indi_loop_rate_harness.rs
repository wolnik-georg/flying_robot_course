//! Host harness: 1-axis INDI attitude chain + measured brushless actuator lag (τ≈44 ms),
//! extended from `test_indi_actuator_lag.rs` with 1 kHz loop, filter design dt / prewarp /
//! notch, and optional 500 Hz output hold (harness-only decimation).
//!
//! Writes sweep JSON to `experiments/analysis/out/indi_loop_rates/harness_sweep.json`
//! when `INDI_HARNESS_SWEEP=1`.

use std::f32::consts::PI;
use std::path::PathBuf;

const J: f32 = 23.951e-6;
const TAU_CLAMP: f32 = 0.014;
const TAU_ACT: f32 = 0.044;
const PLANT_DEAD_TICKS: usize = 2;
const KR: f32 = 2400.0;
const KW: f32 = 170.0;
const FC_BW: f32 = 206.0;

#[derive(Clone, Copy)]
struct Bw2 {
    b: f32,
    a1: f32,
    a2: f32,
    x1: f32,
    x2: f32,
    y1: f32,
    y2: f32,
}

impl Bw2 {
    fn init(&mut self, fc: f32, dt: f32, prewarp: bool) {
        let tau = 1.0 / (2.0 * PI * fc);
        if prewarp {
            let q = 0.7071_f32;
            let k = (dt / (2.0 * tau)).tan();
            let poly = k * k + k / q + 1.0;
            self.b = k * k / poly;
            self.a1 = 2.0 * (k * k - 1.0) / poly;
            self.a2 = (k * k - k / q + 1.0) / poly;
        } else {
            let s2 = std::f32::consts::SQRT_2;
            let denom = tau * tau + s2 * tau * dt + dt * dt;
            self.b = dt * dt / denom;
            self.a1 = 2.0 * (dt * dt - tau * tau) / denom;
            self.a2 = (tau * tau - s2 * tau * dt + dt * dt) / denom;
        }
        self.x1 = 0.0;
        self.x2 = 0.0;
        self.y1 = 0.0;
        self.y2 = 0.0;
    }
    fn update(&mut self, x: f32) -> f32 {
        let y = self.b * x + 2.0 * self.b * self.x1 + self.b * self.x2 - self.a1 * self.y1 - self.a2 * self.y2;
        self.x2 = self.x1;
        self.x1 = x;
        self.y2 = self.y1;
        self.y1 = y;
        y
    }
}

#[derive(Clone, Copy)]
struct Notch {
    b0: f32,
    b1: f32,
    b2: f32,
    a1: f32,
    a2: f32,
    x1: f32,
    x2: f32,
    y1: f32,
    y2: f32,
}

impl Notch {
    fn init(&mut self, f0: f32, bw: f32, dt: f32) {
        let w0 = 2.0 * PI * f0 * dt;
        let q = (f0 / bw.max(0.1)).max(0.1);
        let alpha = (w0).sin() / (2.0 * q);
        let c = w0.cos();
        let a0 = 1.0 + alpha;
        self.b0 = 1.0 / a0;
        self.b1 = -2.0 * c / a0;
        self.b2 = 1.0 / a0;
        self.a1 = -2.0 * c / a0;
        self.a2 = (1.0 - alpha) / a0;
        self.x1 = 0.0;
        self.x2 = 0.0;
        self.y1 = 0.0;
        self.y2 = 0.0;
    }
    fn update(&mut self, x: f32) -> f32 {
        let y = self.b0 * x + self.b1 * self.x1 + self.b2 * self.x2 - self.a1 * self.y1 - self.a2 * self.y2;
        self.x2 = self.x1;
        self.x1 = x;
        self.y2 = self.y1;
        self.y1 = y;
        y
    }
}

struct Cfg {
    loop_dt: f32,
    filt_dt_us: u32,
    prewarp: bool,
    notch_en: bool,
    decimate_hold: bool,
}

struct SimOut {
    omega_sigma: f32,
    freq_hz: f32,
    stable: bool,
}

fn run(cfg: Cfg) -> SimOut {
    let dt = cfg.loop_dt;
    let dt_filt = if cfg.filt_dt_us == 0 {
        dt
    } else {
        cfg.filt_dt_us as f32 * 1e-6
    };
    let mut bw_pre = Bw2 {
        b: 0.0,
        a1: 0.0,
        a2: 0.0,
        x1: 0.0,
        x2: 0.0,
        y1: 0.0,
        y2: 0.0,
    };
    let mut bw_ref = bw_pre;
    let mut bw_tau = bw_pre;
    let mut notch_m = Notch {
        b0: 1.0,
        b1: 0.0,
        b2: 0.0,
        a1: 0.0,
        a2: 0.0,
        x1: 0.0,
        x2: 0.0,
        y1: 0.0,
        y2: 0.0,
    };
    let mut notch_r = notch_m;
    bw_pre.init(FC_BW, dt_filt, cfg.prewarp);
    bw_ref = bw_pre;
    bw_tau = bw_pre;
    bw_ref.init(FC_BW, dt_filt, cfg.prewarp);
    bw_tau.init(FC_BW, dt_filt, cfg.prewarp);
    if cfg.notch_en {
        notch_m.init(6.9, 3.0, dt_filt);
        notch_r = notch_m;
        notch_r.init(6.9, 3.0, dt_filt);
    }

    let (mut theta, mut omega) = (0.05_f32, 0.0_f32);
    let mut tau_applied = 0.0_f32;
    let mut tau_prev_cmd = 0.0_f32;
    let mut tau_hold = 0.0_f32;
    let mut dead = [0.0_f32; PLANT_DEAD_TICKS];
    let k_plant = dt / (dt + TAU_ACT);
    let mut rpm_delay = [0.0_f32; 2];
    let mut omega_filt_prev = 0.0_f32;

    let n = (12.0 / dt) as usize;
    let mut trace = Vec::with_capacity(n);
    for tick in 0..n {
        let update_law = !cfg.decimate_hold || (tick % 2 == 0);
        if update_law {
            let omega_filt = bw_pre.update(omega);
            let mut alpha_meas = (omega_filt - omega_filt_prev) / dt;
            omega_filt_prev = omega_filt;
            if cfg.notch_en {
                alpha_meas = notch_m.update(alpha_meas);
            }

            let er = theta.sin();
            let mut alpha_ref = -KR * er - KW * omega;
            alpha_ref = bw_ref.update(alpha_ref);
            if cfg.notch_en {
                alpha_ref = notch_r.update(alpha_ref);
            }

            let base_raw = rpm_delay[1];
            let base = bw_tau.update(base_raw);
            tau_prev_cmd = (base + J * (alpha_ref - alpha_meas)).clamp(-TAU_CLAMP, TAU_CLAMP);
            tau_hold = tau_prev_cmd;
        }

        let tau_cmd = tau_hold;
        let tau_delayed = dead[PLANT_DEAD_TICKS - 1];
        for i in (1..PLANT_DEAD_TICKS).rev() {
            dead[i] = dead[i - 1];
        }
        dead[0] = tau_cmd;
        tau_applied += (tau_delayed - tau_applied) * k_plant;
        rpm_delay[1] = rpm_delay[0];
        rpm_delay[0] = tau_applied;
        let alpha = tau_applied / J;
        omega += alpha * dt;
        theta += omega * dt;
        trace.push(omega);
    }

    let tail = &trace[trace.len() - (4.0 / dt) as usize..];
    let mean = tail.iter().sum::<f32>() / tail.len() as f32;
    let sigma = (tail.iter().map(|w| (w - mean).powi(2)).sum::<f32>() / tail.len() as f32).sqrt();
    let mut crossings = 0usize;
    for w in tail.windows(2) {
        if (w[0] - mean) * (w[1] - mean) < 0.0 {
            crossings += 1;
        }
    }
    let freq = crossings as f32 / 2.0 / 4.0;
    SimOut {
        omega_sigma: sigma,
        freq_hz: freq,
        stable: sigma < 0.05,
    }
}

fn bode_fc_realized(fc: f32, dt: f32, prewarp: bool) -> f32 {
    // Report design fc; with legacy non-prewarp at 1kHz call, -3dB shifts (~1.9x) per docs/22.
    if prewarp {
        fc
    } else if dt < 0.0015 {
        fc * 1.9
    } else {
        fc * 1.35
    }
}

#[test]
fn harness_validates_limit_cycle_at_flown_gains() {
    // Oct-02 yaml card (crazyflies.yaml): filt_dt_us=1000, filt_prewarp=1, notch_en=0
    let r = run(Cfg {
        loop_dt: 0.001,
        filt_dt_us: 1000,
        prewarp: true,
        notch_en: false,
        decimate_hold: false,
    });
    println!(
        "validation: sigma={:.3} rad/s freq={:.2} Hz",
        r.omega_sigma, r.freq_hz
    );
    assert!(
        r.omega_sigma > 0.5,
        "expected limit cycle (illustrative harness), sigma={}",
        r.omega_sigma
    );
    assert!(
        r.freq_hz > 3.0 && r.freq_hz < 10.0,
        "freq {} Hz outside 3–10",
        r.freq_hz
    );
}

#[test]
fn harness_sweep_filter_and_rate_grid() {
    if std::env::var("INDI_HARNESS_SWEEP").ok().as_deref() != Some("1") {
        println!("skip sweep (set INDI_HARNESS_SWEEP=1 to write JSON)");
        return;
    }
    let mut rows = Vec::new();
    for &filt_us in &[2000u32, 1000u32] {
        for &pre in &[false, true] {
            for &notch in &[false, true] {
                for &decim in &[false, true] {
                    let cfg = Cfg {
                        loop_dt: 0.001,
                        filt_dt_us: filt_us,
                        prewarp: pre,
                        notch_en: notch,
                        decimate_hold: decim,
                    };
                    let r = run(cfg);
                    rows.push(format!(
                        "{{\"filt_dt_us\":{},\"prewarp\":{},\"notch_en\":{},\"decimate_hold\":{},\
\"omega_sigma\":{:.4},\"freq_hz\":{:.3},\"stable\":{},\"fc3db_hz\":{:.1}}}",
                        filt_us,
                        pre,
                        notch,
                        decim,
                        r.omega_sigma,
                        r.freq_hz,
                        r.stable,
                        bode_fc_realized(FC_BW, if filt_us == 0 { 0.001 } else { filt_us as f32 * 1e-6 }, pre)
                    ));
                }
            }
        }
    }
    let json = format!("[\n{}\n]", rows.join(",\n"));
    let out = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("../experiments/analysis/out/indi_loop_rates/harness_sweep.json");
    std::fs::create_dir_all(out.parent().unwrap()).ok();
    std::fs::write(&out, json).expect("write sweep");
    println!("wrote {}", out.display());
}
