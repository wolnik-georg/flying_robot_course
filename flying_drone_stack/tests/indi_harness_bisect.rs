//! Bisection of 1-axis harness elements (NEW — does not modify existing harness files).
//! Run: `cargo test -p flying_drone_stack indi_harness_bisect -- --nocapture`
//! JSON: `INDI_HARNESS_BISECT=1 cargo test ...`

use std::f32::consts::PI;
use std::path::PathBuf;

const J: f32 = 23.951e-6;
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
pub struct BisectCfg {
    pub loop_dt: f32,
    pub filt_dt_us: u32,
    pub prewarp: bool,
    pub kr: f32,
    pub kw: f32,
    pub tau_act: f32,
    pub dead_ticks: usize,
    pub rpm_base_lag: usize,
    pub use_bw_pre: bool,
    pub use_bw_ref: bool,
    pub use_bw_tau: bool,
    pub use_clamp: bool,
    pub tau_clamp: f32,
    pub spool_asym: f32,
}

impl Default for BisectCfg {
    fn default() -> Self {
        Self {
            loop_dt: 0.001,
            filt_dt_us: 1000,
            prewarp: true,
            kr: KR,
            kw: KW,
            tau_act: 0.044,
            dead_ticks: 2,
            rpm_base_lag: 1,
            use_bw_pre: true,
            use_bw_ref: true,
            use_bw_tau: true,
            use_clamp: true,
            tau_clamp: 0.014,
            spool_asym: 1.0,
        }
    }
}

pub struct BisectOut {
    pub omega_sigma: f32,
    pub freq_hz: f32,
    pub limit_cycle: bool,
}

pub fn run_bisect(cfg: BisectCfg) -> BisectOut {
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
    if cfg.use_bw_pre {
        bw_pre.init(FC_BW, dt_filt, cfg.prewarp);
    }
    if cfg.use_bw_ref {
        bw_ref.init(FC_BW, dt_filt, cfg.prewarp);
    }
    if cfg.use_bw_tau {
        bw_tau.init(FC_BW, dt_filt, cfg.prewarp);
    }

    let (mut theta, mut omega) = (0.05_f32, 0.0_f32);
    let mut tau_applied = 0.0_f32;
    let mut tau_hold = 0.0_f32;
    let mut omega_filt_prev = omega;
    let dead_n = cfg.dead_ticks;
    let mut dead = vec![0.0_f32; dead_n.max(1)];
    let lag_n = cfg.rpm_base_lag.max(1);
    let mut rpm_hist = vec![0.0_f32; lag_n + 1];

    let n = (12.0 / dt) as usize;
    let mut trace = Vec::with_capacity(n);
    for _tick in 0..n {
        let omega_filt = if cfg.use_bw_pre {
            bw_pre.update(omega)
        } else {
            omega
        };
        let alpha_meas = (omega_filt - omega_filt_prev) / dt;
        omega_filt_prev = omega_filt;

        let er = theta.sin();
        let mut alpha_ref = -cfg.kr * er - cfg.kw * omega;
        if cfg.use_bw_ref {
            alpha_ref = bw_ref.update(alpha_ref);
        }

        let base_raw = rpm_hist[cfg.rpm_base_lag.min(rpm_hist.len() - 1)];
        let base = if cfg.use_bw_tau {
            bw_tau.update(base_raw)
        } else {
            base_raw
        };

        let mut tau_cmd = base + J * (alpha_ref - alpha_meas);
        if cfg.use_clamp {
            tau_cmd = tau_cmd.clamp(-cfg.tau_clamp, cfg.tau_clamp);
        }
        tau_hold = tau_cmd;

        let tau_delayed = if dead_n == 0 {
            tau_hold
        } else {
            let td = dead[dead_n - 1];
            for i in (1..dead_n).rev() {
                dead[i] = dead[i - 1];
            }
            dead[0] = tau_hold;
            td
        };

        let k_up = dt / (dt + cfg.tau_act);
        let k_down = dt / (dt + cfg.tau_act * cfg.spool_asym);
        let k_plant = if tau_delayed.abs() >= tau_applied.abs() {
            k_up
        } else {
            k_down
        };
        tau_applied += (tau_delayed - tau_applied) * k_plant;

        for i in (1..rpm_hist.len()).rev() {
            rpm_hist[i] = rpm_hist[i - 1];
        }
        rpm_hist[0] = tau_applied;

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
    BisectOut {
        omega_sigma: sigma,
        freq_hz: freq,
        limit_cycle: sigma > 0.5,
    }
}

fn find_kr_ceiling(base: BisectCfg) -> f32 {
    let mut lo = 400.0_f32;
    let mut hi = 3000.0_f32;
    while hi - lo > 50.0 {
        let mid = (lo + hi) * 0.5;
        let mut c = base;
        c.kr = mid;
        c.kw = mid * 170.0 / 2400.0;
        if run_bisect(c).limit_cycle {
            hi = mid;
        } else {
            lo = mid;
        }
    }
    (lo + hi) * 0.5
}

#[test]
fn bisect_baseline_limit_cycles() {
    let r = run_bisect(BisectCfg::default());
    println!("baseline sigma={:.3} freq={:.2}", r.omega_sigma, r.freq_hz);
    assert!(r.limit_cycle, "baseline should LC, sigma={}", r.omega_sigma);
}

#[test]
fn bisect_matrix_and_optional_json() {
    let base = BisectCfg::default();
    let mut rows: Vec<String> = Vec::new();
    let cases: &[(&str, fn(BisectCfg) -> BisectCfg)] = &[
        ("baseline", |c| c),
        ("no_dead", |mut c| {
            c.dead_ticks = 0;
            c
        }),
        ("dead_1", |mut c| {
            c.dead_ticks = 1;
            c
        }),
        ("no_rpm_lag", |mut c| {
            c.rpm_base_lag = 0;
            c
        }),
        ("no_bw", |mut c| {
            c.use_bw_pre = false;
            c.use_bw_ref = false;
            c.use_bw_tau = false;
            c
        }),
        ("no_clamp", |mut c| {
            c.use_clamp = false;
            c
        }),
        ("tau_0", |mut c| {
            c.tau_act = 0.0;
            c
        }),
        ("tau_20ms", |mut c| {
            c.tau_act = 0.020;
            c
        }),
        ("tau_60ms", |mut c| {
            c.tau_act = 0.060;
            c
        }),
        ("spool_125", |mut c| {
            c.spool_asym = 1.25;
            c
        }),
    ];
    for (name, f) in cases {
        let r = run_bisect(f(base));
        rows.push(format!(
            "{{\"case\":\"{name}\",\"sigma\":{:.4},\"freq_hz\":{:.3},\"limit_cycle\":{}}}",
            r.omega_sigma,
            r.freq_hz,
            r.limit_cycle
        ));
    }
    for &tau in &[0.0_f32, 0.020, 0.044, 0.060] {
        for dead in 0..=3 {
            let mut c = base;
            c.tau_act = tau;
            c.dead_ticks = dead;
            let r = run_bisect(c);
            rows.push(format!(
                "{{\"case\":\"grid_tau_dead\",\"tau\":{tau},\"dead_ticks\":{dead},\"sigma\":{:.4},\"freq_hz\":{:.3},\"limit_cycle\":{}}}",
                r.omega_sigma,
                r.freq_hz,
                r.limit_cycle
            ));
        }
    }
    let ceiling = find_kr_ceiling(base);
    rows.push(format!("{{\"case\":\"kr_ceiling\",\"kr_approx\":{ceiling:.0}}}"));

    if std::env::var("INDI_HARNESS_BISECT").ok().as_deref() == Some("1") {
        let out = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("../experiments/analysis/out/indi_harness_bisect/bisect.json");
        std::fs::create_dir_all(out.parent().unwrap()).ok();
        std::fs::write(&out, format!("[\n{}\n]", rows.join(",\n"))).expect("write");
        println!("wrote {}", out.display());
    }
}
