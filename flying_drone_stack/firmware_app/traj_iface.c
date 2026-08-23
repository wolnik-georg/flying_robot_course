/**
 * traj_iface.c — Trajectory upload interface for onboard spline evaluation.
 *
 * Declares coefficient buffers and CRTP parameters that allow the laptop to:
 *   1. Upload position coefficients via ci/cv/cw (one float at a time).
 *   2. Upload Z-axis coefficients via zci/zcv/zcw (3D trajectories only).
 *   3. Upload attitude coefficients via aci/acv/acw (planning Mode 2 only).
 *   4. Set trajectory metadata (origin, hover height, number of segments).
 *   5. Switch between Mode B (passthrough) and Mode D (onboard eval).
 *   6. Trigger the onboard trajectory by writing traj.start = 1.
 *
 * Position buffer layout (per segment, 19 floats):
 *   [0]      = duration  (s)
 *   [1..9]   = cx[0..8] — degree-8 polynomial for x, normalised to t∈[0,1]
 *   [10..18] = cy[0..8] — degree-8 polynomial for y, normalised to t∈[0,1]
 * Max 14 segments → 266 floats (ci index 0..265, u16).
 *
 * Z-axis buffer layout (per segment, 9 floats):
 *   [0..8]   = cz[0..8] — degree-8 polynomial for z offset [m], normalised
 * Max 14 segments → 126 floats (zci index 0..125).
 * z_mode = 0: z = traj.hz + traj.dz * lap_frac (constant + linear ramp, default).
 * z_mode = 1: z = traj.hz + poly_eval(cz) — full 3D trajectory (loop, etc.).
 *
 * Attitude buffer layout (per segment, 18 floats, planning Mode 2 only):
 *   [0..8]   = croll[0..8]  — degree-8 polynomial for roll  [rad], normalised
 *   [9..17]  = cpitch[0..8] — degree-8 polynomial for pitch [rad], normalised
 * Max 14 segments → 252 floats (aci index 0..251).
 * att_mode = 0: firmware derives attitude from flatness (default — Mode 0/1 behaviour).
 * att_mode = 1: firmware evaluates uploaded roll/pitch polynomials at 500 Hz.
 *
 * Upload protocol (same for pos, z, and att — different param names):
 *   set traj.ci/zci/aci = idx   (u8)
 *   set traj.cv/zcv/acv = value (f32)
 *   set traj.cw/zcw/acw = 1     (commit trigger; firmware clears to 0 within 2 ms)
 *
 * Mode switch:
 *   traj.mode = 0  → Mode B passthrough (laptop sends full-state setpoints)
 *   traj.mode = 1  → Mode D onboard eval (all trajectories: flat, helix, loop)
 *                    z_mode selects Z source: 0=ramp (flat/helix), 1=polynomial (loop)
 *
 * These symbols are declared extern in firmware_app/src/lib.rs and accessed there.
 */

#include "param.h"
#include "log.h"
#include <stdint.h>
#include <stdbool.h>

/* ── Position coefficient buffer ─────────────────────────────────────────── */
/* 14 = 12-segment periodic cores (oval/tilted_oval) + 1 entry + 1 exit segment
 * (Issue B rest-to-rest fix). ci is u16 (14*19 = 266 coefficients > u8 range);
 * zci/aci stay u8 (14*9 = 126, 14*18 = 252 — both fit).                       */
#define TRAJ_MAX_SEGS        14u
#define TRAJ_FLOATS_PER_SEG  19u   /* duration + cx[9] + cy[9] */

float   g_traj_coefs[TRAJ_MAX_SEGS * TRAJ_FLOATS_PER_SEG]; /* zero-initialised by C runtime */

/* ── Z-axis coefficient buffer (3D trajectories: loop, etc.) ─────────────── */
/* Layout per segment: [cz[9]] — 9 floats, no duration field.                  */
/* z_mode=0: z = hover_z + dz*lap_frac (linear ramp, circle/figure8/helix).   */
/* z_mode=1: z = hover_z + poly_eval(cz) — full 3D trajectory.                */
#define TRAJ_Z_FLOATS_PER_SEG    9u   /* cz[9] */

float   g_traj_z_coefs[TRAJ_MAX_SEGS * TRAJ_Z_FLOATS_PER_SEG]; /* zero-initialised */
uint8_t g_traj_z_mode    = 0;    /* 0 = ramp (default), 1 = polynomial          */

/* ── Z upload protocol ───────────────────────────────────────────────────── */
uint8_t g_traj_z_ci      = 0;     /* z coefficient index (0..107)               */
float   g_traj_z_cv      = 0.0f;  /* z coefficient value to write               */
uint8_t g_traj_z_cw      = 0;     /* commit flag: laptop sets 1; Rust clears    */

/* ── Attitude coefficient buffer (planning Mode 2 only) ─────────────────── */
/* Layout per segment: [croll[9], cpitch[9]] — 18 floats, no duration field.  */
/* att_mode=0: firmware uses flatness (unchanged Mode 0/1 behaviour).          */
/* att_mode=1: firmware evaluates these polynomials at 500 Hz.                 */
#define TRAJ_ATT_FLOATS_PER_SEG  18u  /* croll[9] + cpitch[9] */

float   g_traj_att_coefs[TRAJ_MAX_SEGS * TRAJ_ATT_FLOATS_PER_SEG]; /* zero-initialised */
uint8_t g_traj_att_mode  = 0;    /* 0 = derive from flatness (default), 1 = polynomial */

/* att_ctrl_mode controls HOW the polynomial is blended with feedback (when att_mode=1):
 *   0 = hard override: polynomial sets rd AND omega_d (both replaced by polynomial).
 *       Required for loop/flip where flatness is undefined at zero-thrust events.
 *   1 = hybrid: polynomial provides omega_d feedforward ONLY; rd is still derived from
 *       the total feedback force via flatness — position tracking errors are always
 *       reflected in the rotation command. Recommended for normal flat trajectories.
 */
uint8_t g_traj_att_ctrl_mode = 1; /* 1 = hybrid (default — stable for all flat trajectories) */

/* ── Attitude upload protocol ────────────────────────────────────────────── */
uint8_t g_traj_att_ci    = 0;     /* attitude coefficient index (0..215)                 */
float   g_traj_att_cv    = 0.0f;  /* attitude coefficient value to write at g_traj_att_ci */
uint8_t g_traj_att_cw    = 0;     /* commit flag: laptop sets 1; Rust clears to 0        */

/* ── Trajectory metadata ─────────────────────────────────────────────────── */
uint8_t g_traj_n_segs    = 0;     /* number of valid segments in g_traj_coefs  */
uint8_t g_traj_mode      = 0;     /* 0 = passthrough (Mode B), 1 = onboard eval (Mode D) */
uint8_t g_traj_start     = 0;     /* laptop writes 1 to start; firmware latches T0        */
uint8_t g_traj_reps      = 0;     /* 0 = loop forever; N = self-stop after N laps         */
/* Rest-to-rest entry/exit (Issue B fix): the first n_entry segments play ONCE before
 * the core laps, the last n_exit segments play ONCE after them. The core is the middle
 * region and is what traj.reps counts. 0/0 = legacy behaviour (wrap all segments). */
uint8_t g_traj_n_entry   = 0;     /* leading segments played once (0 = none)             */
uint8_t g_traj_n_exit    = 0;     /* trailing segments played once (0 = none)            */
float   g_traj_origin_x  = 0.0f;  /* EKF x offset sampled before takeoff [m]             */
float   g_traj_origin_y  = 0.0f;  /* EKF y offset sampled before takeoff [m]             */
float   g_traj_hover_z   = 1.0f;  /* constant flight altitude [m]                        */
float   g_traj_dz        = 0.0f;  /* Z gain per lap [m]; 0=flat, +0.40=ascending helix   */

/* ── Position upload protocol ────────────────────────────────────────────── */
uint16_t g_traj_coef_ci  = 0;     /* coefficient index (0..265) — u16, see TRAJ_MAX_SEGS */
float   g_traj_coef_cv   = 0.0f;  /* coefficient value to write at g_traj_coef_ci        */
uint8_t g_traj_coef_cw   = 0;     /* commit flag: laptop sets 1; Rust clears to 0        */

/* ── Residual-network weight upload ──────────────────────────────────────────
 * Same index / value / commit shape as the trajectory upload above, for the same reason:
 * CRTP parameters are the only writable channel that needs no new packet type.
 *
 * The count is declared UP FRONT (rnn.n) and checked on commit. A half-arrived network is
 * the dangerous failure -- it produces plausible numbers rather than an obvious fault, and
 * would be indistinguishable from a badly trained model. Refusing an incomplete upload turns
 * a dropped packet into "compensation off", which is diagnosable from the logs.        */
uint16_t g_rnn_wi  = 0;    /* weight index                                              */
float    g_rnn_wv  = 0.0f; /* weight value to write at g_rnn_wi                          */
uint8_t  g_rnn_wc  = 0;    /* commit flag: host sets 1; Rust clears to 0                 */
uint16_t g_rnn_n   = 0;    /* number of weights the host intends to send                 */
uint8_t  g_rnn_begin = 0;  /* host sets 1 to start a fresh upload; Rust clears           */
uint8_t  g_rnn_end   = 0;  /* host sets 1 when done; Rust validates and clears           */
uint8_t  g_rnn_en    = 0;  /* 0 = predict but do not use; 1 = feed into the controller    */
uint8_t  g_rnn_ready = 0;  /* read-only: 1 once a complete, finite weight set is loaded   */

/* Logged so the prediction can be compared against the measured residual in flight --
 * that comparison IS the evaluation of every learned method in this thesis.            */
float    g_rnn_pred_x = 0.0f;
float    g_rnn_pred_y = 0.0f;
float    g_rnn_pred_z = 0.0f;
uint8_t  g_rnn_clamped = 0;  /* last evaluation hit the output clamp -- a fault, not a result */

/* Peer positions for the residual model.
 *
 * Crazyswarm2 broadcasts every vehicle's pose and the firmware already stores the ones that
 * are not its own, so the network's main input costs no new communication. Note the API
 * offers POSITION ONLY -- there is no peer velocity -- so relative velocity is differenced
 * onboard from the timestamps the same struct carries. */
#ifdef CRAZYFLIE_FW   /* peer_localization is drone-only; the host simulator injects peers instead */
#include "peer_localization.h"
uint8_t peer_get_all(float *xs, float *ys, float *zs, uint32_t *ts, uint8_t max)
{
  uint8_t n = 0;
  for (uint8_t i = 0; i < max; ++i) {
    peerLocalizationOtherPosition_t *p = peerLocalizationGetPositionByIdx(i);
    if (p == NULL) break;
    xs[n] = p->pos.x; ys[n] = p->pos.y; zs[n] = p->pos.z; ts[n] = p->pos.timestamp;
    ++n;
  }
  return n;
}
#endif

void rnn_pred_write(float x, float y, float z, uint8_t clamped)
{
  g_rnn_pred_x = x; g_rnn_pred_y = y; g_rnn_pred_z = z; g_rnn_clamped = clamped;
}

PARAM_GROUP_START(rnn)
  PARAM_ADD(PARAM_UINT16, wi,    &g_rnn_wi)
  PARAM_ADD(PARAM_FLOAT,  wv,    &g_rnn_wv)
  PARAM_ADD(PARAM_UINT8,  wc,    &g_rnn_wc)
  PARAM_ADD(PARAM_UINT16, n,     &g_rnn_n)
  PARAM_ADD(PARAM_UINT8,  begin, &g_rnn_begin)
  PARAM_ADD(PARAM_UINT8,  end,   &g_rnn_end)
  PARAM_ADD(PARAM_UINT8,  en,    &g_rnn_en)
  PARAM_ADD(PARAM_UINT8,  ready, &g_rnn_ready)
PARAM_GROUP_STOP(rnn)

LOG_GROUP_START(rnn)
  LOG_ADD(LOG_FLOAT, pred_x,  &g_rnn_pred_x)
  LOG_ADD(LOG_FLOAT, pred_y,  &g_rnn_pred_y)
  LOG_ADD(LOG_FLOAT, pred_z,  &g_rnn_pred_z)
  LOG_ADD(LOG_UINT8, clamped, &g_rnn_clamped)
  LOG_ADD(LOG_UINT8, ready,   &g_rnn_ready)
LOG_GROUP_STOP(rnn)

/* ── CRTP parameter group ────────────────────────────────────────────────── */
PARAM_GROUP_START(traj)
  PARAM_ADD(PARAM_UINT8, mode,     &g_traj_mode)
  PARAM_ADD(PARAM_UINT8, start,    &g_traj_start)
  PARAM_ADD(PARAM_UINT8, reps,     &g_traj_reps)
  PARAM_ADD(PARAM_UINT8, n_entry,  &g_traj_n_entry)
  PARAM_ADD(PARAM_UINT8, n_exit,   &g_traj_n_exit)
  PARAM_ADD(PARAM_UINT8, nseg,     &g_traj_n_segs)
  PARAM_ADD(PARAM_FLOAT, ox,       &g_traj_origin_x)
  PARAM_ADD(PARAM_FLOAT, oy,       &g_traj_origin_y)
  PARAM_ADD(PARAM_FLOAT, hz,       &g_traj_hover_z)
  PARAM_ADD(PARAM_FLOAT, dz,       &g_traj_dz)
  PARAM_ADD(PARAM_UINT16, ci,      &g_traj_coef_ci)
  PARAM_ADD(PARAM_FLOAT, cv,       &g_traj_coef_cv)
  PARAM_ADD(PARAM_UINT8, cw,       &g_traj_coef_cw)
  PARAM_ADD(PARAM_UINT8, z_mode,   &g_traj_z_mode)
  PARAM_ADD(PARAM_UINT8, zci,      &g_traj_z_ci)
  PARAM_ADD(PARAM_FLOAT, zcv,      &g_traj_z_cv)
  PARAM_ADD(PARAM_UINT8, zcw,      &g_traj_z_cw)
  PARAM_ADD(PARAM_UINT8, att_mode,      &g_traj_att_mode)
  PARAM_ADD(PARAM_UINT8, att_ctrl_mode, &g_traj_att_ctrl_mode)
  PARAM_ADD(PARAM_UINT8, aci,           &g_traj_att_ci)
  PARAM_ADD(PARAM_FLOAT, acv,           &g_traj_att_cv)
  PARAM_ADD(PARAM_UINT8, acw,           &g_traj_att_cw)
PARAM_GROUP_STOP(traj)

/* ── INDI tuning params (runtime-configurable, no reflash needed) ─────────── */
/* Change via CS2 yaml firmware_params.indi_gains.*, cfclient Parameters tab,   */
/* or a Python one-liner: cf.param.set_value('indi_gains.kr', '200.0').         */
uint8_t g_controller_mode = 0;   /* 0=geometric, 1=pos INDI, 2=att INDI, 3=full INDI */
float g_indi_kr     = 100.0f;    /* KR_INDI X/Y [1/s^2]  — wn = sqrt(KR)  */
float g_indi_kw     = 30.0f;     /* KW_INDI X/Y [1/s]    — zeta = KW/(2*wn) */
float g_indi_kr_z   = 100.0f;    /* KR_INDI Z   [1/s^2]  */
float g_indi_kw_z   = 30.0f;     /* KW_INDI Z   [1/s]    */
// float g_indi_kt1    = 1.1421e-10f; /* KT_MOTOR M1 [N/RPM^2] — from hover log 2026-06-10 */
// float g_indi_kt2    = 1.1421e-10f; /* KT_MOTOR M2 [N/RPM^2] */
// float g_indi_kt3    = 1.1421e-10f; /* KT_MOTOR M3 [N/RPM^2] */
// float g_indi_kt4    = 1.1421e-10f; /* KT_MOTOR M4 [N/RPM^2] */
float g_indi_kt1    = 1.4811e-10f; /* KT_MOTOR M1 [N/RPM²] — Lee hover 2026-06-18_20-22-55, m=36.4g */
float g_indi_kt2    = 1.4815e-10f;
float g_indi_kt3    = 1.5319e-10f;
float g_indi_kt4    = 1.6119e-10f;
float g_indi_fc_bw  = 60.0f;     /* Butterworth cutoff [Hz] — applied to alpha_raw and alpha_ref */
float g_indi_fc_iir = 60.0f;     /* legacy (unused — kept for backwards-compat with old yaml files) */
float g_indi_mass   = 0.0364f;   /* all-up mass [kg] — CF2.1 + USD + RPM, measured 36.4g */
/* H1a diagnostic (2026-07-18): force tau_current = tau_prev even when the RPM deck IS active,
   to isolate whether the ~7-8 Hz tau_x/alp_x limit cycle comes from the RPM-feedback path itself
   or from motor-mechanical lag (persists either way). 0 = normal (RPM feedback when available). */
uint8_t g_indi_ff_free = 0;

/* Filter order for the INDI angular-acceleration measurement chain (2026-07-18).
   0 = legacy: diff(raw omega) -> Butterworth -> alpha_meas. Today's exact behaviour,
       byte-identical -- standard/upgraded validated gains and results are unaffected.
   1 = paper order: Butterworth(raw omega) -> diff -> alpha_meas. Matches Tal & Karaman 2021
       Sec. III-D and stock crazyflie-firmware controller_indi.c (filter_pqr then
       finite_difference_from_filter) -- verified against both papers and the stock C
       implementation before adding this, not a guess. Default OFF; enable per-drone in yaml. */
uint8_t g_indi_filt_order = 0;

/* Increment base filter (2026-07-18). 0 = legacy: tau_current (mu_f) used unfiltered in the INDI
   increment tau = tau_current + J*(alpha_ref - alpha_meas). Today's exact behaviour.
   1 = filter tau_current with the SAME Butterworth as alpha_meas so the two increment terms are
   phase-matched -- Tal & Karaman 2021 Eq. 29/31 (mu_f is FILTERED) and stock crazyflie-firmware
   controller_indi.c (increment base indi.u[0].o[0] is the filter_pqr'd command). This is the
   candidate root-cause fix for the ~5-8 Hz attitude limit cycle. Default OFF; enable per-drone. */
uint8_t g_indi_filt_tau = 0;

/* INDI effectiveness scale (2026-07-18). Multiplies the J in the increment delta_tau = j_scale*J*
   (alpha_ref - alpha_meas). 1.0 = use J as-is (default, byte-identical). <1.0 = reduce the
   effectiveness estimate. The brushless J (Busetto 2025) is a Crazyflow SIMULATOR DEFAULT for a
   45 g drone under a GEOMETRIC controller -- not system-ID'd, not for this 41 g drone, not
   validated for INDI. If J is too high the INDI attitude loop over-corrects -> ~5-8 Hz limit
   cycle. Sweep down (1.0 -> 0.8 -> 0.65 -> 0.5) at runtime (no reflash) to find where it stops. */
float g_indi_j_scale = 1.0f;

/* Output clamps (2026-07-22, mirroring stock C INDI which clamps u_in ±32000 motor units and
   the outer-loop tilt ±pq_clamp / thrust [MIN,MAX]; we previously clamped NOTHING).
   Robustness/blow-up containment — NOT a fix for the 6.3 Hz cycle (which lives ~96% inside the
   clamp-free region, see docs/investigation §15.10). clamp_en is a bitmask so individual clamps
   can be disabled for aggressive maneuvers: bit0=tau_xy, bit1=tau_z, bit2=tilt, bit3=thrust.
   Phase-4 inverted/loop flight: clear bit2 (tilt >90 deg is intended) but KEEP bit0/bit1
   (motor physics is orientation-independent). Default 0 = all off = byte-identical behaviour;
   enable per-drone in yaml. Values sized from per-motor thrust headroom at hover:
   tau_xy_max = ARM*4*dF, tau_z_max = TORQUE_RATIO*4*dF (CF21BL dF≈0.1 N -> 14 / 2.5 mNm;
   standard ~7/1.2 mNm, upgraded ~10/1.7 mNm by the same formula). */
/* INDI increment-base source (2026-07-22, bench §16 follow-up). 0.0 (default) = RPM base
 * (today's behaviour, byte-identical). >0 = act_dyn base: first-order model of the previous
 * (clamped) torque commands with this time constant [s], replacing the RPM reading — the
 * stock controller_indi.c structure. NOTE the simulator predicts a matched model (0.044 =
 * bench-measured tau) behaves IDENTICALLY to the RPM base (both equal the applied torque);
 * this knob exists to verify that on hardware and to probe model-mismatch effects. */
float   g_indi_act_tau      = 0.0f;

uint8_t g_indi_clamp_en     = 0;       /* bitmask, 0 = all clamps off (default)          */
float   g_indi_tau_xy_max   = 0.014f;  /* [Nm]  roll/pitch torque clamp (CF21BL default) */
float   g_indi_tau_z_max    = 0.0025f; /* [Nm]  yaw torque clamp        (CF21BL default) */
float   g_indi_tilt_max_deg = 30.0f;   /* [deg] max desired-thrust-vector tilt from vertical */
float   g_indi_thrust_max   = 0.8f;    /* [N]   total thrust clamp (4x THRUST_MAX cf21bl)    */

/* Optional stage-2 notch (band-reject) filter (2026-07-28, shake investigation). The stage-1
 * Butterworth (fc_bw above) sits at 60-70 Hz and measurably does NOT touch the 5-10 Hz shake band
 * (|alpha_filtered|/|alpha_raw|=1.000 at 7.22 Hz, 500 Hz SD-log analysis) -- and lowering fc_bw
 * down into that band was tried and made things 3-4x worse (adds phase lag everywhere, not just
 * at the shake frequency). This filter instead targets ONLY the diagnosed band (RBJ Audio-EQ-
 * Cookbook band-reject biquad), applied identically to both alpha_meas and alpha_ref_filt so the
 * two stay phase-matched (mirrors why fc_bw already applies to both). Default OFF = byte-identical
 * to prior behaviour. f0/bw are runtime-tunable [Hz] so the notch can be re-centred without a
 * reflash if the shake frequency drifts (6.3-7.9 Hz measured across different sessions). */
/* omega_src — where the passthrough branch (g_traj_mode==0: HLC/Mode E, cmdFullState) takes
 * the desired body rate omega_d from. The Crazyflie HLC builds its body frame Mellinger-style
 * (y_B = z_B x x_C) in pptraj.c, while our controller's desired_rot/omega_desired build it
 * Faessler-style (x_B = y_C x z_B). The two agree only when the tilt is axis-aligned, so
 * trusting setpoint.attitudeRate gives an omega_d that is inconsistent with the R_d the
 * controller itself uses (measured 0.04-0.22 rad/s on level trajectories; far larger through
 * zero-thrust). Mode D never had this problem because it computes omega_d locally.
 *   0 = setpoint.attitudeRate  (default — unchanged behaviour)
 *   1 = recompute from HLC jerk via omega_desired() — matches Mode D's convention
 * Falls back to attitudeRate automatically when jerk is absent (cmdFullState memsets it),
 * so Mode B / hover keepalive are unaffected either way. */
uint8_t g_indi_omega_src = 0;   /* 0 = setpoint rate (default), 1 = flatness from HLC jerk */

/* frame_conv — which body-frame construction omega_desired()/alpha_desired() use when
 * turning jerk/snap into omega_d / alpha_des. z_B is unambiguous (thrust direction); the
 * choice is only how yaw is placed around it, and the two agree only at axis-aligned tilt.
 *   1 = Mellinger (DEFAULT): y_B = normalize(z_B x x_C), x_B = y_B x z_B.
 *       Matches desired_rot() (which builds R_d), the official controller_lee.c and
 *       controller_mellinger.c, and the HLC's pptraj.c -- i.e. everything else on this
 *       platform. Makes R_d, omega_d and alpha_des share one frame, and makes the locally
 *       computed omega_d identical to the HLC's (so omega_src stops mattering).
 *   0 = Faessler et al. 2018 App. A: x_B = normalize(y_C x z_B), y_B = normalize(z_B x x_B).
 *       What every flight before 2026-08-22 used for omega_d/alpha_des, while R_d was
 *       already Mellinger -- i.e. the old behaviour was internally mixed. Kept so that
 *       pre-existing results can be reproduced exactly.
 * omega_d feeds the KW damping term of BOTH the geometric and INDI attitude laws;
 * alpha_des feeds the INDI snap feedforward only. Measured effect of the switch on level
 * trajectories: omega_d 0.002-0.17 rad/s, alpha_des ~1-1.5%.
 *
 * SCOPE: this applies to the passthrough branch (Mode E / HLC) ONLY. Mode D (onboard
 * trajectory eval) is pinned to Faessler in lib.rs via FRAME_CONV_MODE_D and ignores this
 * parameter, so the completed INDI project's flight results stay reproducible. */
uint8_t g_indi_frame_conv = 1;  /* 1 = Mellinger (default), 0 = Faessler (legacy) */

uint8_t g_indi_notch_en = 0;    /* 0 = off (default), 1 = on */
float   g_indi_notch_f0 = 7.2f; /* notch center frequency [Hz] */
float   g_indi_notch_bw = 5.0f; /* notch bandwidth [Hz] (Q = f0/bw) */

PARAM_GROUP_START(indi_gains)
  PARAM_ADD(PARAM_UINT8, ctrl_mode, &g_controller_mode)
  PARAM_ADD(PARAM_FLOAT, kr,     &g_indi_kr)
  PARAM_ADD(PARAM_FLOAT, kw,     &g_indi_kw)
  PARAM_ADD(PARAM_FLOAT, kr_z,   &g_indi_kr_z)
  PARAM_ADD(PARAM_FLOAT, kw_z,   &g_indi_kw_z)
  PARAM_ADD(PARAM_FLOAT, kt1,    &g_indi_kt1)
  PARAM_ADD(PARAM_FLOAT, kt2,    &g_indi_kt2)
  PARAM_ADD(PARAM_FLOAT, kt3,    &g_indi_kt3)
  PARAM_ADD(PARAM_FLOAT, kt4,    &g_indi_kt4)
  PARAM_ADD(PARAM_FLOAT, fc_bw,  &g_indi_fc_bw)
  PARAM_ADD(PARAM_FLOAT, fc_iir, &g_indi_fc_iir)
  PARAM_ADD(PARAM_FLOAT, mass,   &g_indi_mass)
  PARAM_ADD(PARAM_UINT8, ff_free, &g_indi_ff_free)
  PARAM_ADD(PARAM_UINT8, filt_order, &g_indi_filt_order)
  PARAM_ADD(PARAM_UINT8, filt_tau, &g_indi_filt_tau)
  PARAM_ADD(PARAM_FLOAT, j_scale, &g_indi_j_scale)
  PARAM_ADD(PARAM_FLOAT, act_tau,      &g_indi_act_tau)
  PARAM_ADD(PARAM_UINT8, clamp_en,     &g_indi_clamp_en)
  PARAM_ADD(PARAM_FLOAT, tau_xy_max,   &g_indi_tau_xy_max)
  PARAM_ADD(PARAM_FLOAT, tau_z_max,    &g_indi_tau_z_max)
  PARAM_ADD(PARAM_FLOAT, tilt_max_deg, &g_indi_tilt_max_deg)
  PARAM_ADD(PARAM_FLOAT, thrust_max,   &g_indi_thrust_max)
  PARAM_ADD(PARAM_UINT8, omega_src,    &g_indi_omega_src)
  PARAM_ADD(PARAM_UINT8, frame_conv,   &g_indi_frame_conv)
  PARAM_ADD(PARAM_UINT8, notch_en,     &g_indi_notch_en)
  PARAM_ADD(PARAM_FLOAT, notch_f0,     &g_indi_notch_f0)
  PARAM_ADD(PARAM_FLOAT, notch_bw,     &g_indi_notch_bw)
PARAM_GROUP_STOP(indi_gains)

/* ── Position loop gains (runtime-tunable, no reflash needed) ─────────────── */
float g_kp_xy = 28.0f;  /* position P gain X/Y [m/s² per m]  */
float g_kp_z  = 30.0f;  /* position P gain Z                  */
float g_kv_xy = 3.0f;   /* velocity D gain X/Y [m/s² per m/s] */
float g_kv_z  = 7.0f;   /* velocity D gain Z                  */

PARAM_GROUP_START(pos_gains)
  PARAM_ADD(PARAM_FLOAT, kp_xy, &g_kp_xy)
  PARAM_ADD(PARAM_FLOAT, kp_z,  &g_kp_z)
  PARAM_ADD(PARAM_FLOAT, kv_xy, &g_kv_xy)
  PARAM_ADD(PARAM_FLOAT, kv_z,  &g_kv_z)
PARAM_GROUP_STOP(pos_gains)

/* ── RPM bridge for Rust INDI (Mode 1) ─────────────────────────────────── */
/* Exposes per-motor RPM via the Crazyflie log system.                       */
/* m1rpm–m4rpm are file-static in rpm.c; this bridge uses logGetVarId() to  */
/* locate them at runtime (lazy init) and logGetUint() to read them.         */
/* Returns 0 for each motor when the RPM deck is absent or not yet registered.*/

void rpm_get_all(uint16_t *m1, uint16_t *m2, uint16_t *m3, uint16_t *m4)
{
    static logVarId_t ids[4] = {0xffffu, 0xffffu, 0xffffu, 0xffffu};
#ifdef CONFIG_PLATFORM_CF21BL
    /* EXPERIMENT (2026-07-16): read the optical RPM deck (group "rpm", m1..m4) on the
       brushless too — exactly like the brushed drones — to test whether the DShot
       telemetry delay is what makes attitude INDI limit-cycle at ~7 Hz. Both the deck
       and DShot report *mechanical* RPM, so kt stays the same scale (re-ID as a check).
       To revert to bidirectional-DShot ESC telemetry, swap the two lines below back. */
    static const char *group    = "rpm";
    static const char *names[4] = {"m1", "m2", "m3", "m4"};
    /* DShot (default brushless source — restore to switch back):
    static const char *group    = "motor";
    static const char *names[4] = {"m1_rpm", "m2_rpm", "m3_rpm", "m4_rpm"}; */
#else
    /* CF2.1 brushed: optical RPM deck (log group "rpm", vars m1..m4). */
    static const char *group    = "rpm";
    static const char *names[4] = {"m1", "m2", "m3", "m4"};
#endif

    for (int i = 0; i < 4; i++) {
        if (!logVarIdIsValid(ids[i]))
            ids[i] = logGetVarId(group, names[i]);
    }

    uint16_t v[4];
    for (int i = 0; i < 4; i++) {
        v[i] = logVarIdIsValid(ids[i]) ? (uint16_t)logGetUint(ids[i]) : 0u;
        /* DShot telemetry uses UINT16_MAX as the "invalid / no value" sentinel;
           treat it as absent so INDI falls back instead of using garbage RPM.
           (CF2.1 optical RPM never reaches this value, so this is a no-op there.) */
        if (v[i] == 0xffffu) v[i] = 0u;
    }
    *m1 = v[0]; *m2 = v[1]; *m3 = v[2]; *m4 = v[3];
}

/* ── INDI log bridge ────────────────────────────────────────────────────── */
static float log_alp_raw_x, log_alp_raw_y, log_alp_raw_z;
static float log_alp_x,     log_alp_y,     log_alp_z;
static float log_tau_x,     log_tau_y,     log_tau_z;
static float log_alp_notch_x, log_alp_notch_y, log_alp_notch_z;
static float log_a_res_x, log_a_res_y, log_a_res_z;

void indi_log_write(float arx, float ary, float arz,
                    float ax,  float ay,  float az)
{
    log_alp_raw_x = arx; log_alp_raw_y = ary; log_alp_raw_z = arz;
    log_alp_x = ax;      log_alp_y = ay;      log_alp_z = az;
}

void indi_tau_write(float tx, float ty, float tz)
{
    log_tau_x = tx; log_tau_y = ty; log_tau_z = tz;
}

/* Stage-2 notch output (indi_gains.notch_en, 2026-07-28) -- alpha_meas AFTER the optional
 * band-reject filter, always computed/logged regardless of notch_en so alp_raw / alp (stage-1 BW)
 * / alp_notch (stage-2 notch) can all be compared from one flight. */
void indi_notch_log_write(float anx, float any, float anz)
{
    log_alp_notch_x = anx; log_alp_notch_y = any; log_alp_notch_z = anz;
}

/* Residual acceleration a_res = a_meas - a_model [m/s^2], world frame (2026-08-22).
 * Equals f_res/m -- the unmodelled force on the vehicle, dominated by other drones' downwash in
 * close-proximity formation flight. This is the quantity the residual-learning literature trains
 * on, so it is the core measurement of the thesis.
 * Written every tick whenever RPMs are available, REGARDLESS of indi_gains.ctrl_mode, so the
 * residual can be recorded while flying the plain geometric controller (training data for
 * Geometric+NN must come from non-INDI flights). Zero when no RPM source is present. */
void indi_a_res_write(float ax, float ay, float az)
{
    log_a_res_x = ax; log_a_res_y = ay; log_a_res_z = az;
}

LOG_GROUP_START(indi)
  LOG_ADD(LOG_FLOAT, alp_raw_x, &log_alp_raw_x)
  LOG_ADD(LOG_FLOAT, alp_raw_y, &log_alp_raw_y)
  LOG_ADD(LOG_FLOAT, alp_raw_z, &log_alp_raw_z)
  LOG_ADD(LOG_FLOAT, alp_x,     &log_alp_x)
  LOG_ADD(LOG_FLOAT, alp_y,     &log_alp_y)
  LOG_ADD(LOG_FLOAT, alp_z,     &log_alp_z)
  LOG_ADD(LOG_FLOAT, tau_x,     &log_tau_x)
  LOG_ADD(LOG_FLOAT, tau_y,     &log_tau_y)
  LOG_ADD(LOG_FLOAT, tau_z,     &log_tau_z)
  LOG_ADD(LOG_FLOAT, alp_notch_x, &log_alp_notch_x)
  LOG_ADD(LOG_FLOAT, alp_notch_y, &log_alp_notch_y)
  LOG_ADD(LOG_FLOAT, alp_notch_z, &log_alp_notch_z)
  LOG_ADD(LOG_FLOAT, a_res_x,   &log_a_res_x)
  LOG_ADD(LOG_FLOAT, a_res_y,   &log_a_res_y)
  LOG_ADD(LOG_FLOAT, a_res_z,   &log_a_res_z)
LOG_GROUP_STOP(indi)
