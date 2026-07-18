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
 * Max 12 segments → 228 floats (ci index 0..227).
 *
 * Z-axis buffer layout (per segment, 9 floats):
 *   [0..8]   = cz[0..8] — degree-8 polynomial for z offset [m], normalised
 * Max 12 segments → 108 floats (zci index 0..107).
 * z_mode = 0: z = traj.hz + traj.dz * lap_frac (constant + linear ramp, default).
 * z_mode = 1: z = traj.hz + poly_eval(cz) — full 3D trajectory (loop, etc.).
 *
 * Attitude buffer layout (per segment, 18 floats, planning Mode 2 only):
 *   [0..8]   = croll[0..8]  — degree-8 polynomial for roll  [rad], normalised
 *   [9..17]  = cpitch[0..8] — degree-8 polynomial for pitch [rad], normalised
 * Max 12 segments → 216 floats (aci index 0..215).
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
#define TRAJ_MAX_SEGS        12u
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
float   g_traj_origin_x  = 0.0f;  /* EKF x offset sampled before takeoff [m]             */
float   g_traj_origin_y  = 0.0f;  /* EKF y offset sampled before takeoff [m]             */
float   g_traj_hover_z   = 1.0f;  /* constant flight altitude [m]                        */
float   g_traj_dz        = 0.0f;  /* Z gain per lap [m]; 0=flat, +0.40=ascending helix   */

/* ── Position upload protocol ────────────────────────────────────────────── */
uint8_t g_traj_coef_ci   = 0;     /* coefficient index (0..227)                          */
float   g_traj_coef_cv   = 0.0f;  /* coefficient value to write at g_traj_coef_ci        */
uint8_t g_traj_coef_cw   = 0;     /* commit flag: laptop sets 1; Rust clears to 0        */

/* ── CRTP parameter group ────────────────────────────────────────────────── */
PARAM_GROUP_START(traj)
  PARAM_ADD(PARAM_UINT8, mode,     &g_traj_mode)
  PARAM_ADD(PARAM_UINT8, start,    &g_traj_start)
  PARAM_ADD(PARAM_UINT8, reps,     &g_traj_reps)
  PARAM_ADD(PARAM_UINT8, nseg,     &g_traj_n_segs)
  PARAM_ADD(PARAM_FLOAT, ox,       &g_traj_origin_x)
  PARAM_ADD(PARAM_FLOAT, oy,       &g_traj_origin_y)
  PARAM_ADD(PARAM_FLOAT, hz,       &g_traj_hover_z)
  PARAM_ADD(PARAM_FLOAT, dz,       &g_traj_dz)
  PARAM_ADD(PARAM_UINT8, ci,       &g_traj_coef_ci)
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
LOG_GROUP_STOP(indi)
